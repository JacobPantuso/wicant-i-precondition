// Walk-away auto-lock; see autolock.h and docs/plans/wican-disconnect-auto-lock.md.
//
// Arming: the car leaving READY (0x038, tracked by precondition.c) after a
// drive. car_in_ready_fresh() also counts a 0x038 that stopped repeating as
// off, since the bus can sleep before the "off" frame arrives.
//
// Once armed, whichever comes first locks:
//  - Signal: the phone's link RSSI, read here every 500 ms and smoothed as
//    the app's AutoLockDecider does (median of 3, then a 0.6 moving
//    average), at or below the line the app sets (ATXALT) for 3 s, or 6 dB
//    past it for 1 s. It runs on the WiCAN, so it works while iOS has the
//    app suspended.
//  - Disconnect: the link drops and stays gone for 5 s.
//
// Guards: never while the car is in READY. Doors and seats live on the
// diagnostic bus, out of reach here, so the app forwards its OBD reading as
// a hold (ATXALB1). The hold lapses after 30 s unless refreshed, so an app
// iOS has suspended can't leave it stuck.
//
// After a lock, coming back to the car arms it again: the phone's signal at
// least 10 dB above the line for 2 s while the car stays off. Opening the
// car to grab something and walking off again locks again. Coming back never
// locks by itself.
//
// Execution goes through bodycmd's lock burst as a local request: bodycmd
// drops client requests once their link is gone. Verification is the burst
// itself, the frame counter advancing by the 11-frame sequence, so "locked"
// means "lock sequence on the bus", the ATXLK1 semantics the app trusts.

#include "autolock.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"

#include "ble.h"
#include "bodycmd.h"
#include "precondition.h"
#include "types.h"
#include "unlockauth.h"

#define NVS_NAMESPACE   "autolock"
#define NVS_ENABLED     "en"
#define NVS_COUNT       "count"
#define NVS_LAST        "last"
#define NVS_THRESHOLD   "thr"

#define HOLD_US         (5LL * 1000 * 1000)       // disconnect hold
#define VERIFY_US       (2LL * 1000 * 1000)       // burst frames must all land
#define ARMED_WINDOW_US (12LL * 3600 * 1000 * 1000)
#define LOCK_BURST_FRAMES 11                      // bodycmd: 4 prepare + 4 execute + 3 finish

#define RSSI_PERIOD_US  (500LL * 1000)
#define SIGNAL_HOLD_US  (3LL * 1000 * 1000)
#define QUICK_HOLD_US   (1LL * 1000 * 1000)
#define QUICK_MARGIN_DB 6
#define NEAR_MARGIN_DB  10
#define NEAR_HOLD_US    (2LL * 1000 * 1000)
#define BLOCK_MS        30000U
#define SMOOTHING       0.6f
#define THRESHOLD_MIN   (-100)
#define THRESHOLD_MAX   (-40)

typedef enum {
    RESULT_NONE = 0,
    RESULT_LOCKED,
    RESULT_REJECTED,
    RESULT_UNCONFIRMED,
} result_t;

static const char *TAG = "autolock";

static bool enabled = true;
static bool armed;
static int64_t armed_at_us;
static bool ready_seen;            // READY rose since the last drive end
static bool parked;                // a drive ended; the car hasn't been in READY since
static int64_t hold_until_us;      // 0 = no disconnect hold running
static int64_t verify_until_us;    // 0 = no burst verification running
static uint32_t frames_at_request;
static result_t last = RESULT_NONE;
static uint32_t count;
static bool ready_unavailable_logged;

static int8_t threshold = -85;
static bool connected;
static int64_t rssi_next_us;
static volatile int latest_rssi;   // written by the BTC task
static volatile uint32_t rssi_seq;
static uint32_t rssi_seq_seen;
static int window[3];
static int window_len;
static float smoothed;
static bool smoothed_valid;
static int64_t away_since;
static int64_t quick_since;
static int64_t near_since;
static volatile uint32_t hold_until_ms; // the app's hold; 0 = none
static bool was_blocked;

static QueueHandle_t *tx_queue;

static const char *result_name(result_t r)
{
    switch (r) {
    case RESULT_LOCKED:      return "locked";
    case RESULT_REJECTED:    return "rejected";
    case RESULT_UNCONFIRMED: return "unconfirmed";
    default:                 return "none";
    }
}

static void persist(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    nvs_set_u8(handle, NVS_ENABLED, enabled ? 1 : 0);
    nvs_set_u32(handle, NVS_COUNT, count);
    nvs_set_u8(handle, NVS_LAST, (uint8_t)last);
    nvs_set_i8(handle, NVS_THRESHOLD, threshold);
    nvs_commit(handle);
    nvs_close(handle);
}

void autolock_init(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return;   // nothing stored yet: defaults stand
    }
    uint8_t u8;
    uint32_t u32;
    int8_t i8;
    if (nvs_get_u8(handle, NVS_ENABLED, &u8) == ESP_OK) {
        enabled = u8 != 0;
    }
    if (nvs_get_u32(handle, NVS_COUNT, &u32) == ESP_OK) {
        count = u32;
    }
    if (nvs_get_u8(handle, NVS_LAST, &u8) == ESP_OK && u8 <= RESULT_UNCONFIRMED) {
        last = (result_t)u8;
    }
    if (nvs_get_i8(handle, NVS_THRESHOLD, &i8) == ESP_OK
            && i8 >= THRESHOLD_MIN && i8 <= THRESHOLD_MAX) {
        threshold = i8;
    }
    nvs_close(handle);
    ESP_LOGI(TAG, "walk-away lock %s at %d dBm, %lu locks so far, last %s",
             enabled ? "enabled" : "disabled", threshold,
             (unsigned long)count, result_name(last));
}

void autolock_set_tx_queue(QueueHandle_t *queue)
{
    tx_queue = queue;
}

static void report(char *response, size_t response_len)
{
    snprintf(response, response_len, "XAL:%u,%s,%lu,%d,%d",
             armed ? 1 : 0,
             result_name(last),
             (unsigned long)count,
             threshold,
             smoothed_valid ? (int)smoothed : 0);
}

// The app hears about a lock at once while the phone is still connected;
// otherwise it reads the count on the next connect.
static void push_report(void)
{
    static xdev_buffer buffer;
    if (tx_queue == NULL || *tx_queue == NULL || !connected) {
        return;
    }
    char line[48];
    report(line, sizeof(line));
    int len = snprintf((char *)buffer.ucElement, sizeof(buffer.ucElement), "$%s\r", line);
    if (len <= 0) {
        return;
    }
    if (len >= (int)sizeof(buffer.ucElement)) {
        len = sizeof(buffer.ucElement) - 1;
    }
    buffer.usLen = len;
    buffer.dev_channel = DEV_BLE;
    xQueueSend(*tx_queue, &buffer, 0);
}

static void reset_signal(void)
{
    window_len = 0;
    smoothed_valid = false;
    away_since = 0;
    quick_since = 0;
    near_since = 0;
}

static bool app_holds(void)
{
    uint32_t until = hold_until_ms;
    if (until == 0) {
        return false;
    }
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    return (int32_t)(until - now) > 0;
}

void autolock_note_connected(void)
{
    connected = true;
    reset_signal();
    if (hold_until_us != 0) {
        ESP_LOGI(TAG, "client reconnected during the hold; not locking");
        hold_until_us = 0;
    }
}

void autolock_note_disconnected(void)
{
    connected = false;
    reset_signal();
    if (!enabled || !armed || !unlockauth_has_key()
            || hold_until_us != 0 || verify_until_us != 0) {
        return;
    }
    hold_until_us = esp_timer_get_time() + HOLD_US;
    ESP_LOGI(TAG, "key-holder disconnected while armed; locking in 5 s unless it returns");
}

void autolock_note_rssi(int rssi)
{
    latest_rssi = rssi;
    rssi_seq++;
}

static void fire(const char *why)
{
    bodycmd_status_t status;
    bodycmd_get_status(&status);
    frames_at_request = status.frames;
    bodycmd_request_lock_local();
    verify_until_us = esp_timer_get_time() + VERIFY_US;
    ESP_LOGI(TAG, "firing walk-away lock: %s", why);
}

// One link RSSI reading while parked: smooth it, then lock when armed and
// the phone is away, or arm again when it's back after a lock.
static void sample(int rssi, int64_t now)
{
    bool blocked = app_holds();
    // A hold just lifted (door shut): start the smoothing over, so readings
    // from standing at the open door don't drag the average.
    if (was_blocked && !blocked) {
        reset_signal();
    }
    was_blocked = blocked;

    if (window_len < 3) {
        window[window_len++] = rssi;
    } else {
        window[0] = window[1];
        window[1] = window[2];
        window[2] = rssi;
    }
    int sorted[3];
    memcpy(sorted, window, sizeof(int) * window_len);
    for (int i = 1; i < window_len; i++) {
        for (int j = i; j > 0 && sorted[j - 1] > sorted[j]; j--) {
            int t = sorted[j];
            sorted[j] = sorted[j - 1];
            sorted[j - 1] = t;
        }
    }
    float median = (float)sorted[window_len / 2];
    smoothed = smoothed_valid ? smoothed + SMOOTHING * (median - smoothed) : median;
    smoothed_valid = true;

    if (armed) {
        away_since = smoothed <= threshold ? (away_since ? away_since : now) : 0;
        quick_since = smoothed <= threshold - QUICK_MARGIN_DB ? (quick_since ? quick_since : now) : 0;
        if (blocked || verify_until_us != 0 || !unlockauth_has_key()) {
            return;
        }
        bool quick = quick_since && now - quick_since >= QUICK_HOLD_US;
        bool held = away_since && now - away_since >= SIGNAL_HOLD_US;
        if (quick || held) {
            char why[48];
            snprintf(why, sizeof(why), "phone at %d dBm (line %d)", (int)smoothed, threshold);
            away_since = 0;
            quick_since = 0;
            fire(why);
        }
    } else {
        near_since = smoothed >= threshold + NEAR_MARGIN_DB ? (near_since ? near_since : now) : 0;
        if (near_since && now - near_since >= NEAR_HOLD_US) {
            armed = true;
            armed_at_us = now;
            near_since = 0;
            ESP_LOGI(TAG, "phone back at the car (%d dBm); armed again", (int)smoothed);
        }
    }
}

void autolock_tick(void)
{
    int64_t now = esp_timer_get_time();

    // Arming from READY edges. If 0x038 never reaches this bus the flag
    // never moves; say so once rather than on every tick.
    bool ready = car_in_ready_fresh();
    if (ready) {
        ready_seen = true;
        parked = false;
        ready_unavailable_logged = false;
    } else if (ready_seen) {
        ready_seen = false;
        parked = true;
        reset_signal();
        if (!armed) {
            armed = true;
            armed_at_us = now;
            ESP_LOGI(TAG, "drive ended; armed for walk-away lock");
        }
    } else if (!armed && !parked && !ready_unavailable_logged) {
        ready_unavailable_logged = true;
        ESP_LOGI(TAG, "no READY signal on this bus yet; walk-away lock stays disarmed");
    }

    // The arming window closes after 12 h parked: by then a disconnect is
    // a battery event, not a walk-away.
    if (armed && now - armed_at_us > ARMED_WINDOW_US) {
        armed = false;
        parked = false;
        ESP_LOGI(TAG, "arming window expired");
    }

    // The phone's signal, while it matters: armed to lock, or locked and
    // waiting to see the phone come back.
    if (enabled && parked && connected) {
        if (now >= rssi_next_us) {
            rssi_next_us = now + RSSI_PERIOD_US;
            ble_read_rssi();
        }
        uint32_t seq = rssi_seq;
        if (seq != rssi_seq_seen) {
            rssi_seq_seen = seq;
            sample(latest_rssi, now);
        }
    }

    // Disconnect hold.
    if (hold_until_us != 0 && now >= hold_until_us) {
        hold_until_us = 0;
        if (ready) {
            // Sitting in the car with the phone's radio glitching must not
            // lock; stay armed for the real walk-away.
            ESP_LOGI(TAG, "car in READY at fire time; holding the lock");
        } else if (app_holds()) {
            ESP_LOGI(TAG, "the app reports a door open or someone seated; not locking");
        } else {
            fire("phone gone for 5 s");
        }
    }

    // Burst verification: every frame of the sequence on the bus is the
    // only confirmation available on M-CAN (the door bits live behind UDS
    // on the diagnostic bus).
    if (verify_until_us != 0) {
        bodycmd_status_t status;
        bodycmd_get_status(&status);
        if (status.frames >= frames_at_request + LOCK_BURST_FRAMES) {
            last = RESULT_LOCKED;
            count++;
            armed = false;   // until the phone comes back to the car
            verify_until_us = 0;
            persist();
            ESP_LOGI(TAG, "lock sequence on the bus (%lu total)", (unsigned long)count);
            push_report();
        } else if (now >= verify_until_us) {
            last = status.frames > frames_at_request ? RESULT_UNCONFIRMED
                                                     : RESULT_REJECTED;
            count++;
            armed = false;
            verify_until_us = 0;
            persist();
            ESP_LOGW(TAG, "lock burst %s", result_name(last));
            push_report();
        }
    }
}

bool autolock_command(const char *args, char *response, size_t response_len)
{
    if (args[0] == 0) {
        report(response, response_len);
        return true;
    }
    if (args[0] == 't') {
        char *end = NULL;
        long value = strtol(&args[1], &end, 10);
        if (end == &args[1] || *end != 0 || value < THRESHOLD_MIN || value > THRESHOLD_MAX) {
            return false;
        }
        threshold = (int8_t)value;
        persist();
        ESP_LOGI(TAG, "walk-away line %d dBm by ATXALT", threshold);
        snprintf(response, response_len, "OK");
        return true;
    }
    if (args[0] == 'b' && (args[1] == '0' || args[1] == '1') && args[2] == 0) {
        uint32_t until = 0;
        if (args[1] == '1') {
            until = (uint32_t)(esp_timer_get_time() / 1000) + BLOCK_MS;
            if (until == 0) {
                until = 1;
            }
        }
        hold_until_ms = until;
        snprintf(response, response_len, "OK");
        return true;
    }
    bool on;
    if (args[0] == '1' && args[1] == 0) {
        on = true;
    } else if (args[0] == '0' && args[1] == 0) {
        on = false;
    } else {
        return false;
    }
    enabled = on;
    if (!on) {
        armed = false;
        hold_until_us = 0;
    }
    persist();
    ESP_LOGI(TAG, "walk-away lock %s by ATXAL", on ? "enabled" : "disabled");
    snprintf(response, response_len, "OK");
    return true;
}
