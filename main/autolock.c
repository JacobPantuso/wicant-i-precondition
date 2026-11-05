// Disconnect auto-lock; see autolock.h and docs/plans/wican-disconnect-auto-lock.md.
//
// Arming: the car's READY state (0x038, tracked by precondition.c) going
// on and then off is a drive. Only then does a disconnect mean "walked
// away" rather than "walked up". If the bus never carries 0x038 this never
// arms and the feature stays inert — a safe default, not a guessed one.
//
// Trigger: the BLE client disconnects and stays gone for 5 s. iOS rebuilds
// a dead link in under 2 s, so 5 s is past transient drops while locking
// sooner after a real walk-away. A reconnect cancels the hold.
//
// Guards: the car must not be in READY at fire time. Doors and seats live
// in the IGMP's diagnostic replies (22BC03/22BC04), which reach the OBD
// adapter on the diagnostic bus, not this M-CAN link — so they cannot be
// guarded here and the app-side occupancy check remains the finer-grained
// path for harness locks it sends itself.
//
// Execution goes through bodycmd's lock burst, but as a local request: the
// whole point is the BLE link is down, and bodycmd drops client requests
// once their link is gone. Verification is the burst itself — the frame
// counter advancing by the 11-frame sequence means the bus took every
// frame. The door bits are unreadable here, so "locked" honestly means
// "lock sequence on the bus", the same ATXLK1-accepted semantics the app
// already trusts.

#include "autolock.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"

#include "bodycmd.h"
#include "precondition.h"
#include "unlockauth.h"

#define NVS_NAMESPACE   "autolock"
#define NVS_ENABLED     "en"
#define NVS_COUNT       "count"
#define NVS_LAST        "last"

#define HOLD_US         (5LL * 1000 * 1000)       // disconnect hold
#define VERIFY_US       (2LL * 1000 * 1000)       // burst frames must all land
#define ARMED_WINDOW_US (12LL * 3600 * 1000 * 1000)
#define LOCK_BURST_FRAMES 11                      // bodycmd: 4 prepare + 4 execute + 3 finish

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
static bool ready_seen;            // READY rose since the last disarm
static int64_t hold_until_us;      // 0 = no hold running
static int64_t verify_until_us;    // 0 = no burst verification running
static uint32_t frames_at_request;
static result_t last = RESULT_NONE;
static uint32_t count;
static bool ready_unavailable_logged;

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
    if (nvs_get_u8(handle, NVS_ENABLED, &u8) == ESP_OK) {
        enabled = u8 != 0;
    }
    if (nvs_get_u32(handle, NVS_COUNT, &u32) == ESP_OK) {
        count = u32;
    }
    if (nvs_get_u8(handle, NVS_LAST, &u8) == ESP_OK && u8 <= RESULT_UNCONFIRMED) {
        last = (result_t)u8;
    }
    nvs_close(handle);
    ESP_LOGI(TAG, "disconnect auto-lock %s, %lu locks so far, last %s",
             enabled ? "enabled" : "disabled", (unsigned long)count, result_name(last));
}

void autolock_note_connected(void)
{
    if (hold_until_us != 0) {
        ESP_LOGI(TAG, "client reconnected during the hold; not locking");
        hold_until_us = 0;
    }
}

void autolock_note_disconnected(void)
{
    if (!enabled || !armed || !unlockauth_has_key() || hold_until_us != 0) {
        return;
    }
    hold_until_us = esp_timer_get_time() + HOLD_US;
    ESP_LOGI(TAG, "key-holder disconnected while armed; locking in 5 s unless it returns");
}

static void fire(void)
{
    bodycmd_status_t status;
    bodycmd_get_status(&status);
    frames_at_request = status.frames;
    bodycmd_request_lock_local();
    verify_until_us = esp_timer_get_time() + VERIFY_US;
    ESP_LOGI(TAG, "firing disconnect lock");
}

void autolock_tick(void)
{
    int64_t now = esp_timer_get_time();

    // Arming from READY edges. If 0x038 never reaches this bus the flag
    // never moves; say so once rather than on every tick.
    bool ready = car_in_ready();
    if (ready) {
        ready_seen = true;
        ready_unavailable_logged = false;
    } else if (ready_seen) {
        ready_seen = false;
        if (!armed) {
            armed = true;
            armed_at_us = now;
            ESP_LOGI(TAG, "drive ended; armed for disconnect lock");
        }
    } else if (!armed && !ready_unavailable_logged) {
        ready_unavailable_logged = true;
        ESP_LOGI(TAG, "no READY signal on this bus yet; disconnect lock stays disarmed");
    }

    // The arming window closes after 12 h parked: by then a disconnect is
    // a battery event, not a walk-away.
    if (armed && now - armed_at_us > ARMED_WINDOW_US) {
        armed = false;
        ESP_LOGI(TAG, "arming window expired");
    }

    // Disconnect hold.
    if (hold_until_us != 0 && now >= hold_until_us) {
        hold_until_us = 0;
        if (ready) {
            // Sitting in the car with the phone's radio glitching must not
            // lock; stay armed for the real walk-away.
            ESP_LOGI(TAG, "car in READY at fire time; holding the lock");
        } else {
            fire();
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
            armed = false;   // one lock per drive
            verify_until_us = 0;
            persist();
            ESP_LOGI(TAG, "lock sequence on the bus (%lu total)", (unsigned long)count);
        } else if (now >= verify_until_us) {
            last = status.frames > frames_at_request ? RESULT_UNCONFIRMED
                                                     : RESULT_REJECTED;
            count++;
            armed = false;
            verify_until_us = 0;
            persist();
            ESP_LOGW(TAG, "lock burst %s", result_name(last));
        }
    }
}

bool autolock_command(const char *args, char *response, size_t response_len)
{
    if (args[0] == 0) {
        snprintf(response, response_len, "XAL:%u,%s,%lu",
                 armed ? 1 : 0,
                 result_name(last),
                 (unsigned long)count);
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
    ESP_LOGI(TAG, "disconnect auto-lock %s by ATXAL", on ? "enabled" : "disabled");
    snprintf(response, response_len, "OK");
    return true;
}
