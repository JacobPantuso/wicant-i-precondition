// Walk-away auto-lock; see autolock.h and docs/plans/wican-disconnect-auto-lock.md.
//
// Arming, after an observed on session, from whichever comes first:
//  - the car's own power frame (0x038) explicitly reading not-READY, or
//  - fresh OBD off from the phone (ATXALP0).
// Missing CAN frames never authorize off: a 0x038 that merely went quiet
// leaves car_in_ready() where it was. READY inhibits execution.
// The bus edge exists because ATXALP0 alone raced the owner: walking off
// before the phone's OBD read landed left the car unarmed (2026-11-18 19:05).
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
// a hold (ATXALB1). While the phone is connected only its fresh clear
// releases it (ATXALB0). Once the phone's link has been gone for 30 s the
// hold lapses, as the original firmware's hold did: a phone that walked off
// before its clear arrived otherwise left the car unlocked for good
// (2026-11-16 16:28). If still armed, the disconnect lock fires then.
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
#include "approach.h"

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
#include "standby.h"
#include "types.h"
#include "unlockauth.h"

// Weak defaults preserve host tests and builds without the optional BLE client.
__attribute__((weak)) bool obd_handoff_blocks(void) { return false; }
__attribute__((weak)) bool obd_handoff_managed(void) { return false; }
__attribute__((weak)) void obd_handoff_tick(void) {}
__attribute__((weak)) void obd_handoff_note_on(void) {}

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
// After a departure, a phone that reconnects is nearly always its owner
// walking back: read faster for a minute, whatever standby would choose.
#define RETURN_PERIOD_US (250LL * 1000)
#define RETURN_WINDOW_US (60LL * 1000 * 1000)
#define SIGNAL_HOLD_US  (3LL * 1000 * 1000)
// A short line sits a couple of metres from the car, where 3 s of walking put
// the lock ~5 m past it (2026-11-20 15:07, locked at -92). Lines this close
// or closer hold for 1.5 s instead.
#define SHORT_LINE_DBM  (-75)
#define SHORT_HOLD_US   (1500LL * 1000)
#define QUICK_HOLD_US   (1LL * 1000 * 1000)
#define QUICK_MARGIN_DB 6
#define NEAR_MARGIN_DB  10
#define NEAR_HOLD_US    (2LL * 1000 * 1000)
#define BLOCK_MS        30000U
#define HOLD_LAPSE_US   (30LL * 1000 * 1000)      // hold lapses once the phone is gone this long
// A hold the app set while the car was still on ("car on", a belt during the
// drive) is stale once the bus says not-READY. It kept the WiCAN from
// locking for 43 s while the phone came and went (2026-11-20 16:54). Unless
// the app sends a fresh one after the switch-off, it lapses this long after.
#define STALE_HOLD_US   (15LL * 1000 * 1000)
#define SMOOTHING       0.6f
// After an approach unlock: no walk-away lock for a while, so the last
// metres of a walk back can't relock behind the owner (unlocked at -73,
// relocked at -84 two seconds later, 2026-11-20 10:25). Then, if the car
// isn't started and no door was reported open, relock as a fob would.
#define APPROACH_GRACE_US  (15LL * 1000 * 1000)
#define APPROACH_RELOCK_US (30LL * 1000 * 1000)
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
static bool obd_off;
static bool ready_seen;            // an on session was observed
static bool parked;                // a drive ended; the car hasn't been in READY since
static int64_t hold_until_us;      // 0 = no disconnect hold running
static int64_t verify_until_us;    // 0 = no burst verification running
static uint32_t frames_at_request;
static uint32_t locks_at_request;
static result_t last = RESULT_NONE;
static uint32_t count;

static int8_t threshold = -85;
static bool connected;
static int64_t rssi_next_us;
static int64_t reconnect_probe_until_us;
static int64_t return_fast_until_us;
static int64_t lock_sample_next_us;
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
static int64_t disconnected_at_us; // 0 while the phone is connected
static bool frame_ready_last;      // car_in_ready() at the previous tick
static int64_t hold_set_us;        // when the app last set its hold (ATXALB1)
static int64_t ready_off_us;       // when the bus last said not-READY; 0 = on/unknown
static int64_t grace_until_us;     // 0 = no post-approach grace
static int64_t relock_at_us;       // 0 = no "not started" relock pending
static bool relock_in_flight;      // the burst being verified is that relock

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
    approach_set_tx_queue(queue);
    standby_set_tx_queue(queue);
}

static void report(char *response, size_t response_len)
{
    snprintf(response, response_len, "XAL:%u,%s,%lu,%d,%d,1",
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
    return hold_until_ms != 0 || obd_handoff_blocks(); // Direct ownership requires a fresh validated read.
}

void autolock_note_connected(void)
{
    connected = true;
    disconnected_at_us = 0;
    standby_note_connected();
    reset_signal();
    {
        int64_t at = esp_timer_get_time();
        bool returning = approach_connected();
        reconnect_probe_until_us = returning ? at + 3000000LL : 0;
        return_fast_until_us = returning ? at + RETURN_WINDOW_US : 0;
    }
    rssi_next_us = 0;
    lock_sample_next_us = 0;
    rssi_seq_seen = rssi_seq; // no RSSI from the previous link can authorize return

    if (hold_until_us != 0) {
        ESP_LOGI(TAG, "client reconnected during the hold; not locking");
        hold_until_us = 0;
    }
}

void autolock_note_disconnected(void)
{
    connected = false;
    disconnected_at_us = esp_timer_get_time();
    standby_note_disconnected();
    approach_disconnected();
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
    if (app_holds()) return; // Guard both RSSI and disconnect paths at dispatch.
    bodycmd_status_t status;
    bodycmd_get_status(&status);
    if (status.busy) return;
    frames_at_request = status.frames;
    locks_at_request = status.locks_completed;
    relock_at_us = 0; // any lock makes a pending relock moot
    bodycmd_request_lock_local();
    verify_until_us = esp_timer_get_time() + VERIFY_US;
    ESP_LOGI(TAG, "firing walk-away lock: %s", why);
}

// A fresh authenticated approach unlock establishes a return, even when
// the separate walk-away filter has not crossed its stronger rearm threshold.
void autolock_approach_unlocked(void)
{
    standby_note_unlocked();   // the owner is back: leave standby either way
    if (!enabled || !parked || !obd_off || car_in_ready_fresh()) return;
    armed = true;
    armed_at_us = esp_timer_get_time();
    grace_until_us = armed_at_us + APPROACH_GRACE_US;
    relock_at_us = armed_at_us + APPROACH_RELOCK_US;
    hold_until_us = 0;
    reset_signal();
    ESP_LOGI(TAG, "authenticated return; observing the next departure");
}

// One link RSSI reading while parked: smooth it, then lock when armed and
// the phone is away, or arm again when it's back after a lock.
static int64_t signal_hold_us(void)
{
    return threshold >= SHORT_LINE_DBM ? SHORT_HOLD_US : SIGNAL_HOLD_US;
}

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
        // Well past the line, the median alone decides: the moving average
        // lags a clear walk-off by about half a second. The median of three
        // still ignores a single dip.
        quick_since = median <= threshold - QUICK_MARGIN_DB ? (quick_since ? quick_since : now) : 0;
        if (blocked || verify_until_us != 0 || !unlockauth_has_key()) {
            return;
        }
        if (grace_until_us && now < grace_until_us) {
            away_since = 0; // the walk back isn't over yet
            quick_since = 0;
            return;
        }
        bool quick = quick_since && now - quick_since >= QUICK_HOLD_US;
        bool held = away_since && now - away_since >= signal_hold_us();
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

// The drive is over: arm for the walk-away. Shared by the bus edge and the
// phone's ATXALP0, so whichever lands first arms and the other is a no-op.
static void arm_after_drive(const char *source)
{
    obd_off = true;   // "off is authorized", by the bus or by the phone
    parked = true;
    standby_note_off();
    ready_seen = false;
    armed = enabled;
    armed_at_us = esp_timer_get_time();
    grace_until_us = 0;   // a new drive's departure: no leftover approach state
    relock_at_us = 0;
    reset_signal();
    if (!connected && armed) hold_until_us = armed_at_us + HOLD_US;
    ESP_LOGI(TAG, "drive ended (%s); %s", source,
             armed ? "armed for walk-away lock" : "walk-away lock off");
}

void autolock_tick(void)
{
    int64_t now = esp_timer_get_time();
    obd_handoff_tick();

    // Arming from READY edges. If 0x038 never reaches this bus the flag
    // never moves; say so once rather than on every tick.
    bool ready = car_in_ready_fresh();
    // Only a change counts: READY merely being false (ACC, or the phone's
    // OBD reading the car on while 0x038 never said READY) never arms.
    bool frame_ready = car_in_ready();
    bool frame_fell = frame_ready_last && !frame_ready;
    frame_ready_last = frame_ready;
    if (frame_fell) ready_off_us = now;   // whichever path arms, holds before this are stale
    if (ready) {
        standby_note_drive();
        approach_reset_session();
        ready_seen = true;
        obd_off = false;
        parked = false;
        armed = false;
        grace_until_us = 0;
        relock_at_us = 0; // started: no "not started" relock
        ready_off_us = 0;
        bodycmd_cancel_local_lock();
        hold_until_us = 0;
        reset_signal();
    } else if (frame_fell && ready_seen && !obd_off) {
        // CAN silence is unknown, never an off edge: only a 0x038 frame that
        // explicitly reads not-READY clears car_in_ready(). ATXALP0 arms too.
        arm_after_drive("READY off on the bus");
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
    if (enabled && parked && obd_off && !ready && connected) {
        if (now >= rssi_next_us) {
            // Parked at Home with the phone idle, standby slows this to 2 s;
            // a phone just back from a departure is read faster instead.
            int64_t period = now < reconnect_probe_until_us ? 100000LL
                : now < return_fast_until_us ? RETURN_PERIOD_US
                : standby_rssi_period_us(RSSI_PERIOD_US);
            approach_set_sample_period(period);
            rssi_next_us = now + period;
            ble_read_rssi();
        }
        uint32_t seq = rssi_seq;
        if (seq != rssi_seq_seen) {
            rssi_seq_seen = seq;
            approach_sample(latest_rssi, parked && obd_off && !ready, now);
            // Burst sampling is for approach only. Departure keeps its normal
            // cadence, filter and elapsed-time safety holds.
            if (now >= lock_sample_next_us) {
                lock_sample_next_us = now + RSSI_PERIOD_US;
                sample(latest_rssi, now);
            }
        }
    }

    // Disconnect hold.
    if (hold_until_us != 0 && now >= hold_until_us) {
        hold_until_us = 0;
        if (!enabled || !armed || !obd_off || ready) {
            // Sitting in the car with the phone's radio glitching must not
            // lock; stay armed for the real walk-away.
            ESP_LOGI(TAG, "car in READY at fire time; holding the lock");
        } else if (app_holds()) {
            ESP_LOGI(TAG, "the app reports a door open or someone seated; not locking");
        } else {
            fire("phone gone for 5 s");
        }
    }

    // A hold from before the switch-off lapses 15 s after it. A fresh hold
    // sent since (door open, still buckled) is kept.
    if (hold_until_ms != 0 && ready_off_us != 0 && hold_set_us < ready_off_us
            && now - ready_off_us >= STALE_HOLD_US) {
        hold_until_ms = 0;
        ESP_LOGI(TAG, "hold from before the switch-off lapsed");
        if (enabled && armed && obd_off && !ready && !connected && hold_until_us == 0
                && verify_until_us == 0 && unlockauth_has_key()) {
            fire("phone gone, stale hold lapsed");
        }
    }

    // Door/seat hold lapse: the phone has been gone 30 s and can't send its
    // clear any more. READY still blocks in fire()'s callers and here.
    if (hold_until_ms != 0 && !connected && disconnected_at_us != 0
            && now - disconnected_at_us >= HOLD_LAPSE_US) {
        hold_until_ms = 0;
        ESP_LOGI(TAG, "phone gone for 30 s; door/seat hold lapsed");
        if (enabled && armed && obd_off && !ready && hold_until_us == 0
                && verify_until_us == 0 && unlockauth_has_key()) {
            fire("phone gone, hold lapsed");
        }
    }

    // Not started within 30 s of an approach unlock: relock. A door the app
    // reported open (ATXALB1) cancels it instead; the walk-away lock covers
    // the next departure then.
    if (relock_at_us && now >= relock_at_us) {
        relock_at_us = 0;
        // The phone right at the car means they're in it or beside it, a door
        // the app hasn't read yet (2026-11-20 14:28: relocked at -49 with the
        // owner seated). The walk-away lock covers their leaving instead.
        bool at_car = connected && smoothed_valid && smoothed >= threshold + NEAR_MARGIN_DB;
        if (at_car) {
            ESP_LOGI(TAG, "approach relock skipped: phone at the car (%d dBm)", (int)smoothed);
        } else if (enabled && parked && obd_off && !ready && verify_until_us == 0
                && !app_holds() && unlockauth_has_key()) {
            relock_in_flight = true;
            fire("not started within 30 s of the approach unlock");
            if (verify_until_us == 0) relock_in_flight = false; // fire() declined
        } else {
            ESP_LOGI(TAG, "approach relock skipped");
        }
    }

    // Burst verification: every frame of the sequence on the bus is the
    // only confirmation available on M-CAN (the door bits live behind UDS
    // on the diagnostic bus).
    if (verify_until_us != 0) {
        bodycmd_status_t status;
        bodycmd_get_status(&status);
        if (status.locks_completed > locks_at_request) {
            last = RESULT_LOCKED;
            if (relock_in_flight) approach_relocked(); else approach_departed();
            relock_in_flight = false;
            standby_note_locked("walk-away");
            count++;
            armed = false;   // until the phone comes back to the car
            verify_until_us = 0;
            persist();
            ESP_LOGI(TAG, "lock sequence on the bus (%lu total)", (unsigned long)count);
            push_report();
        } else if (now >= verify_until_us) {
            last = status.frames > frames_at_request ? RESULT_UNCONFIRMED
                                                     : RESULT_REJECTED;
            relock_in_flight = false;
            count++;
            armed = false;
            verify_until_us = 0;
            persist();
            ESP_LOGW(TAG, "lock burst %s", result_name(last));
            push_report();
        }
    }

    standby_tick();
}

static bool autolock_apply_command(const char *,char *,size_t);

void autolock_handoff_invalidate(void)
{
    armed=false;obd_off=false;parked=false;ready_seen=false;
    grace_until_us=0;relock_at_us=0;relock_in_flight=false;
    hold_until_ms=1;hold_until_us=0;verify_until_us=0;
    bodycmd_cancel_local_lock();reset_signal();approach_reset_session();
}

bool autolock_can_handoff(void)
{
    return enabled && (ready_seen || (parked && obd_off)) && standby_lease_allowed();
}

// The owner locked the car (fob, handle or app) while WiCAN held the OBD
// lease: the lease saw a door open after the drive, then every door locked
// with the car off and nobody belted in. Same opportunity as WiCAN's own
// walk-away lock: one approach unlock, then standby. No lock burst follows.
void autolock_owner_locked(void)
{
    if (!parked || !obd_off || car_in_ready_fresh() || verify_until_us != 0) return;
    armed = false;
    hold_until_us = 0;
    grace_until_us = 0;
    relock_at_us = 0;
    bodycmd_cancel_local_lock();
    reset_signal();
    approach_departed();
    standby_note_locked("owner");
    ESP_LOGI(TAG, "owner locked the car; walk-away lock stands down");
}

void autolock_handoff_observation(bool on, bool blocked)
{
    // Called only for the selected adapter's complete, VIN-validated response.
    char response[48];
    autolock_apply_command(on ? "p1" : "p0", response, sizeof(response));
    autolock_apply_command(blocked ? "b1" : "b0", response, sizeof(response));
}

bool autolock_command(const char *args, char *response, size_t response_len)
{
    // Once direct ownership begins, app OFF/clear cannot override a missing,
    // stale or blocked direct observation. Fresh app ON remains a veto.
    if (!strcmp(args, "p1")) obd_handoff_note_on();
    if (obd_handoff_managed() && (!strcmp(args,"p0") || !strcmp(args,"b0"))) {
        snprintf(response,response_len,"OK");
        return true;
    }
    return autolock_apply_command(args,response,response_len);
}

static bool autolock_apply_command(const char *args, char *response, size_t response_len)
{
    if (args[0] == 0) {
        report(response, response_len);
        return true;
    }
    if (args[0] == 'p' && (args[1] == '0' || args[1] == '1') && args[2] == 0) {
        if (args[1] == '1') {
            standby_note_drive();
            approach_reset_session();
            ready_seen = true;
            obd_off = false;
            parked = false;
            armed = false;
            grace_until_us = 0;
            relock_at_us = 0;
            bodycmd_cancel_local_lock();
            hold_until_us = 0;
            reset_signal();
        } else if (!car_in_ready_fresh() && ready_seen && !obd_off) {
            arm_after_drive("phone: OBD off");
        }
        snprintf(response, response_len, "OK");
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
            until = 1;
            if (until == 0) {
                until = 1;
            }
        }
        hold_until_ms = until;
        if (until != 0) {
            hold_set_us = esp_timer_get_time();
            bodycmd_cancel_local_lock();
            if (relock_at_us) {
                relock_at_us = 0; // a door opened: they're using the car
                ESP_LOGI(TAG, "door open after the approach unlock; no relock");
            }
        }
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
        bodycmd_cancel_local_lock();
        hold_until_us = 0;
    }
    persist();
    ESP_LOGI(TAG, "walk-away lock %s by ATXAL", on ? "enabled" : "disabled");
    snprintf(response, response_len, "OK");
    return true;
}
