// Experimental approach detection on WiCAN, authenticated per unlock on phone.
// A successful local departure lock is required before a near event can fire.
#include "approach.h"
#include <stdio.h>
#include <string.h>
#include "esp_timer.h"
#include "unlockauth.h"
#include "bodycmd.h"
#include "precondition.h"
#include "types.h"
#include "autolock.h"
#include "esp_log.h"

#define NEAR_DBM (-65)
#define NEAR_DWELL_US 2000000LL
#define FAST_DWELL_US 1000000LL
#define FAST_RISE_DB 8
#define FAST_RETREAT_DB 4
// A clear walk back may qualify before NEAR_DBM: the median must have climbed
// RISING_GAIN_DB from a floor that was really away (FAR_DBM or weaker), and
// still be climbing (rising_from_away). Phone-in-the-house jitter around
// -80 never has a floor that low, and keeps the -65 dBm steady rule.
#define RISING_DBM (-75)
#define RISING_GAIN_DB 15
#define FAR_DBM (-85)
// One raw reading may sit this far under the event's floor without voiding an
// issued challenge (pocket/body dips); a real retreat goes further.
#define RAW_DIP_DB 10
// About 8 s of history at any sample rate: one entry per HISTORY_STEP_US.
// Ten raw entries covered only 2.5 s at the 250 ms return rate, too short
// for the 4 s look-back below (2026-11-20 14:28).
#define HISTORY_LEN 16
#define HISTORY_STEP_US 450000LL
// A steady walk with the phone in a pocket climbs ~1 dB/s: this much over
// 4-8 s also counts as walking back (2026-11-20 14:28 waited for -53).
#define SLOW_RISE_DB 6
#define RECONNECT_LIFE_US 3000000LL
#define RECONNECT_CONFIRM_US 100000LL
// Long enough for a phone app woken cold in the background to answer
// (2026-11-20 10:21). The MAC is single-use and every recheck still applies.
#define EVENT_LIFE_US 8000000LL
// An issued event survives the median sitting this far under the level it
// qualified at: a pocket walk wobbles a few dB around -65 (2026-11-20 10:21).
#define EVENT_MARGIN_DB 6
// The phone saw itself come back into a geofence around the parked car from
// outside it (ATXAUA): real distance, not house jitter. For a while the walk
// back may then qualify earlier, still only from a floor at FAR_DBM or weaker.
#define HINT_RISING_DBM (-78)
#define HINT_GAIN_DB 10
#define HINT_LIFE_US (5LL * 60 * 1000000)
#define SAMPLE_LIFE_US 1500000LL
#define DEFAULT_PERIOD_US 500000LL
static unsigned mode; // 0 off, 1 detect, 2 unlock. Volatile, reset at boot.
static bool departed;
static bool pending;
static uint32_t sequence;
static int signal;
static int64_t near_since;   // median at or above the event floor in play
static int64_t close_since;  // median at or above NEAR_DBM
static int64_t sample_at;
static int64_t period_us = DEFAULT_PERIOD_US;
static int floor_signal;     // weakest median since departure or connect
static bool floor_valid;
static int event_floor = NEAR_DBM;
static int64_t connected_at;
static int64_t event_at;
static bool power_ok;
static uint32_t challenged_sequence;
static bool challenge_issued;
static uint32_t event_lock_requests;
static QueueHandle_t *tx;
static int window[3];
static unsigned window_len;
static struct { int value; int64_t at; } history[HISTORY_LEN];
static unsigned history_len;
static bool fast_candidate;
static int candidate_peak;
static int raw_signal;
static int64_t reconnect_until;
static unsigned reconnect_close_samples;
static int64_t away_hint_until; // 0 = no geofence hint
// After a "not started" relock the owner is standing at the car: no new event
// until the phone has really been away (floor at FAR_DBM) or the link dropped.
static bool need_away;

static void reset_detection(void) {
    near_since = 0; close_since = 0; sample_at = 0; window_len = 0; history_len = 0;
    fast_candidate = false; challenge_issued = false;
    reconnect_until = 0; reconnect_close_samples = 0;
}

// The away level is kept across sample gaps: a slow or interrupted sampling
// run must not forget that the phone started far away.
static void reset_floor(void) { floor_valid = false; }

// Readings further apart than this are a gap, not one continuous approach.
static int64_t sample_life_us(void) {
    int64_t life = 3 * period_us;
    return life > SAMPLE_LIFE_US ? life : SAMPLE_LIFE_US;
}

// How old the last reading may be when an unlock is answered.
static int64_t fresh_life_us(void) {
    int64_t life = period_us + 500000LL;
    return life > SAMPLE_LIFE_US ? life : SAMPLE_LIFE_US;
}

void approach_set_sample_period(int64_t us) {
    period_us = us > 0 ? us : DEFAULT_PERIOD_US;
}

static int median_sample(int value) {
    if (window_len == 3) {
        window[0] = window[1]; window[1] = window[2];
        window[2] = value;
    } else window[window_len++] = value;
    int sorted[3]; memcpy(sorted, window, window_len * sizeof(int));
    for (unsigned i = 1; i < window_len; i++) {
        for (unsigned j = i; j > 0 && sorted[j - 1] > sorted[j]; j--) {
            int tmp = sorted[j]; sorted[j] = sorted[j - 1]; sorted[j - 1] = tmp;
        }
    }
    return sorted[window_len / 2];
}

static bool rising_from_away(int value, int64_t now) {
    unsigned fast = 0, slow = 0;
    for (unsigned i = 0; i < history_len; i++) {
        int64_t age = now - history[i].at;
        if (age >= 1500000LL && age <= 4000000LL
            && value - history[i].value >= FAST_RISE_DB) fast++;
        if (age > 4000000LL && age <= 8000000LL
            && value - history[i].value >= SLOW_RISE_DB) slow++;
    }
    return fast >= 2 || slow >= 2;
}

static void remember_signal(int value, int64_t now) {
    if (history_len && now - history[history_len - 1].at < HISTORY_STEP_US) return;
    if (history_len == HISTORY_LEN) {
        memmove(history, history + 1, (HISTORY_LEN - 1) * sizeof(history[0]));
        history_len--;
    }
    history[history_len].value = value;
    history[history_len++].at = now;
}

void approach_set_tx_queue(QueueHandle_t *queue) { tx = queue; }
void approach_departed(void) {
    departed = true; pending = false; need_away = false; reset_detection(); reset_floor();
}
void approach_relocked(void) {
    departed = true; pending = false; need_away = true; reset_detection(); reset_floor();
}
void approach_reset_session(void) {
    departed = false; pending = false; power_ok = false; need_away = false;
    away_hint_until = 0; reset_detection(); reset_floor();
}
bool approach_connected(void) {
    pending = false; power_ok = false; need_away = false; // the link dropped: they left
    reset_detection(); reset_floor();
    if (!departed || mode == 0) return false;
    connected_at = esp_timer_get_time();
    reconnect_until = connected_at + RECONNECT_LIFE_US;
    ESP_LOGI("approach", "phone back after a departure; watching for the return");
    return true;
}
void approach_disconnected(void) {
    pending = false; power_ok = false; reset_detection();
}
static void push_event(void) {
    if (!tx || !*tx) return;
    xdev_buffer buffer = { .dev_channel = DEV_BLE };
    buffer.usLen = snprintf((char *)buffer.ucElement, sizeof buffer.ucElement,
                            "$XAU:near,%lu,%d\r", (unsigned long)sequence, signal);
    xQueueSend(*tx, &buffer, 0);
}
void approach_sample(int value, bool parked_off, int64_t now) {
    // Approach has its own responsive median. Departure keeps its existing
    // median+EMA; the slow lock filter must not add delay to a return.
    if (value < -127 || value >= 0) {
        pending = false; power_ok = false; reset_detection(); return;
    }
    if (sample_at && now <= sample_at) return; // duplicate/out-of-order callback
    if (sample_at && now - sample_at > sample_life_us()) {
        pending = false; reset_detection();
    }
    sample_at = now; raw_signal = value; power_ok = parked_off;
    signal = median_sample(value);
    if (!parked_off) {
        pending = false; departed = false; reset_detection(); reset_floor();
        return;
    }
    if (!floor_valid || signal < floor_signal) { floor_signal = signal; floor_valid = true; }
    if (need_away && floor_signal <= FAR_DBM) need_away = false;
    bool hinted = away_hint_until && now <= away_hint_until;
    int rising_dbm = hinted ? HINT_RISING_DBM : RISING_DBM;
    int rising_gain = hinted ? HINT_GAIN_DB : RISING_GAIN_DB;
    // An issued event lapses on a retreat: the median under its floor, or one
    // raw reading well under it. A single pocket dip does not.
    // The median gets EVENT_MARGIN_DB of slack; one raw reading a real
    // retreat's distance under the floor still cancels at once.
    if (pending && (signal < event_floor - EVENT_MARGIN_DB || value < event_floor - RAW_DIP_DB))
        pending = false;

    // The reconnect shortcut has no hold, so every reading it counts must be
    // close on its own, not only the median.
    if (value < NEAR_DBM) reconnect_close_samples = 0;
    bool close = signal >= NEAR_DBM;
    bool climbing = signal >= rising_dbm && floor_valid && floor_signal <= FAR_DBM
        && signal - floor_signal >= rising_gain;
    if (!close) close_since = 0;
    if (!close && !climbing) {
        near_since = 0; fast_candidate = false; reconnect_close_samples = 0;
        remember_signal(signal, now);
        return;
    }
    if (!mode || !departed || need_away) { remember_signal(signal, now); return; }
    if (!near_since) {
        near_since = now;
        fast_candidate = rising_from_away(signal, now);
        candidate_peak = signal;
        ESP_LOGI("approach", "candidate at %d dBm (raw %d), floor %d, %s%s",
                 signal, value, floor_signal, fast_candidate ? "rising" : "not rising",
                 hinted ? ", geofence hint" : "");
    }
    // A slow walk may only show its climb once already in the band.
    if (!fast_candidate && climbing && rising_from_away(signal, now)) {
        fast_candidate = true;
        candidate_peak = signal;
        ESP_LOGI("approach", "steady climb at %d dBm, floor %d", signal, floor_signal);
    }
    if (close && !close_since) close_since = now;
    if (signal > candidate_peak) candidate_peak = signal;
    if (signal < candidate_peak - FAST_RETREAT_DB) fast_candidate = false;
    remember_signal(signal, now);
    if (close && value >= NEAR_DBM) reconnect_close_samples++;
    bool reconnect_close = close && reconnect_until && now <= reconnect_until
        && reconnect_close_samples >= 2 && window_len >= 2
        && now - near_since >= RECONNECT_CONFIRM_US;
    bool rising = fast_candidate && window_len >= 3 && now - near_since >= FAST_DWELL_US;
    bool steady = close_since && window_len >= 3 && now - close_since >= NEAR_DWELL_US;
    if (!reconnect_close && !rising && !steady) return;
    event_floor = close ? NEAR_DBM : rising_dbm;
    away_hint_until = 0; // one event per hint
    ESP_LOGI("approach", "qualified %s at %d dBm after %lld ms; floor %d, peak %d, %lld ms since connect",
             reconnect_close ? "reconnect" : rising ? "rising" : "steady",
             signal, (long long)((now - near_since) / 1000), floor_signal, candidate_peak,
             connected_at ? (long long)((now - connected_at) / 1000) : -1LL);
    departed = false; // One event per departure, even in detection-only mode.
    pending = mode == 2;
    event_at = now;
    sequence++;
    challenge_issued = false;
    bodycmd_status_t status;
    bodycmd_get_status(&status);
    event_lock_requests = status.lock_requests;
    push_event();
}
static bool eligible(void) {
    int64_t now = esp_timer_get_time();
    bodycmd_status_t status;
    bodycmd_get_status(&status);
    return !car_in_ready_fresh() && status.lock_requests == event_lock_requests && mode == 2 && pending && power_ok && sample_at != 0
        && now - sample_at <= fresh_life_us() && now - event_at <= EVENT_LIFE_US
        && signal >= event_floor - EVENT_MARGIN_DB && raw_signal >= event_floor - RAW_DIP_DB;
}
bool approach_command(const char *args, char *response, size_t length) {
    if (!*args) {
        char nonce[2 * UNLOCKAUTH_NONCE_LEN + 1] = "-";
        if (eligible()) {
            unlockauth_new_challenge(nonce);
            challenged_sequence = sequence;
            challenge_issued = true;
        }
        snprintf(response, length, "XAU:%u,%lu,%d,%s", mode,
                 (unsigned long)sequence, signal, nonce);
        return true;
    }
    if (args[0] == 'a' && args[1] == 0) {
        away_hint_until = esp_timer_get_time() + HINT_LIFE_US;
        ESP_LOGI("approach", "geofence hint: the phone came back from outside");
        snprintf(response, length, "OK");
        return true;
    }
    if (args[0] == 'm' && args[1] >= '0' && args[1] <= '2' && args[2] == 0) {
        unsigned wanted = (unsigned)(args[1] - '0');
        if (wanted != mode) { approach_reset_session(); mode = wanted; }
        snprintf(response, length, "OK");
        return true;
    }
    uint8_t mac[UNLOCKAUTH_MAC_LEN];
    if (!eligible() || !challenge_issued || challenged_sequence != sequence || !unlockauth_parse_hex(args, mac, sizeof mac)) return false;
    bool accepted = unlockauth_check(UNLOCKAUTH_APPROACH, mac);
    pending = false; // A failed answer also consumes this approach.
    if (!accepted) return false;
    bodycmd_status_t status;
    bodycmd_get_status(&status);
    if (status.busy) return false;
    bodycmd_request_unlock();
    autolock_approach_unlocked();
    snprintf(response, length, "OK");
    return true;
}
