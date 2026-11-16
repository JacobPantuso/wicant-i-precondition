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
#define HISTORY_LEN 10
#define RECONNECT_LIFE_US 3000000LL
#define RECONNECT_CONFIRM_US 100000LL
#define EVENT_LIFE_US 5000000LL
#define SAMPLE_LIFE_US 1500000LL
static unsigned mode; // 0 off, 1 detect, 2 unlock. Volatile, reset at boot.
static bool departed;
static bool pending;
static uint32_t sequence;
static int signal;
static int64_t near_since;
static int64_t sample_at;
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

static void reset_detection(void) {
    near_since = 0; sample_at = 0; window_len = 0; history_len = 0;
    fast_candidate = false; challenge_issued = false;
    reconnect_until = 0; reconnect_close_samples = 0;
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
    unsigned supporting = 0;
    for (unsigned i = 0; i < history_len; i++) {
        int64_t age = now - history[i].at;
        if (age >= 1500000LL && age <= 4000000LL
            && value - history[i].value >= FAST_RISE_DB) supporting++;
    }
    return supporting >= 2;
}

static void remember_signal(int value, int64_t now) {
    if (history_len == HISTORY_LEN) {
        memmove(history, history + 1, (HISTORY_LEN - 1) * sizeof(history[0]));
        history_len--;
    }
    history[history_len].value = value;
    history[history_len++].at = now;
}

void approach_set_tx_queue(QueueHandle_t *queue) { tx = queue; }
void approach_departed(void) { departed = true; pending = false; reset_detection(); }
void approach_reset_session(void) {
    departed = false; pending = false; power_ok = false; reset_detection();
}
bool approach_connected(void) {
    pending = false; power_ok = false;
    reset_detection();
    if (!departed || mode == 0) return false;
    reconnect_until = esp_timer_get_time() + RECONNECT_LIFE_US;
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
    if (sample_at && now - sample_at > SAMPLE_LIFE_US) {
        pending = false; reset_detection();
    }
    sample_at = now; raw_signal = value; power_ok = parked_off;
    signal = median_sample(value);
    if (!parked_off || signal < NEAR_DBM || value < NEAR_DBM) {
        pending = false; near_since = 0; fast_candidate = false;
        reconnect_close_samples = 0;
        if (!parked_off) { departed = false; reset_detection(); }
        else remember_signal(signal, now);
        return;
    }
    if (!mode || !departed) { remember_signal(signal, now); return; }
    if (!near_since) {
        near_since = now;
        fast_candidate = rising_from_away(signal, now);
        candidate_peak = signal;
    }
    if (signal > candidate_peak) candidate_peak = signal;
    if (signal < candidate_peak - FAST_RETREAT_DB) fast_candidate = false;
    remember_signal(signal, now);
    reconnect_close_samples++;
    bool reconnect_close = reconnect_until && now <= reconnect_until
        && reconnect_close_samples >= 2 && window_len >= 2
        && now - near_since >= RECONNECT_CONFIRM_US;
    int64_t dwell = fast_candidate ? FAST_DWELL_US : NEAR_DWELL_US;
    if (!reconnect_close && (window_len < 3 || now - near_since < dwell)) return;
    ESP_LOGI("approach", "qualified %s at %d dBm after %lld ms close",
             reconnect_close ? "reconnect" : fast_candidate ? "rising" : "steady",
             signal, (long long)((now - near_since) / 1000));
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
        && now - sample_at <= SAMPLE_LIFE_US && now - event_at <= EVENT_LIFE_US
        && signal >= NEAR_DBM && raw_signal >= NEAR_DBM;
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
