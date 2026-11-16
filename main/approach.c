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

#define NEAR_DBM (-65)
#define NEAR_DWELL_US 2000000LL
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

void approach_set_tx_queue(QueueHandle_t *queue) { tx = queue; }
void approach_departed(void) { departed = true; pending = false; near_since = 0; }
void approach_reset_session(void) {
    departed = false; pending = false; near_since = 0; power_ok = false; challenge_issued = false;
}
void approach_disconnected(void) {
    pending = false; near_since = 0; sample_at = 0; power_ok = false; challenge_issued = false;
}
static void push_event(void) {
    if (!tx || !*tx) return;
    xdev_buffer buffer = { .dev_channel = DEV_BLE };
    buffer.usLen = snprintf((char *)buffer.ucElement, sizeof buffer.ucElement,
                            "$XAU:near,%lu,%d\r", (unsigned long)sequence, signal);
    xQueueSend(*tx, &buffer, 0);
}
void approach_sample(int value, bool parked_off, int64_t now) {
    // The caller provides WiCAN's filtered signal. A gap cannot count as dwell.
    if (sample_at == 0 || now - sample_at > SAMPLE_LIFE_US) near_since = 0;
    sample_at = now; signal = value; power_ok = parked_off;
    if (!parked_off || value < NEAR_DBM) {
        pending = false; near_since = 0;
        if (!parked_off) departed = false;
        return;
    }
    if (!mode || !departed) return;
    if (!near_since) near_since = now;
    if (now - near_since < NEAR_DWELL_US) return;
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
        && signal >= NEAR_DBM;
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
    snprintf(response, length, "OK");
    return true;
}
