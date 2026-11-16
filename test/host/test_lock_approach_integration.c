// Public-API integration of the real approach and autolock translation units.
#include <stdbool.h>
#include <string.h>
#include "test_support.h"
#include "autolock.h"
#include "approach.h"
#include "bodycmd.h"
#include "unlockauth.h"

static bool ready, radio_up = true;
static bool direct_managed,direct_blocked;
bool obd_handoff_managed(void){return direct_managed;}
bool obd_handoff_blocks(void){return direct_managed&&direct_blocked;}
void obd_handoff_note_on(void){}
void obd_handoff_tick(void){}
static int radio_signal = -60;
static int64_t radio_due, body_due;
static bool locking;
static unsigned lock_requests, unlock_requests;
static bodycmd_status_t body;
static QueueHandle_t queue = (void *)1;
static char response[96];

bool car_in_ready_fresh(void) { return ready; }
bool unlockauth_has_key(void) { return true; }
void unlockauth_new_challenge(char *out) { memset(out, 'a', 32); out[32] = 0; }
bool unlockauth_check(unlockauth_purpose_t p, const uint8_t *mac) { return p == UNLOCKAUTH_APPROACH; }
bool unlockauth_parse_hex(const char *s, uint8_t *out, size_t len) { return strlen(s) == len * 2; }
void bodycmd_get_status(bodycmd_status_t *out) { *out = body; }
void bodycmd_request_lock_local(void) {
    lock_requests++; body.lock_requests++; body.busy = true; locking = true;
    body_due = fake_now + 600000;
}
void bodycmd_request_unlock(void) {
    unlock_requests++; body.busy = true; locking = false;
    body_due = fake_now + 600000;
}
void bodycmd_cancel_local_lock(void) { if (locking) { body.busy = false; body_due = 0; } }
void ble_read_rssi(void) { if (radio_up && !radio_due) radio_due = fake_now + 40000; }
BaseType_t xQueueSend(QueueHandle_t q, const void *p, TickType_t t) { return 1; }

static void command(const char *args) {
    CHECK(autolock_command(args, response, sizeof response));
}
static void advance(int milliseconds) {
    for (int elapsed = 0; elapsed < milliseconds; elapsed += 40) {
        fake_now += 40000;
        if (radio_due && fake_now >= radio_due) {
            radio_due = 0;
            if (radio_up) autolock_note_rssi(radio_signal);
        }
        if (body_due && fake_now >= body_due) {
            body.frames += 11;
            if (locking) body.locks_completed++;
            body.busy = false; body_due = 0;
        }
        autolock_tick();
    }
}
static void connected(void) { radio_up = true; radio_due = 0; autolock_note_connected(); }
static void disconnected(void) { radio_up = false; radio_due = 0; autolock_note_disconnected(); }
static void start_parked_cycle(void) {
    ready = false; connected();
    command("0"); command("p1"); command("1"); command("t-72"); command("b0"); command("p0");
    command(""); CHECK(strncmp(response, "XAL:1,", 6) == 0);
}
int main(void) {
    fake_now = 1000000;
    autolock_set_tx_queue(&queue);
    CHECK(approach_command("m2", response, sizeof response));

    // Signal departure through autolock_tick(), not the private sample() shortcut.
    start_parked_cycle(); radio_signal = -60; advance(2000);
    unsigned before = lock_requests;
    radio_signal = -95; advance(8000);
    CHECK(lock_requests == before + 1);
    command(""); CHECK(strncmp(response, "XAL:0,locked,", 13) == 0);

    // Actual reconnect detector, authenticated unlock and next signal departure.
    disconnected(); advance(1000); radio_signal = -60; connected(); advance(400);
    CHECK(approach_command("", response, sizeof response));
    CHECK(strstr(response, ",-60,aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa") != NULL);
    CHECK(approach_command("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", response, sizeof response));
    CHECK(unlock_requests == 1);
    advance(1000); command(""); CHECK(strncmp(response, "XAL:1,", 6) == 0);
    before = lock_requests; radio_signal = -95; advance(8000);
    CHECK(lock_requests == before + 1);

    // Bluetooth loss remains an independent trigger after an observed on/off cycle.
    start_parked_cycle(); before = lock_requests; disconnected(); advance(4960);
    CHECK(lock_requests == before);
    advance(800); CHECK(lock_requests == before + 1);

    // A latched occupancy hold blocks BOTH paths until a real clear is forwarded.
    start_parked_cycle(); command("b1"); before = lock_requests;
    radio_signal = -95; advance(8000); disconnected(); advance(6000);
    CHECK(lock_requests == before);
    connected(); radio_signal = -95; command("b0"); advance(8000);
    CHECK(lock_requests == before + 1);

    // An on observation without a subsequent off, or off alone after reset of
    // the authorization, cannot be replaced by weak signal or a disconnect.
    command("p1"); before = lock_requests;
    connected(); radio_signal = -95; advance(8000); disconnected(); advance(6000);
    CHECK(lock_requests == before);
    // Direct ownership gates both paths even if the app sends a cached clear.
    direct_managed=false; start_parked_cycle();before=lock_requests;
    direct_managed=true;direct_blocked=true;
    command("b0");radio_signal=-95;advance(8000);disconnected();advance(6000);
    CHECK(lock_requests==before);
    // A complete selected-adapter reading can clear the direct hold. The
    // caller's gate represents VIN, freshness and lease identity validation.
    connected();direct_blocked=false;
    autolock_handoff_observation(false,false);advance(8000);
    CHECK(lock_requests==before+1);
    // Stale/cancelled/ended direct lease remains a veto after approach rearm.
    before=lock_requests;direct_blocked=true;
    autolock_approach_unlocked();command("b0");advance(8000);disconnected();advance(6000);
    CHECK(lock_requests==before);
    // Explicit relinquish erases prior ON/OFF authorization: OFF alone fails.
    direct_managed=false;autolock_handoff_invalidate();command("p0");command("b0");
    connected();advance(8000);disconnected();advance(6000);CHECK(lock_requests==before);
    // A fresh direct ON overrides an earlier clear even with weak signal.
    command("p1");command("p0");direct_managed=true;direct_blocked=false;
    autolock_handoff_observation(true,true);connected();advance(8000);disconnected();advance(6000);
    CHECK(lock_requests==before);
    return test_report("lock/approach integration");
}
