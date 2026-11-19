// Public-API integration of the real approach and autolock translation units.
#include <stdbool.h>
#include <string.h>
#include "test_support.h"
#include "autolock.h"
#include "approach.h"
#include "bodycmd.h"
#include "unlockauth.h"
#include "standby.h"

static standby_state_t applied_state;
static int apply_calls;
static float vbat = 12.6f;
void standby_platform_apply(standby_state_t s, bool pm) { applied_state = s; apply_calls++; }
int standby_platform_reset_reason(void) { return 1; }

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
bool car_in_ready(void) { return ready; }
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
        standby_note_voltage(vbat, vbat > 0);   // the ADC task, about every second
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

    // Field failure 2026-11-16 21:00: the lease failed, the app took its
    // adapter back and sent fresh off/clear readings. While WiCAN still owned
    // safety those were swallowed and walk-away never fired. Once ownership
    // returns, the hold latched at the failure still blocks until a reading
    // sent after it, which then lets the signal walk-away lock.
    direct_managed = false; direct_blocked = false; vbat = 12.6f;
    start_parked_cycle(); radio_signal = -60; advance(1000);
    direct_managed = true; direct_blocked = true;            // lease running
    command("b1");                                            // hold latched at finish
    direct_managed = false;                                   // lease failed: released
    before = lock_requests; radio_signal = -95; advance(8000);
    CHECK(lock_requests == before);                           // latched hold still blocks
    command("b0"); advance(8000);                             // the app's fresh clear
    CHECK(lock_requests == before + 1);

    // Hold lapse. A hold sent while connected survives the 5 s disconnect
    // check, then lapses 30 s after the link drops and the lock fires
    // (2026-11-16 16:28: the phone walked off before its clear arrived).
    direct_managed = false;
    start_parked_cycle(); connected(); radio_signal = -60; advance(1000);
    command("b1"); before = lock_requests;
    disconnected(); advance(6000);
    CHECK(lock_requests == before);                  // 5 s check: held
    advance(23000); CHECK(lock_requests == before);  // 29 s gone: still held
    advance(1500);  CHECK(lock_requests == before + 1);
    // Connected, the hold never lapses on time alone: the phone is there to
    // send its own clear.
    start_parked_cycle(); connected(); radio_signal = -60; advance(1000);
    command("b1"); before = lock_requests; advance(60000);
    CHECK(lock_requests == before);
    // A reconnect inside the 30 s restarts the clock; the hold stays.
    disconnected(); advance(20000); connected(); advance(1000);
    disconnected(); advance(20000); CHECK(lock_requests == before);
    advance(11000); CHECK(lock_requests == before + 1);
    // Car back on: the lapse never fires a lock.
    start_parked_cycle(); connected(); advance(1000); command("b1");
    before = lock_requests; disconnected(); advance(3000); ready = true;
    advance(35000); CHECK(lock_requests == before); ready = false;

    // ---- Parked BLE standby -------------------------------------------------
    char xpk[112];
    #define PK(args) CHECK(standby_command(args, xpk, sizeof xpk))
    direct_managed = false; direct_blocked = false; vbat = 12.6f;

    // Off by default: a walk-away lock leaves WiCAN active as before.
    start_parked_cycle(); radio_signal = -95; advance(8000);
    command(""); CHECK(strncmp(response, "XAL:0,locked,", 13) == 0);
    CHECK(standby_state() == STANDBY_ACTIVE);

    // On: the drive's on edge grants one lease, the off edge doesn't add one.
    PK("e1"); command("p1"); CHECK(autolock_can_handoff());
    standby_note_lease_started(); CHECK(!autolock_can_handoff());
    command("p1"); advance(80); CHECK(!autolock_can_handoff());   // same on session
    command("p0"); start_parked_cycle(); CHECK(autolock_can_handoff()); // a new drive
    standby_note_lease_started(); CHECK(!autolock_can_handoff());
    // A lease that failed before touching the car gives it back; one that
    // probed the car does not.
    standby_note_lease_ended(false); CHECK(autolock_can_handoff());
    standby_note_lease_started(); standby_note_lease_ended(true); CHECK(!autolock_can_handoff());

    // WiCAN's walk-away lock enters standby; the side effects run on the tick.
    connected(); radio_signal = -60; advance(2000);
    radio_signal = -95; advance(8000);
    CHECK(standby_state() == STANDBY_PARKED && applied_state == STANDBY_PARKED);
    PK(""); CHECK(strncmp(xpk, "XPK:1,parked,walk-away,", 23) == 0);
    CHECK(strstr(xpk, ",12600,11600,0,0,1,1") != NULL);

    // The phone reconnecting from the house: no lease, no new budget.
    int64_t entered = fake_now;
    disconnected(); advance(1000); radio_signal = -80; connected(); advance(2000);
    CHECK(standby_state() == STANDBY_PARKED && !autolock_can_handoff());
    // Home idle: 2 s sampling while parked, never a slow link; a disconnect ends it.
    PK("h1"); advance(80);
    CHECK(standby_rssi_period_us(500000) == 2000000);
    disconnected(); advance(80);
    CHECK(standby_rssi_period_us(500000) == 500000);
    connected(); advance(80);
    CHECK(standby_rssi_period_us(500000) == 500000);   // the hint doesn't survive a reconnect

    // The 48 h budget runs from the lock, not from the last reconnect.
    fake_now = entered + 47LL * 3600 * 1000000; advance(80);
    CHECK(standby_state() == STANDBY_PARKED);
    disconnected(); connected(); advance(80);
    fake_now = entered + 48LL * 3600 * 1000000; advance(80);
    CHECK(standby_state() == STANDBY_PROTECT && standby_protect_requested());
    CHECK(applied_state == STANDBY_PROTECT && !autolock_can_handoff());
    PK(""); CHECK(strncmp(xpk, "XPK:1,protect,budget,", 21) == 0);
    // The next drive leaves protect and grants a lease.
    command("p1"); CHECK(standby_state() == STANDBY_ACTIVE && autolock_can_handoff());
    advance(80); CHECK(applied_state == STANDBY_ACTIVE);

    // Battery floor: 10 min under 11.6 V ends standby. 11.8 V, the resting
    // voltage that tripped the old 12.2 V floor in the field, never does, and
    // a brief dip under the floor resets the clock.
    start_parked_cycle(); connected(); radio_signal = -95; advance(8000);
    CHECK(standby_state() == STANDBY_PARKED);
    vbat = 11.8f; fake_now += 30LL * 60 * 1000000; advance(1000);
    CHECK(standby_state() == STANDBY_PARKED);
    vbat = 11.5f; advance(300000); vbat = 11.9f; advance(1000);
    vbat = 11.5f; advance(540000); CHECK(standby_state() == STANDBY_PARKED);
    advance(61000); CHECK(standby_state() == STANDBY_PROTECT);
    PK(""); CHECK(strncmp(xpk, "XPK:1,protect,battery,", 22) == 0);
    // No reading at all counts as low.
    vbat = 12.6f; command("p1"); start_parked_cycle(); connected(); radio_signal = -95; advance(8000);
    CHECK(standby_state() == STANDBY_PARKED);
    vbat = -1; advance(540000); CHECK(standby_state() == STANDBY_PARKED);
    advance(61000); CHECK(standby_state() == STANDBY_PROTECT);
    PK(""); CHECK(strncmp(xpk, "XPK:1,protect,no-voltage,", 25) == 0);
    vbat = 12.6f;

    // An accepted approach unlock leaves standby and grants the next lease.
    command("p1"); start_parked_cycle(); connected(); radio_signal = -60; advance(2000);
    radio_signal = -95; advance(8000); CHECK(standby_state() == STANDBY_PARKED);
    standby_note_lease_started();   // the departure's lease, used
    unsigned unlocks = unlock_requests;
    disconnected(); advance(1000); radio_signal = -60; connected(); advance(400);
    CHECK(approach_command("", response, sizeof response));
    CHECK(approach_command("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", response, sizeof response));
    CHECK(unlock_requests == unlocks + 1);
    CHECK(standby_state() == STANDBY_ACTIVE && autolock_can_handoff());

    // The owner locking during the lease: the same single approach unlock,
    // standby, and no lock burst from WiCAN when they walk away.
    command("p1"); start_parked_cycle(); connected(); radio_signal = -60; advance(1000);
    autolock_owner_locked();
    CHECK(standby_state() == STANDBY_PARKED);
    PK(""); CHECK(strncmp(xpk, "XPK:1,parked,owner,", 19) == 0);
    before = lock_requests; radio_signal = -95; advance(8000); disconnected(); advance(6000);
    CHECK(lock_requests == before);
    unlocks = unlock_requests;
    radio_signal = -60; connected(); advance(400);
    CHECK(approach_command("", response, sizeof response));
    CHECK(strstr(response, ",-60,aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa") != NULL);
    CHECK(approach_command("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", response, sizeof response));
    CHECK(unlock_requests == unlocks + 1);
    // An owner lock while the car is on (no fresh off) is ignored.
    command("p1"); autolock_owner_locked(); CHECK(standby_state() == STANDBY_ACTIVE);

    // Turning standby off while parked returns to normal behaviour.
    command("p1"); start_parked_cycle(); connected(); radio_signal = -95; advance(8000);
    CHECK(standby_state() == STANDBY_PARKED);
    PK("e0"); advance(80); CHECK(standby_state() == STANDBY_ACTIVE && applied_state == STANDBY_ACTIVE);

    // The owner resuming the phone's OBD adapter leaves standby, no lease.
    PK("e1"); command("p1"); start_parked_cycle(); connected(); radio_signal = -95; advance(8000);
    standby_note_lease_started();
    CHECK(standby_state() == STANDBY_PARKED);
    PK("x"); advance(80);
    CHECK(standby_state() == STANDBY_ACTIVE && applied_state == STANDBY_ACTIVE && !autolock_can_handoff());
    PK(""); CHECK(strncmp(xpk, "XPK:1,active,resumed,", 21) == 0);
    PK("x");   // nothing to leave: still OK

    // Command validation.
    CHECK(!standby_command("v10500", xpk, sizeof xpk));
    CHECK(!standby_command("e2", xpk, sizeof xpk));
    PK("v12000"); PK(""); CHECK(strstr(xpk, ",12000,") != NULL);
    return test_report("lock/approach integration");
}
