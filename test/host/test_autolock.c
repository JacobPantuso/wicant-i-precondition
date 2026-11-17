#include <stdbool.h>
#include <string.h>
#include "test_support.h"
#include "autolock.c"
void approach_set_tx_queue(QueueHandle_t *q) {}
void approach_departed(void) {}
void approach_reset_session(void) {}
void approach_disconnected(void) {}
static bool reconnect_permitted;
bool approach_connected(void) { return reconnect_permitted; }
void approach_sample(int s, bool p, int64_t n) {}
void approach_set_sample_period(int64_t us) {}
// Standby has its own suite (test_standby.c); here it always allows a lease.
void standby_set_tx_queue(QueueHandle_t *q) {}
void standby_note_drive(void) {}
void standby_note_off(void) {}
void standby_note_locked(const char *why) {}
void standby_note_unlocked(void) {}
bool standby_lease_allowed(void) { return true; }
void standby_note_connected(void) {}
void standby_note_disconnected(void) {}
void standby_tick(void) {}
int64_t standby_rssi_period_us(int64_t normal_us) { return normal_us; }
static bool fake_ready;
static int requests;
static bodycmd_status_t fake_body;
bool car_in_ready_fresh(void) { return fake_ready; }
bool unlockauth_has_key(void) { return true; }
void ble_read_rssi(void) {}
void bodycmd_get_status(bodycmd_status_t *s) { *s = fake_body; }
void bodycmd_request_lock_local(void) { requests++; }
void bodycmd_cancel_local_lock(void) {}
BaseType_t xQueueSend(QueueHandle_t q, const void *p, TickType_t t) { return 1; }
static void command(const char *s) { char out[64]; CHECK(autolock_command(s, out, sizeof out)); }
static void reset(void) {
    fake_now = 1000000; fake_ready = false; requests = 0;
    reconnect_permitted = false; reconnect_probe_until_us = lock_sample_next_us = 0;
    memset(&fake_body, 0, sizeof fake_body);
    enabled = true; armed = false; ready_seen = false; obd_off = false;
    parked = false; connected = true; hold_until_us = 0; verify_until_us = 0;
    hold_until_ms = 0; count = 0; disconnected_at_us = 0; reset_signal();
}
static void off_after_drive(void) { command("p1"); command("p0"); CHECK(armed); }
int main(void) {
    reset(); command("p0"); CHECK(!armed);
    fake_ready = true; autolock_tick(); CHECK(!armed);
    fake_ready = false; fake_now += 4000000; autolock_tick();
    CHECK(!armed); // CAN silence alone never arms
    command("p0"); CHECK(armed);
    autolock_note_disconnected(); fake_now += 6000000; autolock_tick(); CHECK(requests == 1);
    fake_body.frames = 100; autolock_tick(); CHECK(count == 0); // unrelated frames
    fake_body.locks_completed = 1; autolock_tick(); CHECK(count == 1 && last == RESULT_LOCKED);
    reset(); off_after_drive(); autolock_note_disconnected();
    command("p1"); fake_now += 6000000; autolock_tick(); CHECK(requests == 0 && !armed);
    reset(); off_after_drive(); autolock_note_disconnected();
    command("0"); fake_now += 6000000; autolock_tick(); CHECK(requests == 0);
    reset(); off_after_drive(); autolock_note_disconnected();
    autolock_note_connected(); fake_now += 6000000; autolock_tick(); CHECK(requests == 0);
    reset(); off_after_drive(); command("b1"); autolock_note_disconnected();
    fake_now += 6000000; autolock_tick(); CHECK(requests == 0);
    reset(); command("p1"); connected = false; command("p0");
    fake_now += 6000000; autolock_tick(); CHECK(requests == 1);
    reset(); off_after_drive(); fake_ready = true; autolock_tick(); CHECK(!armed && !parked);
    // Accepted approach unlock rearms without crossing the old -62 line.
    reset(); off_after_drive(); armed = false;
    autolock_approach_unlocked(); CHECK(armed && !smoothed_valid);
    for (int i = 0; i < 5; i++) { fake_now += 500000; sample(-63, fake_now); }
    CHECK(armed && requests == 0);
    for (int i = 0; i < 12; i++) { fake_now += 500000; sample(-95, fake_now); }
    CHECK(requests == 1);
    reset(); autolock_approach_unlocked(); CHECK(!armed); // no parked/off authorization
    reset(); off_after_drive(); armed = false; fake_ready = true;
    autolock_approach_unlocked(); CHECK(!armed);
    reset(); off_after_drive(); armed = false; enabled = false;
    autolock_approach_unlocked(); CHECK(!armed);
    reset(); off_after_drive(); reconnect_permitted = true;
    autolock_note_rssi(-40); // stale sample from the old link
    autolock_note_connected(); CHECK(rssi_seq_seen == rssi_seq);
    autolock_tick(); CHECK(rssi_next_us - fake_now == 100000LL);
    autolock_note_rssi(-63); autolock_tick();
    CHECK(lock_sample_next_us - fake_now == RSSI_PERIOD_US);
    fake_now += 120000; autolock_note_rssi(-62); autolock_tick();
    CHECK(lock_sample_next_us - fake_now == RSSI_PERIOD_US - 120000);
    // After the probe, a returning phone is read every 250 ms for a minute.
    fake_now += 3000000; autolock_tick(); CHECK(rssi_next_us - fake_now == RETURN_PERIOD_US);
    fake_now += RETURN_WINDOW_US; autolock_tick(); CHECK(rssi_next_us - fake_now == RSSI_PERIOD_US);
    // A reconnect with no departure behind it keeps the normal cadence.
    reset(); off_after_drive(); reconnect_permitted = false;
    autolock_note_connected(); autolock_tick(); CHECK(rssi_next_us - fake_now == RSSI_PERIOD_US);
    return test_report("autolock");
}
