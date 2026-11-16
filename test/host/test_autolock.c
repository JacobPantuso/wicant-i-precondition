#include <stdbool.h>
#include <string.h>
#include "test_support.h"
#include "autolock.c"
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
    memset(&fake_body, 0, sizeof fake_body);
    enabled = true; armed = false; ready_seen = false; obd_off = false;
    parked = false; connected = true; hold_until_us = 0; verify_until_us = 0;
    hold_until_ms = 0; count = 0; reset_signal();
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
    return test_report("autolock");
}
