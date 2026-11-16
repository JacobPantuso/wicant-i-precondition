#include <stdbool.h>
#include <string.h>
#include "test_support.h"
#include "approach.c"

static bool authenticated = true;
static bool ready;
bool car_in_ready_fresh(void) { return ready; }
static unsigned unlock_count;
static unsigned notifications;
static bodycmd_status_t body;
void unlockauth_new_challenge(char *out) { memset(out, 'a', 32); out[32] = 0; }
bool unlockauth_check(unlockauth_purpose_t p, const uint8_t *mac) {
    CHECK(p == UNLOCKAUTH_APPROACH);
    return authenticated;
}
bool unlockauth_parse_hex(const char *text, uint8_t *out, size_t len) {
    return strlen(text) == len * 2;
}
void bodycmd_get_status(bodycmd_status_t *out) { *out = body; }
void bodycmd_request_unlock(void) { unlock_count++; }
BaseType_t xQueueSend(QueueHandle_t q, const void *p, TickType_t t) {
    notifications++; return 1;
}
static QueueHandle_t queue = (void *)1;
static char response[80];
static const char *mac = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
static void reset(void) {
    approach_reset_session(); approach_disconnected();
    mode = 0; sequence = 0; unlock_count = notifications = 0;
    authenticated = true; fake_now = 1000000;
    memset(&body, 0, sizeof body);
    approach_set_tx_queue(&queue);
}
static void sample(int rssi, bool off, int ticks) {
    for (int i = 0; i < ticks; i++) {
        fake_now += 500000;
        approach_sample(rssi, off, fake_now);
    }
}
static void arm(unsigned value) {
    char command[] = "m0"; command[1] += value;
    CHECK(approach_command(command, response, sizeof response));
    approach_departed();
}
static bool answer(void) { return approach_command(mac, response, sizeof response); }
static void challenge(void) { CHECK(approach_command("", response, sizeof response)); }
int main(void) {
    reset(); arm(0); sample(-55, true, 10); CHECK(!pending && !unlock_count && !notifications);
    reset(); arm(1); sample(-55, true, 10); CHECK(!pending && notifications == 1);
    challenge(); CHECK(!answer() && !unlock_count);
    reset(); mode = 2; sample(-55, true, 10); CHECK(!pending && !notifications); // no departure
    reset(); arm(2); sample(-55, true, 2); sample(-80, true, 4); CHECK(!pending);
    sample(-55, true, 5); CHECK(pending && notifications == 1);
    CHECK(!answer()); // cannot reuse a challenge from a different event
    challenge(); CHECK(answer() && unlock_count == 1); CHECK(!answer());
    sample(-55, true, 20); CHECK(notifications == 1); // no repeated unlock in range
    reset(); arm(2); sample(-55, true, 5); challenge();
    fake_now += 2000000; CHECK(!answer() && !unlock_count); // stale RSSI
    reset(); arm(2); sample(-55, true, 5); challenge();
    approach_disconnected(); CHECK(!answer());
    reset(); arm(2); sample(-55, true, 5); challenge();
    sample(-55, false, 1); CHECK(!answer()); // on or power invalid
    reset(); arm(2); sample(-55, true, 5); challenge();
    sample(-85, true, 1); CHECK(!answer()); // already left again
    reset(); arm(2); sample(-55, true, 5); challenge();
    body.lock_requests++; CHECK(!answer()); // manually relocked nearby
    reset(); arm(2); sample(-55, true, 5); challenge();
    authenticated = false; CHECK(!answer()); authenticated = true; CHECK(!answer());
    reset(); arm(2); sample(-55, true, 5); challenge();
    sample(-55, true, 11); CHECK(!answer()); // expired event despite fresh RSSI
    reset(); arm(2); sample(-55, true, 5); challenge();
    body.busy = true; CHECK(!answer() && !unlock_count);
    reset(); arm(2); sample(-55, true, 5); challenge(); ready = true; CHECK(!answer()); ready = false;
    reset(); arm(2); sample(-55, true, 1); fake_now += 10000000;
    sample(-55, true, 1); CHECK(!pending); // gaps do not count toward near dwell
    return test_report("approach");
}
