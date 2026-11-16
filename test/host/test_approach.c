#include <stdbool.h>
#include <string.h>
#include "test_support.h"
#include "approach.c"

static bool authenticated = true;
static bool ready;
bool car_in_ready_fresh(void) { return ready; }
static unsigned unlock_count;
static unsigned rearms;
void autolock_approach_unlocked(void) { rearms++; }
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
    authenticated = true; ready = false; rearms = 0; fake_now = 1000000;
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
    challenge(); CHECK(answer() && unlock_count == 1 && rearms == 1); CHECK(!answer());
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
    // A rising approach can use the short close hold, with no EMA lag.
    reset(); arm(2); sample(-88, true, 4);
    sample(-74, true, 1); sample(-66, true, 1); sample(-63, true, 1);
    CHECK(!pending);
    sample(-61, true, 1); CHECK(fast_candidate && !pending);
    int64_t close_at = fake_now;
    sample(-60, true, 1); CHECK(!pending);
    sample(-60, true, 1); CHECK(pending && fake_now - close_at == FAST_DWELL_US);
    challenge(); CHECK(answer() && rearms == 1 && unlock_count == 1);

    // A close reconnect without trajectory retains the full two-second hold.
    reset(); arm(2); sample(-60, true, 4); CHECK(!pending && !fast_candidate);
    sample(-60, true, 1); CHECK(pending);

    // One/two strong spikes surrounded by weak readings cannot qualify.
    reset(); arm(2); sample(-85, true, 4);
    sample(-40, true, 1); sample(-85, true, 4); CHECK(!pending && !notifications);
    sample(-40, true, 2); sample(-85, true, 4); CHECK(!pending && !notifications);

    // Close jitter without a meaningful rise never selects the fast route.
    reset(); arm(2); sample(-69, true, 4); sample(-64, true, 3);
    CHECK(!fast_candidate && !pending);
    sample(-64, true, 2); CHECK(!pending);
    sample(-64, true, 1); CHECK(pending);

    // Retreat cancels an issued challenge immediately, despite the median lag.
    reset(); arm(2); sample(-90, true, 4); sample(-60, true, 4);
    CHECK(pending); challenge(); sample(-80, true, 1); CHECK(!answer() && !rearms);

    // Invalid/127 RSSI, gaps and duplicate timestamps cannot create dwell.
    reset(); arm(2); sample(-90, true, 4); sample(-60, true, 2);
    sample(127, true, 1); CHECK(!pending && !sample_at);
    sample(-60, true, 4); CHECK(!pending);
    sample(-60, true, 1); CHECK(pending);
    reset(); arm(2); sample(-90, true, 4); sample(-60, true, 2);
    fake_now += 2000000; sample(-60, true, 1);
    CHECK(!pending && !fast_candidate);
    for (int i = 0; i < 10; i++) approach_sample(-60, true, fake_now);
    CHECK(!pending);

    // Power invalidation consumes departure; fresh close readings cannot restore it.
    reset(); arm(2); sample(-90, true, 4); sample(-60, true, 1);
    sample(-60, false, 1); sample(-60, true, 10); CHECK(!pending && !notifications);

    // Reconnect after a real departure: two fresh close samples, no 1–2 s hold.
    reset(); arm(2); approach_disconnected(); CHECK(approach_connected());
    fake_now += 120000; approach_sample(-61, true, fake_now); CHECK(!pending);
    fake_now += 120000; approach_sample(-60, true, fake_now); CHECK(pending);
    challenge(); CHECK(answer() && unlock_count == 1); CHECK(!answer());
    sample(-60, true, 10); CHECK(notifications == 1);

    reset(); mode = 2; CHECK(!approach_connected()); // no departure
    sample(-60, true, 2); CHECK(!pending);
    reset(); arm(2); approach_disconnected(); CHECK(approach_connected());
    fake_now += 120000; approach_sample(-60, true, fake_now);
    fake_now += 120000; approach_sample(-80, true, fake_now); CHECK(!pending);
    fake_now += 120000; approach_sample(-60, true, fake_now); CHECK(!pending);
    fake_now += 120000; approach_sample(-60, true, fake_now); CHECK(pending);
    challenge(); fake_now += 120000; approach_sample(-80, true, fake_now); CHECK(!answer());

    reset(); arm(2); approach_disconnected(); CHECK(approach_connected());
    fake_now += RECONNECT_LIFE_US + 1;
    approach_sample(-60, true, fake_now);
    fake_now += 120000; approach_sample(-60, true, fake_now); CHECK(!pending);
    reset(); arm(2); approach_disconnected(); CHECK(approach_connected());
    fake_now += 120000; approach_sample(-60, false, fake_now);
    fake_now += 120000; approach_sample(-60, true, fake_now); CHECK(!pending && !departed);
    reset(); arm(0); CHECK(!approach_connected());

    return test_report("approach");
}
