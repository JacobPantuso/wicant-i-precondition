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
    sample(-55, true, 17); CHECK(!answer()); // expired event (8 s) despite fresh RSSI
    reset(); arm(2); sample(-55, true, 5); challenge();
    body.busy = true; CHECK(!answer() && !unlock_count);
    reset(); arm(2); sample(-55, true, 5); challenge(); ready = true; CHECK(!answer()); ready = false;
    reset(); arm(2); sample(-55, true, 1); fake_now += 10000000;
    sample(-55, true, 1); CHECK(!pending); // gaps do not count toward near dwell
    // A rising approach can use the short close hold, with no EMA lag.
    reset(); arm(2); sample(-88, true, 4);
    sample(-74, true, 1); sample(-66, true, 1); CHECK(!near_since);
    // A median of -66 is 22 dB over a -88 floor: the rising band opens it.
    sample(-63, true, 1); CHECK(fast_candidate && near_since == fake_now && !pending);
    int64_t close_at = fake_now;
    sample(-61, true, 1); CHECK(!pending);
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

    // Today's miss: a walk back from the edge of range with pocket jitter.
    // The median climbs well over the far floor and qualifies in the rising
    // band, before -65 dBm, even though raw readings dip under it.
    reset(); arm(2); approach_disconnected(); CHECK(approach_connected());
    fake_now += RECONNECT_LIFE_US + 1;
    approach_set_sample_period(250000);
    {
        static const int walk[] = { -96, -95, -93, -94, -90, -91, -88, -86, -88, -84,
                                    -82, -84, -80, -78, -81, -76, -74, -79, -72, -71,
                                    -77, -70, -69 };
        int fired_at = -1;
        for (int i = 0; i < (int)(sizeof walk / sizeof walk[0]); i++) {
            fake_now += 250000; approach_sample(walk[i], true, fake_now);
            if (pending && fired_at < 0) fired_at = i;
        }
        CHECK(fired_at >= 0 && signal < NEAR_DBM && event_floor == RISING_DBM);
        challenge(); CHECK(answer() && unlock_count == 1);
    }
    approach_set_sample_period(500000);

    // One raw dip under -65 no longer restarts the steady two-second hold.
    reset(); arm(2); sample(-60, true, 3);
    sample(-70, true, 1); CHECK(!pending && close_since);
    sample(-60, true, 1); CHECK(pending);

    // ...and a pocket dip does not void an issued challenge, a retreat does.
    reset(); arm(2); sample(-60, true, 5); CHECK(pending);
    challenge(); sample(-72, true, 1); CHECK(answer() && unlock_count == 1);

    // A steady pocket walk back at ~1 dB/s (2026-11-20 14:28 waited for -53):
    // the slow-climb rule fires in the -75 band, before the -65 door rule.
    reset(); arm(2); approach_disconnected(); CHECK(approach_connected());
    fake_now += RECONNECT_LIFE_US + 1;
    approach_set_sample_period(250000);
    {
        int fired_at = -1, fired_signal = 0;
        for (int i = 0; i < 8; i++) { fake_now += 250000; approach_sample(-96, true, fake_now); }
        for (int i = 0; i < 160 && fired_at < 0; i++) {
            int level = -92 + i / 4;                       // +1 dB per second
            int wobble = (i % 3 == 0) ? -2 : (i % 3 == 1) ? 1 : 0;
            fake_now += 250000; approach_sample(level + wobble, true, fake_now);
            if (pending) { fired_at = i; fired_signal = signal; }
        }
        CHECK(fired_at >= 0 && fired_signal < NEAR_DBM && fired_signal >= RISING_DBM - 2);
    }
    approach_set_sample_period(500000);

    // A withdrawn offer gets another chance as the owner keeps walking
    // (2026-11-21 10:16: -73, then -84 before the phone answered).
    reset(); arm(2); sample(-95, true, 6); sample(-60, true, 5); CHECK(pending);
    uint32_t first = sequence;
    sample(-90, true, 1); CHECK(!pending && departed);   // fell back: lapsed, still watching
    sample(-60, true, 6); CHECK(pending && sequence == first + 1);
    challenge(); CHECK(answer() && unlock_count == 1);
    // Unanswered offers expire into another chance too, up to three per departure.
    reset(); arm(2); sample(-95, true, 6);
    for (int n = 0; n < 3; n++) {
        sample(-60, true, 6); CHECK(pending);
        sample(-60, true, 18);                            // > 8 s, never answered
    }
    CHECK(!pending && !departed);                         // offers used up
    sample(-60, true, 10); CHECK(!pending);

    // Jitter in the house never had a far floor: no rising-band event.
    reset(); arm(2);
    for (int i = 0; i < 40; i++) sample(i % 3 == 0 ? -82 : i % 3 == 1 ? -72 : -76, true, 1);
    CHECK(!pending && !notifications);

    // A rising-band event lapses once the median falls back past -75 by more
    // than the pocket margin (EVENT_MARGIN_DB).
    reset(); arm(2); sample(-95, true, 6); sample(-78, true, 2); sample(-72, true, 4);
    CHECK(pending && event_floor == RISING_DBM);
    sample(-84, true, 2); challenge(); CHECK(!answer());
    // ...but a median a few dB under it, walking with the phone in a pocket, keeps it.
    reset(); arm(2); sample(-95, true, 6); sample(-78, true, 2); sample(-72, true, 4);
    CHECK(pending); sample(-79, true, 2); challenge(); CHECK(answer());

    // Geofence hint: the phone came back from outside the fence. A walk back
    // from -86 that only climbs 12 dB (2026-11-20 10:21) qualifies at -78
    // instead of waiting for -65; without the hint it doesn't.
    reset(); arm(2); sample(-86, true, 6); sample(-80, true, 2); sample(-74, true, 4);
    CHECK(!pending);
    reset(); arm(2); CHECK(approach_command("a", response, sizeof response));
    sample(-86, true, 6); sample(-80, true, 2); sample(-74, true, 4);
    CHECK(pending && event_floor == HINT_RISING_DBM && !away_hint_until);
    // The hint never helps house jitter: it still needs a far floor.
    reset(); arm(2); CHECK(approach_command("a", response, sizeof response));
    for (int i = 0; i < 40; i++) sample(i % 3 == 0 ? -82 : i % 3 == 1 ? -72 : -76, true, 1);
    CHECK(!pending);
    // A drive clears it.
    reset(); arm(2); CHECK(approach_command("a", response, sizeof response));
    approach_reset_session(); CHECK(!away_hint_until);

    // After a "not started" relock the owner is at the car: standing there
    // never re-unlocks. Walking away (a far floor) or a dropped link re-arms it.
    reset(); arm(2); approach_relocked(); sample(-60, true, 10); CHECK(!pending);
    sample(-90, true, 4); sample(-60, true, 6); CHECK(pending);
    reset(); arm(2); approach_relocked(); approach_disconnected(); CHECK(approach_connected());
    fake_now += 120000; approach_sample(-61, true, fake_now);
    fake_now += 120000; approach_sample(-60, true, fake_now); CHECK(pending);

    // At Home, standby reads every 2 s: those are not gaps and can qualify.
    reset(); arm(2); approach_set_sample_period(2000000);
    for (int i = 0; i < 4; i++) { fake_now += 2000000; approach_sample(-60, true, fake_now); }
    CHECK(pending);
    approach_set_sample_period(500000);

    return test_report("approach");
}
