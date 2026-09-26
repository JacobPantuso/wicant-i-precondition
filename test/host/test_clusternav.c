// host tests for main/clusternav.c: command parsing, staleness, link loss,
// the per-frame encodings, and the 0x6E7 text sender.
//
// clusternav.c is #included (not linked) so the test shares its translation unit.

#include <stdbool.h>
#include <string.h>
#include "test_support.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "clusternav.c"

// ---- FIFO fake queue: clusternav.c only creates, sends and receives ----

#define FAKE_Q_LEN CLUSTERNAV_QUEUE_LEN
static clusternav_request_t fake_q[FAKE_Q_LEN];
static int fake_q_head = 0;
static int fake_q_count = 0;
static int fake_q_handle;

QueueHandle_t xQueueCreate(UBaseType_t len, UBaseType_t item_size)
{
    CHECK(len == FAKE_Q_LEN);
    CHECK(item_size == sizeof(clusternav_request_t));
    return &fake_q_handle;
}

BaseType_t xQueueSend(QueueHandle_t q, const void *item, TickType_t wait)
{
    (void)q;
    (void)wait;
    if (fake_q_count == FAKE_Q_LEN) {
        return 0;
    }
    memcpy(&fake_q[(fake_q_head + fake_q_count) % FAKE_Q_LEN], item, sizeof(clusternav_request_t));
    fake_q_count++;
    return pdTRUE;
}

BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t wait)
{
    (void)q;
    (void)wait;
    if (fake_q_count == 0) {
        return 0;
    }
    memcpy(item, &fake_q[fake_q_head], sizeof(clusternav_request_t));
    fake_q_head = (fake_q_head + 1) % FAKE_Q_LEN;
    fake_q_count--;
    return pdTRUE;
}

// ---- recording CAN sink ----

#define MAX_SENT 64
static uint8_t sent[MAX_SENT][8];
static uint32_t sent_id[MAX_SENT];
static int nsent = 0;
static bool sink_ok = true;

static bool record_send(const twai_message_t *m)
{
    if (!sink_ok) {
        return false;
    }
    CHECK(m->data_length_code == 8 && !m->extd);
    if (nsent < MAX_SENT) {
        memcpy(sent[nsent], m->data, 8);
        sent_id[nsent] = m->identifier;
    }
    nsent++;
    return true;
}

static bool frame_is(int i, const uint8_t want[8])
{
    return i < nsent && i < MAX_SENT && memcmp(sent[i], want, 8) == 0;
}

// ---- helpers ----

// A right turn in 250 m as four frames, 22 hex digits each (id, mask, d0..7):
// 4CC and 63E whole, 640 only its approach bar (d7 = 80%), 4E8 whole.
#define RIGHT_TURN \
    "04ccff0d00000cfa000000" \
    "063eff0000fa00b0040000" \
    "0640800000000000000050" \
    "04e8ff3110180b0c005000"

static const uint8_t IDLE_4CC[8] = {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t IDLE_63E[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00};
static const uint8_t IDLE_640[8] = {0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x00};
static const uint8_t IDLE_641[8] = {0xFF, 0xFF, 0xFF, 0x18, 0x00, 0xFF, 0xFF, 0xFF};
static const uint8_t IDLE_4E8[8] = {0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x00};

static twai_message_t frame(uint32_t id, const uint8_t data[8])
{
    twai_message_t m;
    memset(&m, 0, sizeof(m));
    m.identifier = id;
    m.data_length_code = 8;
    memcpy(m.data, data, 8);
    return m;
}

static void command(const char *arg, int64_t now)
{
    clusternav_state_t s;
    switch (clusternav_parse_command(arg, &s)) {
    case CLUSTERNAV_CMD_SET:
        clusternav_request(&s);
        break;
    case CLUSTERNAV_CMD_CLEAR:
        clusternav_request(NULL);
        break;
    default:
        CHECK_MSG(0, "unexpected command '%s'", arg);
    }
    clusternav_tick(now, true);
}

// ---- tests: maneuver frames ----

static void test_parse(void)
{
    clusternav_state_t s;
    CHECK(clusternav_parse_command("", &s) == CLUSTERNAV_CMD_QUERY);
    CHECK(clusternav_parse_command("0", &s) == CLUSTERNAV_CMD_CLEAR);

    CHECK(clusternav_parse_command("1" RIGHT_TURN, &s) == CLUSTERNAV_CMD_SET);
    CHECK(s.count == 4);
    CHECK(s.frames[0].id == 0x4CC && s.frames[0].mask == 0xFF);
    CHECK(s.frames[0].data[0] == 0x0D && s.frames[0].data[3] == 0x0C && s.frames[0].data[4] == 0xFA);
    CHECK(s.frames[1].id == 0x63E && s.frames[1].data[0] == 0x00 && s.frames[1].data[4] == 0xB0);
    CHECK(s.frames[2].id == 0x640 && s.frames[2].mask == 0x80 && s.frames[2].data[7] == 0x50);
    CHECK(s.frames[3].id == 0x4E8 && s.frames[3].data[6] == 0x50);

    // one frame, upper case
    CHECK(clusternav_parse_command("104CCFF0D00000CFA000000", &s) == CLUSTERNAV_CMD_SET && s.count == 1);
    // 641 and 4EA are nav frames too
    CHECK(clusternav_parse_command("10641ff0000000000000000", &s) == CLUSTERNAV_CMD_SET);
    CHECK(clusternav_parse_command("104ea400000000000001000", &s) == CLUSTERNAV_CMD_SET);
    // anything else is refused
    CHECK(clusternav_parse_command("100c7ff0000000000000000", &s) == CLUSTERNAV_CMD_INVALID);
    // a partial frame
    CHECK(clusternav_parse_command("104ccff0d00000cfa0000", &s) == CLUSTERNAV_CMD_INVALID);
    // not hex
    CHECK(clusternav_parse_command("104ccff0d00000cfa00000g", &s) == CLUSTERNAV_CMD_INVALID);
    CHECK(clusternav_parse_command("1", &s) == CLUSTERNAV_CMD_INVALID);
    CHECK(clusternav_parse_command("2", &s) == CLUSTERNAV_CMD_INVALID);
    CHECK(clusternav_parse_command("00", &s) == CLUSTERNAV_CMD_INVALID);

    char many[2 + 22 * 6 + 1];
    strcpy(many, "1");
    for (int i = 0; i < 5; i++) {
        strcat(many, "04ccff0d00000cfa000000");
    }
    CHECK(clusternav_parse_command(many, &s) == CLUSTERNAV_CMD_SET && s.count == 5);
    strcat(many, "04ccff0d00000cfa000000");
    CHECK(clusternav_parse_command(many, &s) == CLUSTERNAV_CMD_INVALID);
}

static void test_passthrough_until_set(void)
{
    twai_message_t m = frame(0x4CC, IDLE_4CC);
    CHECK(clusternav_fwd(&m, 0) == FWD_PASSTHROUGH);
    CHECK(memcmp(m.data, IDLE_4CC, 8) == 0);
}

static void test_masked_overwrite(void)
{
    command("0", 0);
    command("1" RIGHT_TURN, 1000);
    twai_message_t m;

    m = frame(0x4CC, IDLE_4CC);
    CHECK(clusternav_fwd(&m, 2000) == FWD_MODIFIED);
    const uint8_t want_4cc[8] = {0x0D, 0x00, 0x00, 0x0C, 0xFA, 0x00, 0x00, 0x00};
    CHECK(memcmp(m.data, want_4cc, 8) == 0);

    m = frame(0x63E, IDLE_63E);
    CHECK(clusternav_fwd(&m, 2000) == FWD_MODIFIED);
    const uint8_t want_63e[8] = {0x00, 0x00, 0xFA, 0x00, 0xB0, 0x04, 0x00, 0x00};
    CHECK(memcmp(m.data, want_63e, 8) == 0);

    // mask 0x80: only d7 is replaced
    m = frame(0x640, IDLE_640);
    CHECK(clusternav_fwd(&m, 2000) == FWD_MODIFIED);
    const uint8_t want_640[8] = {0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x50};
    CHECK(memcmp(m.data, want_640, 8) == 0);

    m = frame(0x4E8, IDLE_4E8);
    CHECK(clusternav_fwd(&m, 2000) == FWD_MODIFIED);
    const uint8_t want_4e8[8] = {0x31, 0x10, 0x18, 0x0B, 0x0C, 0x00, 0x50, 0x00};
    CHECK(memcmp(m.data, want_4e8, 8) == 0);

    // a nav frame the payload doesn't mention
    m = frame(0x641, IDLE_641);
    CHECK(clusternav_fwd(&m, 2000) == FWD_PASSTHROUGH);
    CHECK(memcmp(m.data, IDLE_641, 8) == 0);
}

static void test_zero_mask_passes_through(void)
{
    command("104cc000d00000cfa000000", 0);
    twai_message_t m = frame(0x4CC, IDLE_4CC);
    CHECK(clusternav_fwd(&m, 10) == FWD_PASSTHROUGH);
    CHECK(memcmp(m.data, IDLE_4CC, 8) == 0);
}

static void test_stale(void)
{
    command("0", 0);
    command("1" RIGHT_TURN, 1000000);
    twai_message_t m;

    m = frame(0x4CC, IDLE_4CC);
    CHECK(clusternav_fwd(&m, 1000000 + CLUSTERNAV_STALE_US - 1) == FWD_MODIFIED);
    m = frame(0x4CC, IDLE_4CC);
    CHECK(clusternav_fwd(&m, 1000000 + CLUSTERNAV_STALE_US) == FWD_PASSTHROUGH);
    CHECK(memcmp(m.data, IDLE_4CC, 8) == 0);

    // a fresh update revives it
    command("1" RIGHT_TURN, 1000000 + CLUSTERNAV_STALE_US + 5);
    m = frame(0x4CC, IDLE_4CC);
    CHECK(clusternav_fwd(&m, 1000000 + CLUSTERNAV_STALE_US + 10) == FWD_MODIFIED);
}

static void test_link_down_clears(void)
{
    command("1" RIGHT_TURN, 0);
    clusternav_tick(40000, false);
    twai_message_t m = frame(0x4CC, IDLE_4CC);
    CHECK(clusternav_fwd(&m, 50000) == FWD_PASSTHROUGH);

    // the link coming back isn't enough: the app has to send a new state
    clusternav_tick(80000, true);
    m = frame(0x4CC, IDLE_4CC);
    CHECK(clusternav_fwd(&m, 90000) == FWD_PASSTHROUGH);
}

static void test_clear(void)
{
    command("1" RIGHT_TURN, 0);
    command("0", 10);
    twai_message_t m = frame(0x4CC, IDLE_4CC);
    CHECK(clusternav_fwd(&m, 20) == FWD_PASSTHROUGH);
}

static void test_ignores_extended_and_short(void)
{
    command("1" RIGHT_TURN, 0);
    twai_message_t m = frame(0x4CC, IDLE_4CC);
    m.extd = 1;
    CHECK(clusternav_fwd(&m, 10) == FWD_PASSTHROUGH);
    m = frame(0x4CC, IDLE_4CC);
    m.data_length_code = 4;
    CHECK(clusternav_fwd(&m, 10) == FWD_PASSTHROUGH);
}

static void test_status(void)
{
    command("0", 0);
    clusternav_status_t st;
    clusternav_get_status(&st);
    CHECK(!st.active);
    uint32_t updates_before = st.updates;
    uint32_t injected_before = st.injected;

    command("1" RIGHT_TURN, 100);
    twai_message_t m = frame(0x4CC, IDLE_4CC);
    clusternav_fwd(&m, 200);
    clusternav_get_status(&st);
    CHECK(st.active);
    CHECK(st.updates == updates_before + 1);
    CHECK(st.injected == injected_before + 1);
}

// ---- tests: multi-frame text ----
//
// These run on their own ever-increasing clock: the sender paces off the
// time of its last frame, which an earlier test's clock would sit in the
// future of.

static int64_t t_us = 100000000;

// "Carlton St" as F0 on 0x6E7, exactly as the head unit sent it
#define CARLTON "06e7f04300610072006c0074006f006e00200053007400"

static const uint8_t BLANK_F0[8] = {0x03, 0xF0, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA};
static const uint8_t BLANK_F4[8] = {0x03, 0xF4, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA};

static void text(const char *hex)
{
    clusternav_text_t payload;
    CHECK_MSG(clusternav_parse_text(hex, &payload) == CLUSTERNAV_CMD_SET, "'%s'", hex);
    clusternav_request_text(&payload);
    clusternav_tick(t_us, true);
}

static void live_state(void)
{
    command("1" RIGHT_TURN, t_us);
}

// polls every 1 ms for `ms` milliseconds
static void run_ms(int ms)
{
    for (int i = 0; i < ms; i++) {
        t_us += 1000;
        clusternav_poll(t_us);
    }
}

static void test_parse_text(void)
{
    clusternav_text_t p;
    CHECK(clusternav_parse_text("", &p) == CLUSTERNAV_CMD_QUERY);
    CHECK(clusternav_parse_text("06e7f04100", &p) == CLUSTERNAV_CMD_SET);
    CHECK(p.id == 0x6E7 && p.len == 3 && p.data[0] == 0xF0 && p.data[1] == 0x41);
    CHECK(clusternav_parse_text("06DFF24100", &p) == CLUSTERNAV_CMD_SET && p.id == 0x6DF);
    CHECK(clusternav_parse_text("0680f10000", &p) == CLUSTERNAV_CMD_SET && p.id == 0x680);
    CHECK(clusternav_parse_text("0641f04100", &p) == CLUSTERNAV_CMD_INVALID);   // not a text id
    CHECK(clusternav_parse_text("06e7f14100", &p) == CLUSTERNAV_CMD_INVALID);   // F1 isn't a 6E7 field
    CHECK(clusternav_parse_text("06e7f0410", &p) == CLUSTERNAV_CMD_INVALID);    // odd length
    CHECK(clusternav_parse_text("06e7f041", &p) == CLUSTERNAV_CMD_INVALID);     // shorter than a blank
    CHECK(clusternav_parse_text("06e", &p) == CLUSTERNAV_CMD_INVALID);
    CHECK(clusternav_parse_text("06e7f0zz00", &p) == CLUSTERNAV_CMD_INVALID);

    char hex[4 + 2 * CLUSTERNAV_TEXT_MAX + 8];
    strcpy(hex, "0680f1");
    for (int i = 1; i < CLUSTERNAV_TEXT_MAX; i++) {
        strcat(hex, "00");
    }
    CHECK(clusternav_parse_text(hex, &p) == CLUSTERNAV_CMD_SET && p.len == CLUSTERNAV_TEXT_MAX);
    strcat(hex, "00");
    CHECK(clusternav_parse_text(hex, &p) == CLUSTERNAV_CMD_INVALID);
}

static void test_text_waits_for_live(void)
{
    command("0", t_us);
    nsent = 0;
    text("06e7f04100");
    run_ms(50);
    CHECK(nsent == 0);

    live_state();
    run_ms(50);
    const uint8_t want[8] = {0x03, 0xF0, 0x41, 0x00, 0xAA, 0xAA, 0xAA, 0xAA};
    CHECK(nsent == 1 && frame_is(0, want) && sent_id[0] == 0x6E7);

    // unchanged text isn't resent
    text("06e7f04100");
    run_ms(50);
    CHECK(nsent == 1);
}

static void test_text_multi_frame(void)
{
    live_state();
    nsent = 0;
    text(CARLTON);
    t_us += 10000;
    clusternav_poll(t_us);
    CHECK(nsent == 1);
    clusternav_poll(t_us + CLUSTERNAV_TEXT_GAP_US - 1);
    CHECK(nsent == 1);   // paced like the head unit
    t_us += CLUSTERNAV_TEXT_GAP_US;
    clusternav_poll(t_us);
    CHECK(nsent == 2);
    run_ms(30);
    CHECK(nsent == 4);

    const uint8_t ff[8]  = {0x10, 0x15, 0xF0, 0x43, 0x00, 0x61, 0x00, 0x72};
    const uint8_t cf1[8] = {0x21, 0x00, 0x6C, 0x00, 0x74, 0x00, 0x6F, 0x00};
    const uint8_t cf2[8] = {0x22, 0x6E, 0x00, 0x20, 0x00, 0x53, 0x00, 0x74};
    const uint8_t cf3[8] = {0x23, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
    CHECK(frame_is(0, ff));
    CHECK(frame_is(1, cf1));
    CHECK(frame_is(2, cf2));
    CHECK(frame_is(3, cf3));
}

static void test_text_on_hud_id(void)
{
    live_state();
    nsent = 0;
    text("06dff04100");
    run_ms(20);
    const uint8_t want[8] = {0x03, 0xF0, 0x41, 0x00, 0xAA, 0xAA, 0xAA, 0xAA};
    CHECK(nsent == 1 && frame_is(0, want) && sent_id[0] == 0x6DF);
}

static void test_text_retries_failed_send(void)
{
    live_state();
    nsent = 0;
    text("06e7f04200");
    sink_ok = false;
    run_ms(20);
    CHECK(nsent == 0);
    sink_ok = true;
    run_ms(20);
    CHECK(nsent == 1 && sent[0][2] == 0x42);
}

static void test_text_blanked_when_stale(void)
{
    live_state();
    text("06e7f04300");
    run_ms(20);
    nsent = 0;

    t_us += CLUSTERNAV_STALE_US;
    run_ms(40);
    // F0 on 6E7 and on 6DF (from test_text_on_hud_id) both blank
    CHECK(nsent == 2);
    CHECK(frame_is(0, BLANK_F0) && sent_id[0] == 0x6E7);
    CHECK(frame_is(1, BLANK_F0) && sent_id[1] == 0x6DF);

    // a lapse keeps the text: the next state brings both back
    nsent = 0;
    live_state();
    run_ms(40);
    CHECK(nsent == 2 && sent[0][2] == 0x43);
}

static void test_text_forgotten_on_clear(void)
{
    live_state();
    text("06e7f4004100");
    run_ms(20);
    nsent = 0;

    command("0", t_us);
    run_ms(40);
    CHECK(nsent == 3);
    CHECK(frame_is(0, BLANK_F0) && sent_id[0] == 0x6E7);
    CHECK(frame_is(1, BLANK_F4) && sent_id[1] == 0x6E7);
    CHECK(frame_is(2, BLANK_F0) && sent_id[2] == 0x6DF);

    // ATXNV0 ended the trip: its text doesn't come back with a new one
    nsent = 0;
    live_state();
    run_ms(20);
    CHECK(nsent == 0);
}

static void test_text_forgotten_on_link_down(void)
{
    live_state();
    text("06e7f04400");
    run_ms(20);
    nsent = 0;

    clusternav_tick(t_us, false);
    run_ms(20);
    CHECK(nsent == 1 && frame_is(0, BLANK_F0));
}

static void test_f1_graphic_and_its_blank(void)
{
    // the 81-byte F1 the built-in nav sent on 0x680 (IProbe drive)
    char hex[4 + 2 * 81 + 1];
    strcpy(hex, "0680f10200030700");
    for (int i = 6; i < 81; i++) {
        strcat(hex, "00");
    }
    live_state();
    nsent = 0;
    text(hex);
    run_ms(100);
    CHECK(nsent == 12);   // first frame + 11 consecutive
    const uint8_t ff[8] = {0x10, 0x51, 0xF1, 0x02, 0x00, 0x03, 0x07, 0x00};
    CHECK(frame_is(0, ff) && sent_id[0] == 0x680);

    // lapsing sends F1 + 80 zeros, framed exactly like the head unit's
    nsent = 0;
    t_us += CLUSTERNAV_STALE_US;
    run_ms(100);
    CHECK(nsent == 12);
    const uint8_t blank_ff[8] = {0x10, 0x51, 0xF1, 0x00, 0x00, 0x00, 0x00, 0x00};
    const uint8_t blank_last[8] = {0x2B, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA, 0xAA};
    CHECK(frame_is(0, blank_ff));
    CHECK(frame_is(11, blank_last));
    // end the trip so the F1 message doesn't come back in the next test
    command("0", t_us);
}

static void test_text_status(void)
{
    clusternav_status_t before, after;
    clusternav_get_status(&before);
    live_state();
    text(CARLTON);
    run_ms(40);
    clusternav_get_status(&after);
    CHECK(after.texts_sent == before.texts_sent + 1);
    CHECK(after.text_frames == before.text_frames + 4);
}

int main(void)
{
    clusternav_init(record_send);

    test_parse();
    test_passthrough_until_set();
    test_masked_overwrite();
    test_zero_mask_passes_through();
    test_stale();
    test_link_down_clears();
    test_clear();
    test_ignores_extended_and_short();
    test_status();

    test_parse_text();
    test_text_waits_for_live();
    test_text_multi_frame();
    test_text_on_hud_id();
    test_text_retries_failed_send();
    test_text_blanked_when_stale();
    test_text_forgotten_on_clear();
    test_text_forgotten_on_link_down();
    test_f1_graphic_and_its_blank();
    test_text_status();

    return test_report("test_clusternav");
}
