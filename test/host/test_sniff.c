// host tests for main/sniff.c: filtering, line format, commands, lifecycle.
//
// sniff.c is #included (not linked) so the test shares its translation unit.

#include <stdbool.h>
#include <string.h>
#include "test_support.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sniff.c"

// ---- 1-deep fake queue: sniff.c only creates, overwrites and receives ----

static sniff_config_t fake_q_item;
static bool fake_q_full = false;
static int fake_q_handle;

QueueHandle_t xQueueCreate(UBaseType_t len, UBaseType_t item_size)
{
    (void)len;
    CHECK(item_size == sizeof(sniff_config_t));
    return &fake_q_handle;
}

BaseType_t xQueueOverwrite(QueueHandle_t q, const void *item)
{
    (void)q;
    memcpy(&fake_q_item, item, sizeof(fake_q_item));
    fake_q_full = true;
    return pdTRUE;
}

BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t wait)
{
    (void)q;
    (void)wait;
    if (!fake_q_full) {
        return 0;
    }
    memcpy(item, &fake_q_item, sizeof(fake_q_item));
    fake_q_full = false;
    return pdTRUE;
}

// ---- recording sink ----

#define MAX_LINES 1024
static char lines[MAX_LINES][64];
static int nlines = 0;
static bool sink_accepts = true;

static bool record_sink(const char *line, uint8_t len)
{
    if (!sink_accepts) {
        return false;
    }
    CHECK(len > 0 && len < 64 && line[len - 1] == '\r');
    if (nlines < MAX_LINES) {
        memcpy(lines[nlines], line, len);
        lines[nlines][len] = 0;
        nlines++;
    }
    return true;
}

static twai_message_t frame(uint32_t id, uint8_t dlc, const uint8_t *data)
{
    twai_message_t m;
    memset(&m, 0, sizeof(m));
    m.identifier = id;
    m.data_length_code = dlc;
    if (data != NULL) {
        memcpy(m.data, data, dlc > 8 ? 8 : dlc);
    }
    return m;
}

static void command(const char *arg, int64_t now)
{
    sniff_config_t cfg;
    CHECK_MSG(sniff_parse_command(arg, &cfg) == SNIFF_CMD_SET, "arg '%s'", arg);
    CHECK(sniff_request(&cfg));
    sniff_tick(now, true);
}

static void fresh(const char *arg)
{
    sink_accepts = true;
    nlines = 0;
    command(arg, 0);
    nlines = 0;
}

static sniff_status_t status(void)
{
    sniff_status_t s;
    sniff_get_status(&s);
    return s;
}

static const uint8_t A[8] = {0x03, 0x22, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t B[8] = {0x03, 0x22, 0x96, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t C[8] = {0x32, 0x22, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00};

static void test_parse(void)
{
    sniff_config_t cfg;

    CHECK(sniff_parse_command("", &cfg) == SNIFF_CMD_QUERY);
    CHECK(sniff_parse_command("0", &cfg) == SNIFF_CMD_SET && !cfg.enable);
    CHECK(sniff_parse_command("1", &cfg) == SNIFF_CMD_SET && cfg.enable && cfg.npins == 0);

    CHECK(sniff_parse_command("163e,4cc,6e7", &cfg) == SNIFF_CMD_SET);
    CHECK(cfg.enable && cfg.npins == 3);
    CHECK(cfg.pins[0] == 0x63E && cfg.pins[1] == 0x4CC && cfg.pins[2] == 0x6E7);

    CHECK(sniff_parse_command("11fffffff", &cfg) == SNIFF_CMD_SET && cfg.pins[0] == 0x1FFFFFFF);

    CHECK(sniff_parse_command("163e", &cfg) == SNIFF_CMD_SET && !cfg.pins_only);
    CHECK(sniff_parse_command("1p63e,4cc", &cfg) == SNIFF_CMD_SET);
    CHECK(cfg.pins_only && cfg.npins == 2 && cfg.pins[1] == 0x4CC);
    CHECK(sniff_parse_command("1p", &cfg) == SNIFF_CMD_INVALID);
    CHECK(sniff_parse_command("120000000", &cfg) == SNIFF_CMD_INVALID);   // > 29 bits
    CHECK(sniff_parse_command("1123456789", &cfg) == SNIFF_CMD_INVALID);  // 9 digits

    CHECK(sniff_parse_command("1,", &cfg) == SNIFF_CMD_INVALID);
    CHECK(sniff_parse_command("163e,", &cfg) == SNIFF_CMD_INVALID);
    CHECK(sniff_parse_command("163e,,4cc", &cfg) == SNIFF_CMD_INVALID);
    CHECK(sniff_parse_command("1xyz", &cfg) == SNIFF_CMD_INVALID);
    CHECK(sniff_parse_command("2", &cfg) == SNIFF_CMD_INVALID);
    CHECK(sniff_parse_command("0abc", &cfg) == SNIFF_CMD_INVALID);

    CHECK(sniff_parse_command("11,2,3,4,5,6,7,8,9,a,b,c,d,e,f,10", &cfg) == SNIFF_CMD_SET);
    CHECK(cfg.npins == 16 && cfg.pins[15] == 0x10);
    CHECK(sniff_parse_command("11,2,3,4,5,6,7,8,9,a,b,c,d,e,f,10,11", &cfg) == SNIFF_CMD_INVALID);
}

static void test_disabled_until_started(void)
{
    nlines = 0;
    twai_message_t m = frame(0x63E, 8, A);
    sniff_rx(&m, 0);
    CHECK(nlines == 0);
    CHECK(!status().enabled);
}

static void test_line_format(void)
{
    fresh("1");

    twai_message_t m = frame(0x63E, 8, A);
    sniff_rx(&m, 0x0001A2F3);
    CHECK_MSG(strcmp(lines[0], "$0000 0001A2F3 63E 8 0322FF0000000000\r") == 0, "got '%s'", lines[0]);

    uint8_t ext_data[2] = {0xAB, 0xCD};
    m = frame(0x18DAF110, 2, ext_data);
    m.extd = 1;
    sniff_rx(&m, 0x10);
    CHECK_MSG(strcmp(lines[1], "$0001 00000010 18DAF110 2 ABCD\r") == 0, "got '%s'", lines[1]);

    m = frame(0x123, 0, NULL);
    sniff_rx(&m, 0x20);
    CHECK_MSG(strcmp(lines[2], "$0002 00000020 123 0\r") == 0, "got '%s'", lines[2]);

    // DLC over 8 is clamped; the clock wraps at 32 bits
    m = frame(0x456, 12, A);
    sniff_rx(&m, 0x100000005LL);
    CHECK_MSG(strcmp(lines[3], "$0003 00000005 456 8 0322FF0000000000\r") == 0, "got '%s'", lines[3]);

    CHECK(nlines == 4);
}

static void test_change_only_and_rate_cap(void)
{
    fresh("1");
    twai_message_t a = frame(0x63E, 8, A);
    twai_message_t b = frame(0x63E, 8, B);
    twai_message_t c = frame(0x63E, 8, C);

    sniff_rx(&a, 0);          // first sighting: sent
    sniff_rx(&a, 10000);      // unchanged: suppressed
    sniff_rx(&a, 300000);     // unchanged: suppressed
    CHECK(nlines == 1);

    sniff_rx(&b, 400000);     // changed, interval passed: sent
    sniff_rx(&b, 410000);     // unchanged: suppressed
    sniff_rx(&c, 450000);     // changed, but only 50 ms after b: held back
    CHECK(nlines == 2);

    sniff_rx(&c, 650000);     // still differs from the last *sent* (b): sent
    CHECK(nlines == 3);
    CHECK(strstr(lines[2], " 63E 8 3222640000000000\r") != NULL);

    sniff_status_t s = status();
    CHECK(s.rx_total == 7);
    CHECK(s.sent == 3);
    CHECK(s.suppressed == 4);
    CHECK(s.ids == 1);
}

static void test_pinned_pass_everything(void)
{
    fresh("16e7");
    twai_message_t m = frame(0x6E7, 8, A);
    for (int i = 0; i < 5; i++) {
        sniff_rx(&m, i * 1000);   // identical and 1 ms apart
    }
    CHECK(nlines == 5);
    CHECK(status().ids == 0);     // pinned identifiers never enter the table
    CHECK(status().npins == 1);
}

static void test_pins_only(void)
{
    fresh("1p6e7");
    twai_message_t pinned = frame(0x6E7, 8, A);
    twai_message_t other = frame(0x63E, 8, A);
    sniff_rx(&other, 0);          // would pass in the normal mode: first sighting
    sniff_rx(&pinned, 1000);
    sniff_rx(&pinned, 2000);
    CHECK(nlines == 2);
    CHECK(status().suppressed == 1);
    CHECK(status().ids == 0);     // nothing unpinned enters the table
}

static void test_dropped_line_is_retried(void)
{
    fresh("1");
    twai_message_t a = frame(0x63E, 8, A);

    sink_accepts = false;
    sniff_rx(&a, 0);
    CHECK(nlines == 0);
    CHECK(status().dropped == 1);

    sink_accepts = true;
    sniff_rx(&a, 1000);           // not remembered as sent, so it goes again
    CHECK(nlines == 1);
}

static void test_table_full_sends_unfiltered(void)
{
    fresh("1");
    for (uint32_t id = 0; id < SNIFF_TABLE_SIZE; id++) {
        twai_message_t m = frame(id, 8, A);
        sniff_rx(&m, 0);
    }
    CHECK(nlines == SNIFF_TABLE_SIZE);
    CHECK(status().ids == SNIFF_TABLE_SIZE);

    twai_message_t overflow = frame(0x700, 8, A);
    sniff_rx(&overflow, 1000);
    sniff_rx(&overflow, 2000);
    CHECK(nlines == SNIFF_TABLE_SIZE + 2);
}

static void test_stats_line(void)
{
    fresh("1");
    twai_message_t a = frame(0x100, 8, A);
    twai_message_t b = frame(0x200, 8, A);
    twai_message_t c = frame(0x300, 8, A);
    sniff_rx(&a, 1000);
    sniff_rx(&a, 2000);           // suppressed
    sniff_rx(&b, 3000);
    sniff_rx(&c, 4000);
    CHECK(nlines == 3);

    sniff_tick(500000, true);
    CHECK(nlines == 3);           // not due yet

    sniff_tick(1000000, true);
    CHECK(nlines == 4);
    CHECK_MSG(strcmp(lines[3], "$0003 ! 4 3 0 1 3\r") == 0, "got '%s'", lines[3]);
}

static void test_link_down_stops(void)
{
    fresh("1");
    sniff_tick(40000, false);
    CHECK(!status().enabled);

    twai_message_t m = frame(0x63E, 8, A);
    sniff_rx(&m, 50000);
    CHECK(nlines == 0);

    // a start that lands while the link is down never takes effect
    sniff_config_t cfg;
    sniff_parse_command("1", &cfg);
    sniff_request(&cfg);
    sniff_tick(80000, false);
    CHECK(!status().enabled);
}

static void test_stop_then_restart_resets(void)
{
    fresh("1");
    twai_message_t m = frame(0x63E, 8, A);
    sniff_rx(&m, 0);
    CHECK(nlines == 1);

    command("0", 1000);
    CHECK(!status().enabled);
    sniff_rx(&m, 2000);
    CHECK(nlines == 1);

    command("1", 3000);
    sniff_rx(&m, 4000);           // table cleared, so the same frame is new again
    CHECK(nlines == 2);
    CHECK(status().rx_total == 1);
}

static void test_sequence_numbers(void)
{
    fresh("16e7");            // pinned: every frame passes the filter
    twai_message_t m = frame(0x6E7, 8, A);

    sniff_rx(&m, 1000);
    sink_accepts = false;
    sniff_rx(&m, 2000);       // refused: must not consume a number
    sink_accepts = true;
    sniff_rx(&m, 3000);
    sniff_rx(&m, 4000);

    CHECK(nlines == 3);
    CHECK_MSG(strncmp(lines[0], "$0000 ", 6) == 0, "got '%s'", lines[0]);
    CHECK_MSG(strncmp(lines[1], "$0001 ", 6) == 0, "got '%s'", lines[1]);
    CHECK_MSG(strncmp(lines[2], "$0002 ", 6) == 0, "got '%s'", lines[2]);

    command("1", 5000);       // a restart begins numbering again
    nlines = 0;
    sniff_rx(&m, 6000);
    CHECK_MSG(strncmp(lines[0], "$0000 ", 6) == 0, "got '%s'", lines[0]);
}

int main(void)
{
    sniff_init(record_sink);

    test_parse();
    test_disabled_until_started();
    test_line_format();
    test_change_only_and_rate_cap();
    test_pinned_pass_everything();
    test_pins_only();
    test_dropped_line_is_retried();
    test_table_full_sends_unfiltered();
    test_stats_line();
    test_link_down_stops();
    test_stop_then_restart_resets();
    test_sequence_numbers();

    return test_report("test_sniff");
}
