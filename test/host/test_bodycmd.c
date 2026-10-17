// host tests for main/bodycmd.c: parsing, seat and lock burst shapes, merging,
// ordering between the two.
//
// bodycmd.c is #included (not linked) so the test shares its translation unit.

#include <stdbool.h>
#include <string.h>
#include "test_support.h"
#include "bodycmd.c"

// ---- can_send recorder ----

static twai_message_t sent_frames[64];
static int sent_count = 0;

esp_err_t can_send(can_bus_t bus, twai_message_t *message, TickType_t ticks_to_wait)
{
    (void)bus;
    (void)ticks_to_wait;
    if (sent_count < 64) {
        sent_frames[sent_count] = *message;
    }
    sent_count++;
    return ESP_OK;
}

static void reset(void)
{
    atomic_store(&pending_seats, 0);
    atomic_store(&pending_lock, false);
    burst = BURST_NONE;
    step = 0;
    sent_count = 0;
    atomic_store(&busy, false);
}

static void run_ticks(int n, bool link_up)
{
    for (int i = 0; i < n; i++) {
        bodycmd_tick(link_up);
    }
}

static void check_bytes(int i, const uint8_t want[8])
{
    CHECK(sent_frames[i].identifier == 0x4A2U);
    CHECK(sent_frames[i].data_length_code == 8);
    CHECK(memcmp(sent_frames[i].data, want, 8) == 0);
}

static void check_seat(int i, uint8_t d5, uint8_t d6)
{
    const uint8_t want[8] = { 0x00, 0x00, 0xF0, 0x03, d5, d6, 0x00, 0x00 };
    check_bytes(i, want);
}

static void test_parse_seat(void)
{
    uint16_t v = 0;
    CHECK(bodycmd_parse_seat("", &v) == BODYCMD_CMD_QUERY);
    CHECK(bodycmd_parse_seat("8fff", &v) == BODYCMD_CMD_SET && v == 0x8FFF);
    CHECK(bodycmd_parse_seat("5F68", &v) == BODYCMD_CMD_SET && v == 0x5F68);
    CHECK(bodycmd_parse_seat("2222", &v) == BODYCMD_CMD_SET && v == 0x2222);
    CHECK(bodycmd_parse_seat("ffff", &v) == BODYCMD_CMD_INVALID);
    CHECK(bodycmd_parse_seat("9fff", &v) == BODYCMD_CMD_INVALID);
    CHECK(bodycmd_parse_seat("1fff", &v) == BODYCMD_CMD_INVALID);
    CHECK(bodycmd_parse_seat("ff3f", &v) == BODYCMD_CMD_INVALID);
    CHECK(bodycmd_parse_seat("ff7f", &v) == BODYCMD_CMD_INVALID);
    CHECK(bodycmd_parse_seat("8ff", &v) == BODYCMD_CMD_INVALID);
    CHECK(bodycmd_parse_seat("8ffff", &v) == BODYCMD_CMD_INVALID);
    CHECK(bodycmd_parse_seat("8fgf", &v) == BODYCMD_CMD_INVALID);
}

static void test_parse_lock(void)
{
    CHECK(bodycmd_parse_lock("") == BODYCMD_CMD_QUERY);
    CHECK(bodycmd_parse_lock("1") == BODYCMD_CMD_SET);
    CHECK(bodycmd_parse_lock("0") == BODYCMD_CMD_INVALID);   // no unlock
    CHECK(bodycmd_parse_lock("2") == BODYCMD_CMD_INVALID);
    CHECK(bodycmd_parse_lock("11") == BODYCMD_CMD_INVALID);
}

static void test_seat_burst(void)
{
    reset();
    bodycmd_request_seats(0x8FFF);
    run_ticks(8, true);
    CHECK(sent_count == 6);
    for (int i = 0; i < 3; i++) check_seat(i, 0x8F, 0xFF);
    for (int i = 3; i < 6; i++) check_seat(i, 0xFF, 0xFF);
    CHECK(!atomic_load(&busy));
}

static void test_seat_merge(void)
{
    reset();
    bodycmd_request_seats(0x8FFF);
    bodycmd_request_seats(0xF5FF);
    bodycmd_request_seats(0x6FFF);
    run_ticks(6, true);
    CHECK(sent_count == 6);
    check_seat(0, 0x65, 0xFF);
}

static void test_lock_sequence(void)
{
    const uint8_t prepare[8] = { 0x00, 0x00, 0xFF, 0x73, 0xFF, 0xFF, 0xFF, 0x00 };
    const uint8_t execute[8] = { 0x00, 0x00, 0xFF, 0xF3, 0xFF, 0xFF, 0x7F, 0x00 };
    const uint8_t finish[8]  = { 0x00, 0x00, 0xFF, 0xF3, 0xFF, 0xFF, 0xFF, 0x00 };
    reset();
    bodycmd_request_lock();
    run_ticks(14, true);
    CHECK(sent_count == 11);
    for (int i = 0; i < 4; i++) check_bytes(i, prepare);
    for (int i = 4; i < 8; i++) check_bytes(i, execute);
    for (int i = 8; i < 11; i++) check_bytes(i, finish);
    CHECK(!atomic_load(&busy));
}

static void test_lock_goes_before_waiting_seats(void)
{
    reset();
    bodycmd_request_seats(0x8FFF);
    bodycmd_request_lock();
    run_ticks(20, true);
    CHECK(sent_count == 17);
    CHECK(sent_frames[0].data[3] == 0x73);   // lock first
    check_seat(11, 0x8F, 0xFF);              // then the seats
}

static void test_bursts_never_interleave(void)
{
    reset();
    bodycmd_request_seats(0x8FFF);
    run_ticks(2, true);
    bodycmd_request_lock();
    run_ticks(20, true);
    CHECK(sent_count == 17);
    check_seat(2, 0x8F, 0xFF);
    check_seat(5, 0xFF, 0xFF);
    CHECK(sent_frames[6].data[3] == 0x73);
}

static void test_no_link_drops_requests(void)
{
    reset();
    bodycmd_request_seats(0x2FFF);
    bodycmd_request_lock();
    run_ticks(1, false);
    run_ticks(20, true);
    CHECK(sent_count == 0);
}

static void test_burst_finishes_after_link_drop(void)
{
    reset();
    bodycmd_request_lock();
    run_ticks(2, true);
    run_ticks(12, false);
    CHECK(sent_count == 11);
}

int main(void)
{
    test_parse_seat();
    test_parse_lock();
    test_seat_burst();
    test_seat_merge();
    test_lock_sequence();
    test_lock_goes_before_waiting_seats();
    test_bursts_never_interleave();
    test_no_link_drops_requests();
    test_burst_finishes_after_link_drop();
    return test_report("test_bodycmd");
}
