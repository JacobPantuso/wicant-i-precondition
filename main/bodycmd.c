// Head-unit body requests on 0x4A2. The head unit sends this id only on an
// event, as a short burst 40 ms apart, and it keeps its 11-bit id on M-CAN
// (head-unit frames aren't part of the CCU hub's 29-bit remap).
//
// Seats (measured on an IONIQ 6, 2026-10-16): the new value x3, then idle x3.
//
//   00 00 F0 03 D5 D6 00 00
//
//   D5 high nibble driver, low nibble passenger; D6 high rear left, low rear
//   right. Each nibble is the target, not a toggle: 2 off, 3/4/5 ventilation
//   Low/Medium/High, 6/7/8 heat Low/Medium/High. The rear seats only heat, at
//   2/6/8. F leaves that seat alone.
//
// Lock (community capture; not yet seen on this car, whose idle frame has D3
// F0 and D7 00 where the community's has FF):
//
//   00 00 FF 73 FF FF FF 00  x4   prepare
//   00 00 FF F3 FF FF 7F 00  x4   execute
//   00 00 FF F3 FF FF FF 00  x3   finish
//
// Lock only. The unlock sequence is deliberately not here: anything that can
// reach the WiCAN over BLE can send these commands, and a lock is the only one
// that is harmless in the wrong hands. Every frame is built here; nothing the
// app sends reaches the bytes.

#include <stdatomic.h>
#include <string.h>
#include "bodycmd.h"
#include "can.h"

#ifndef ESP_OK
#define ESP_OK 0
#endif

#define BODY_ID           0x4A2U
#define BODY_BUS          CAN_BUS_0
#define SEAT_VALUE_FRAMES 3
#define SEAT_IDLE_FRAMES  3
#define SEAT_IDLE         0xFFFFU
#define SEAT_PENDING      0x10000U   // set in `pending_seats` alongside the levels
#define LOCK_PREPARE      4
#define LOCK_EXECUTE      4
#define LOCK_FINISH       3

typedef enum { BURST_NONE = 0, BURST_SEATS, BURST_LOCK } burst_t;

// Written by the ELM327 task, taken by the CAN task.
static _Atomic uint32_t pending_seats;   // 0 = nothing waiting
static _Atomic bool pending_lock;

// CAN task only.
static burst_t burst = BURST_NONE;
static int step;
static uint16_t seat_levels;

static _Atomic uint32_t seat_requests;
static _Atomic uint32_t lock_requests;
static _Atomic uint32_t frames;
static _Atomic bool busy;

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool front_ok(int n) { return n == 0xF || (n >= 2 && n <= 8); }
static bool rear_ok(int n)  { return n == 0xF || n == 2 || n == 6 || n == 8; }

bodycmd_cmd_t bodycmd_parse_seat(const char *args, uint16_t *levels)
{
    if (args[0] == 0) {
        return BODYCMD_CMD_QUERY;
    }
    int n[4];
    for (int i = 0; i < 4; i++) {
        n[i] = hex_value(args[i]);
        if (n[i] < 0) {
            return BODYCMD_CMD_INVALID;
        }
    }
    if (args[4] != 0) {
        return BODYCMD_CMD_INVALID;
    }
    if (!front_ok(n[0]) || !front_ok(n[1]) || !rear_ok(n[2]) || !rear_ok(n[3])) {
        return BODYCMD_CMD_INVALID;
    }
    uint16_t value = (uint16_t)((n[0] << 12) | (n[1] << 8) | (n[2] << 4) | n[3]);
    if (value == SEAT_IDLE) {
        return BODYCMD_CMD_INVALID;   // asks for nothing
    }
    *levels = value;
    return BODYCMD_CMD_SET;
}

bodycmd_cmd_t bodycmd_parse_lock(const char *args)
{
    if (args[0] == 0) {
        return BODYCMD_CMD_QUERY;
    }
    if (args[0] == '1' && args[1] == 0) {
        return BODYCMD_CMD_SET;
    }
    return BODYCMD_CMD_INVALID;
}

static uint16_t merge(uint16_t base, uint16_t add)
{
    for (int shift = 0; shift < 16; shift += 4) {
        uint16_t nibble = (uint16_t)((add >> shift) & 0xFU);
        if (nibble != 0xFU) {
            base = (uint16_t)((base & ~(0xFU << shift)) | (nibble << shift));
        }
    }
    return base;
}

void bodycmd_request_seats(uint16_t levels)
{
    uint32_t old = atomic_load(&pending_seats);
    uint32_t merged;
    do {
        uint16_t base = old ? (uint16_t)old : (uint16_t)SEAT_IDLE;
        merged = SEAT_PENDING | merge(base, levels);
    } while (!atomic_compare_exchange_weak(&pending_seats, &old, merged));
    atomic_fetch_add(&seat_requests, 1);
}

void bodycmd_request_lock(void)
{
    atomic_store(&pending_lock, true);
    atomic_fetch_add(&lock_requests, 1);
}

static int burst_length(burst_t b)
{
    return b == BURST_LOCK ? LOCK_PREPARE + LOCK_EXECUTE + LOCK_FINISH
                           : SEAT_VALUE_FRAMES + SEAT_IDLE_FRAMES;
}

static void build_frame(burst_t b, int n, uint8_t data[8])
{
    memset(data, 0, 8);
    if (b == BURST_LOCK) {
        data[2] = 0xFF;
        data[3] = n < LOCK_PREPARE ? 0x73 : 0xF3;
        data[4] = 0xFF;
        data[5] = 0xFF;
        data[6] = (n >= LOCK_PREPARE && n < LOCK_PREPARE + LOCK_EXECUTE) ? 0x7F : 0xFF;
        return;
    }
    uint16_t levels = n < SEAT_VALUE_FRAMES ? seat_levels : (uint16_t)SEAT_IDLE;
    data[2] = 0xF0;
    data[3] = 0x03;
    data[4] = (uint8_t)(levels >> 8);
    data[5] = (uint8_t)(levels & 0xFFU);
}

static void send_frame(const uint8_t data[8])
{
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier = BODY_ID;
    msg.data_length_code = 8;
    memcpy(msg.data, data, 8);
    if (can_send(BODY_BUS, &msg, 1) == ESP_OK) {
        atomic_fetch_add(&frames, 1);
    }
}

void bodycmd_tick(bool link_up)
{
    if (burst == BURST_NONE) {
        // A request from a client that has already gone is dropped. A burst
        // that has started always finishes, so its closing frames go out.
        bool lock = atomic_exchange(&pending_lock, false);
        if (lock && link_up) {
            burst = BURST_LOCK;
        } else {
            uint32_t seats = lock ? 0 : atomic_exchange(&pending_seats, 0);
            if (seats == 0 || !link_up) {
                if (!link_up) {
                    atomic_store(&pending_seats, 0);
                }
                return;
            }
            seat_levels = (uint16_t)seats;
            burst = BURST_SEATS;
        }
        step = 0;
        atomic_store(&busy, true);
    }
    uint8_t data[8];
    build_frame(burst, step, data);
    send_frame(data);
    if (++step == burst_length(burst)) {
        burst = BURST_NONE;
        atomic_store(&busy, false);
    }
}

void bodycmd_get_status(bodycmd_status_t *out)
{
    out->seat_requests = atomic_load(&seat_requests);
    out->lock_requests = atomic_load(&lock_requests);
    out->frames = atomic_load(&frames);
    out->busy = atomic_load(&busy);
}
