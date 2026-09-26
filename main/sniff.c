#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sniff.h"

// Line formats, each ending in '\r':
//   $SSSS TTTTTTTT III D DD...   a frame: line sequence number, low 32 bits of
//                                the microsecond clock, identifier (3 hex
//                                digits standard, 8 extended), DLC, then the
//                                data bytes
//   $SSSS ! rx sent dropped suppressed ids   every SNIFF_STATS_INTERVAL_US
//
// SSSS numbers the lines the sink accepted, from 0000 at each start, wrapping
// at FFFF. A line the sink refuses doesn't consume a number, so a gap seen by
// the receiver means lines were lost after the firmware handed them over.
//
// An unpinned frame passes only when its payload differs from the last one
// sent for that identifier and SNIFF_MIN_INTERVAL_US has passed since. Pinned
// identifiers pass every frame, so multi-frame ISO-TP messages arrive intact.

_Static_assert(SNIFF_TABLE_SIZE == 512, "slot_for() hashes to 9 bits");

#define SNIFF_EXT_KEY   0x80000000U   // folds the IDE bit into the table key
#define SNIFF_DLC_NONE  0xFFU         // slot has never sent a frame

typedef struct {
    uint32_t key;
    int64_t  last_sent_us;
    uint8_t  data[8];
    uint8_t  dlc;
    bool     used;
} sniff_slot_t;

static sniff_sink_t sink_fn = NULL;
static QueueHandle_t request_queue = NULL;
// Allocated only while a session runs, in small chunks. At boot the heap is
// too tight to hold 16 KB for a sniffer nobody is using -- as a static array
// it starved can_rx_task/can_tx_task of their stacks -- and one 16 KB block is
// rarely free once the device is up, so a single calloc failed and the session
// silently streamed nothing (both measured 2026-09-25).
#define SNIFF_CHUNK_SLOTS 32
#define SNIFF_CHUNKS      (SNIFF_TABLE_SIZE / SNIFF_CHUNK_SLOTS)
static sniff_slot_t *chunks[SNIFF_CHUNKS];

static sniff_slot_t *slot_at(uint32_t index)
{
    return &chunks[index / SNIFF_CHUNK_SLOTS][index % SNIFF_CHUNK_SLOTS];
}

static void free_table(void)
{
    for (int i = 0; i < SNIFF_CHUNKS; i++) {
        free(chunks[i]);
        chunks[i] = NULL;
    }
}

// Allocates any missing chunk and zeroes them all. On failure frees
// everything, so the table is either whole or absent.
static bool reset_table(void)
{
    for (int i = 0; i < SNIFF_CHUNKS; i++) {
        if (chunks[i] == NULL) {
            chunks[i] = calloc(SNIFF_CHUNK_SLOTS, sizeof(sniff_slot_t));
            if (chunks[i] == NULL) {
                free_table();
                return false;
            }
        } else {
            memset(chunks[i], 0, SNIFF_CHUNK_SLOTS * sizeof(sniff_slot_t));
        }
    }
    return true;
}
static sniff_config_t active;
static volatile bool enabled = false;
static int64_t last_stats_us = 0;
static volatile uint32_t rx_total, sent, dropped, suppressed;
static volatile uint16_t ids;
static uint16_t seq = 0;

static const char hex_digits[] = "0123456789ABCDEF";

static char *put_hex(char *p, uint32_t value, int digits)
{
    for (int i = digits - 1; i >= 0; i--) {
        p[i] = hex_digits[value & 0xFU];
        value >>= 4;
    }
    return p + digits;
}

static bool emit(const char *line, uint8_t len)
{
    if (sink_fn != NULL && sink_fn(line, len)) {
        sent++;
        seq++;
        return true;
    }
    dropped++;
    return false;
}

static bool emit_frame(const twai_message_t *msg, uint8_t dlc, int64_t now_us)
{
    char line[SNIFF_LINE_MAX];
    char *p = line;

    *p++ = '$';
    p = put_hex(p, seq, 4);
    *p++ = ' ';
    p = put_hex(p, (uint32_t)now_us, 8);
    *p++ = ' ';
    p = msg->extd ? put_hex(p, msg->identifier, 8)
                  : put_hex(p, msg->identifier & 0x7FFU, 3);
    *p++ = ' ';
    *p++ = (char)('0' + dlc);
    if (dlc > 0) {
        *p++ = ' ';
        for (uint8_t i = 0; i < dlc; i++) {
            p = put_hex(p, msg->data[i], 2);
        }
    }
    *p++ = '\r';
    return emit(line, (uint8_t)(p - line));
}

static bool is_pinned(uint32_t identifier)
{
    for (uint8_t i = 0; i < active.npins; i++) {
        if (active.pins[i] == identifier) {
            return true;
        }
    }
    return false;
}

// Open addressing with linear probing. NULL once the table is full.
static sniff_slot_t *slot_for(uint32_t key)
{
    uint32_t h = (key * 2654435761U) >> 23;
    for (uint32_t n = 0; n < SNIFF_TABLE_SIZE; n++) {
        sniff_slot_t *s = slot_at((h + n) & (SNIFF_TABLE_SIZE - 1));
        if (!s->used) {
            s->used = true;
            s->key = key;
            s->dlc = SNIFF_DLC_NONE;
            ids++;
            return s;
        }
        if (s->key == key) {
            return s;
        }
    }
    return NULL;
}

void sniff_rx(const twai_message_t *msg, int64_t now_us)
{
    if (!enabled) {
        return;
    }
    rx_total++;

    uint8_t dlc = msg->data_length_code > 8 ? 8 : msg->data_length_code;
    sniff_slot_t *s = NULL;

    if (!is_pinned(msg->identifier)) {
        if (active.pins_only) {
            suppressed++;
            return;
        }
        s = slot_for(msg->identifier | (msg->extd ? SNIFF_EXT_KEY : 0U));
        // s == NULL: table full, so send unfiltered rather than lose the frame
        if (s != NULL && s->dlc != SNIFF_DLC_NONE) {
            if (s->dlc == dlc && memcmp(s->data, msg->data, dlc) == 0) {
                suppressed++;
                return;
            }
            if (now_us - s->last_sent_us < SNIFF_MIN_INTERVAL_US) {
                suppressed++;
                return;
            }
        }
    }

    // Remember what was last *sent*: a change held back by the rate cap, or a
    // line the sink dropped, still goes out on a later frame.
    if (emit_frame(msg, dlc, now_us) && s != NULL) {
        s->dlc = dlc;
        memcpy(s->data, msg->data, dlc);
        s->last_sent_us = now_us;
    }
}

static void stop(void)
{
    enabled = false;
    free_table();
}

static void apply(const sniff_config_t *cfg, int64_t now_us)
{
    if (!cfg->enable) {
        stop();
        return;
    }
    if (!reset_table()) {
        enabled = false;   // no memory: the status query shows it off
        return;
    }
    active = *cfg;
    if (active.npins > SNIFF_MAX_PINS) {
        active.npins = SNIFF_MAX_PINS;
    }
    rx_total = 0;
    sent = 0;
    dropped = 0;
    suppressed = 0;
    ids = 0;
    seq = 0;
    last_stats_us = now_us;
    enabled = true;
}

void sniff_tick(int64_t now_us, bool link_up)
{
    sniff_config_t cfg;

    while (request_queue != NULL && xQueueReceive(request_queue, &cfg, 0) == pdTRUE) {
        apply(&cfg, now_us);
    }
    if (!enabled) {
        return;
    }
    // A client that reconnects must never find a stream it didn't ask for.
    if (!link_up) {
        stop();
        return;
    }
    if (now_us - last_stats_us >= SNIFF_STATS_INTERVAL_US) {
        last_stats_us = now_us;
        char line[64];
        int len = snprintf(line, sizeof(line), "$%04X ! %lu %lu %lu %lu %u\r",
                           (unsigned)seq,
                           (unsigned long)rx_total, (unsigned long)sent,
                           (unsigned long)dropped, (unsigned long)suppressed,
                           (unsigned)ids);
        if (len > 0 && len < (int)sizeof(line)) {
            emit(line, (uint8_t)len);
        }
    }
}

void sniff_init(sniff_sink_t sink)
{
    sink_fn = sink;
    request_queue = xQueueCreate(1, sizeof(sniff_config_t));
    configASSERT(request_queue != NULL);
}

bool sniff_request(const sniff_config_t *cfg)
{
    if (request_queue == NULL) {
        return false;
    }
    // Depth 1, last request wins: a stop right after a start is a stop.
    return xQueueOverwrite(request_queue, cfg) == pdTRUE;
}

void sniff_get_status(sniff_status_t *out)
{
    out->enabled = enabled;
    out->npins = active.npins;
    out->rx_total = rx_total;
    out->sent = sent;
    out->dropped = dropped;
    out->suppressed = suppressed;
    out->ids = ids;
}

sniff_cmd_t sniff_parse_command(const char *arg, sniff_config_t *out)
{
    memset(out, 0, sizeof(*out));

    if (arg[0] == 0) {
        return SNIFF_CMD_QUERY;
    }
    if (arg[0] == '0' && arg[1] == 0) {
        return SNIFF_CMD_SET;   // enable stays false: stop
    }
    if (arg[0] != '1') {
        return SNIFF_CMD_INVALID;
    }
    out->enable = true;

    const char *p = arg + 1;
    if (*p == 'p') {
        out->pins_only = true;
        p++;
        if (*p == 0) {
            return SNIFF_CMD_INVALID;   // pins-only with no pins streams nothing
        }
    }
    while (*p != 0) {
        if (out->npins == SNIFF_MAX_PINS) {
            return SNIFF_CMD_INVALID;
        }
        uint32_t value = 0;
        int digits = 0;
        while (*p != 0 && *p != ',') {
            char c = *p++;
            int nibble;
            if (c >= '0' && c <= '9') {
                nibble = c - '0';
            } else if (c >= 'a' && c <= 'f') {
                nibble = c - 'a' + 10;
            } else if (c >= 'A' && c <= 'F') {
                nibble = c - 'A' + 10;
            } else {
                return SNIFF_CMD_INVALID;
            }
            if (++digits > 8) {
                return SNIFF_CMD_INVALID;
            }
            value = (value << 4) | (uint32_t)nibble;
        }
        if (digits == 0 || value > 0x1FFFFFFFU) {
            return SNIFF_CMD_INVALID;
        }
        out->pins[out->npins++] = value;
        if (*p == ',') {
            p++;
            if (*p == 0) {
                return SNIFF_CMD_INVALID;
            }
        }
    }
    return SNIFF_CMD_SET;
}
