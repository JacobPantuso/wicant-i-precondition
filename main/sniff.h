#ifndef __SNIFF_H__
#define __SNIFF_H__

// CAN sniffer: streams received frames to a BLE client as text lines, filtered
// so a busy bus fits through a BLE link. Enabled and configured by the ELM327
// vendor command ATXSN (see elm327.c). Hardware-independent: the clock and the
// output sink are passed in, so it runs under test/host.

#include <stdbool.h>
#include <stdint.h>
#include "driver/twai.h"

#define SNIFF_MAX_PINS            16
#define SNIFF_TABLE_SIZE          512      // slot_for() hashes to 9 bits
#define SNIFF_MIN_INTERVAL_US     200000   // per-ID cap for unpinned frames (5 Hz)
#define SNIFF_STATS_INTERVAL_US   1000000
#define SNIFF_LINE_MAX            48

// Hands one complete line (ending in '\r') to the transport. Returns false if
// the line was dropped.
typedef bool (*sniff_sink_t)(const char *line, uint8_t len);

typedef struct {
    bool     enable;
    // Stream the pinned identifiers only. The change-only stream of every
    // other ID saturates BLE on a busy bus (30% of lines dropped on a drive,
    // 2026-09-25), and a multi-frame message with one lost frame is useless.
    bool     pins_only;
    uint8_t  npins;
    uint32_t pins[SNIFF_MAX_PINS];   // CAN identifiers (11- or 29-bit)
} sniff_config_t;

typedef struct {
    bool     enabled;
    uint8_t  npins;
    uint32_t rx_total;    // frames seen while enabled
    uint32_t sent;        // lines accepted by the sink (frames + stats)
    uint32_t dropped;     // lines the sink refused
    uint32_t suppressed;  // frames filtered out (unchanged or rate-capped)
    uint16_t ids;         // distinct unpinned identifiers in the table
} sniff_status_t;

typedef enum {
    SNIFF_CMD_INVALID = -1,
    SNIFF_CMD_QUERY = 0,
    SNIFF_CMD_SET = 1,
} sniff_cmd_t;

void sniff_init(sniff_sink_t sink);

// Parses the argument of ATXSN (already lowercased and space-stripped by
// elm327_process_cmd; the text after "xsn"):
//   ""                   query
//   "0"                  stop
//   "1"                  start, no pins
//   "1<hex>,<hex>,..."   start, passing every frame of those identifiers
//   "1p<hex>,..."        the same, and nothing else
sniff_cmd_t sniff_parse_command(const char *arg, sniff_config_t *out);

// Safe to call from any task. Applied on the next sniff_tick().
bool sniff_request(const sniff_config_t *cfg);

// CAN task only.
void sniff_tick(int64_t now_us, bool link_up);
void sniff_rx(const twai_message_t *msg, int64_t now_us);

// Any task. The counters are 32-bit, so a read never tears.
void sniff_get_status(sniff_status_t *out);

#endif
