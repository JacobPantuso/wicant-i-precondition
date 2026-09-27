#ifndef __CLUSTERNAV_H__
#define __CLUSTERNAV_H__

// Cluster turn-by-turn injection: the app streams the current CarPlay
// maneuver over BLE (ATXNV, see elm327.c) and this rewrites the head unit's
// own nav frames on their way to the cluster. Hardware-independent, like
// sniff.c: the clock and the CAN sink are passed in, so it runs under test/host.
//
// The firmware knows no frame layouts. The app sends each frame's bytes and a
// mask of which ones to overwrite, and the encoding lives in the app
// (ClusterNavFeed.swift) -- so a decoding discovery is an app update, not a
// reflash. The first layouts, taken from the IONIQ 5 logs, were wrong for the
// IONIQ 6 in ways only a reflash could fix.
//
// Multi-frame messages (ATXNT) are different: the head unit sends none of
// them under CarPlay, so there is nothing to rewrite and this originates the
// ISO-TP messages itself (clusternav_poll). The built-in nav sends its text
// twice -- 0x6E7 for the cluster, 0x6DF for the HUD -- plus an 81-byte F1
// message on both 0x680 and 0x681.

#include <stdbool.h>
#include <stdint.h>
#include "driver/twai.h"
#include "hsm.h"

// injection stops this long after the last ATXNV1, so a dropped BLE link or
// a killed app can't leave a frozen arrow on the cluster
#define CLUSTERNAV_STALE_US     4000000
// frames per ATXNV1; 5 x 22 hex digits still fits ELM327's 126-char command cap
#define CLUSTERNAV_MAX_FRAMES   5
#define CLUSTERNAV_FRAME_BYTES  11       // id (2), mask, d[0..7]

// ATXNT messages: one ISO-TP message per (id, SID), no flow control (the head
// unit never waits for one either), consecutive frames paced like its own
#define CLUSTERNAV_TEXT_MAX     96       // bytes; F1 needs 81. Hex must fit ELM327's 254-char command cap
#define CLUSTERNAV_TEXT_GAP_US  5000
#define CLUSTERNAV_TEXT_SLOTS   8        // 6E7 and 6DF: F0, F2, F4; 680 and 681: F1

// One frame to rewrite: every d[i] whose mask bit i is set is replaced; the
// others keep the head unit's value.
typedef struct {
    uint16_t id;
    uint8_t  mask;
    uint8_t  data[8];
} clusternav_frame_t;

// One ATXNV1 payload: 1..CLUSTERNAV_MAX_FRAMES frames, each on the wire as
// 11 bytes = 22 hex digits: id (big-endian), mask, d[0..7]. Only the
// cluster's nav frames (4CC, 63E, 640, 641, 4E8, 4EA) are accepted.
typedef struct {
    uint8_t count;
    clusternav_frame_t frames[CLUSTERNAV_MAX_FRAMES];
} clusternav_state_t;

// One complete ISO-TP payload for one CAN id, SID first. Layouts (IONIQ 6
// captures), identical on 0x6E7 and 0x6DF:
//   F0 <UTF-16LE street>
//   F2 <lane count> <3 lane codes, 00-padded> <UTF-16LE street> 00 00 00 00
//   F4 00 <UTF-16LE destination>
// and SID 00 00 blanks a field. 0x680/0x681 carry F1 in 81 bytes, blanked
// as F1 followed by 80 zeros.
typedef struct {
    uint16_t id;
    uint8_t len;
    uint8_t data[CLUSTERNAV_TEXT_MAX];
} clusternav_text_t;

typedef struct {
    bool     active;       // injecting right now (fresh state, link up)
    uint32_t updates;      // ATXNV1 payloads applied
    uint32_t injected;     // frames modified
    uint32_t texts_sent;   // 0x6E7 messages completed
    uint32_t text_frames;  // 0x6E7 frames sent
    uint32_t text_resends; // messages re-sent after the head unit blanked its own
} clusternav_status_t;

typedef enum {
    CLUSTERNAV_CMD_INVALID = -1,
    CLUSTERNAV_CMD_QUERY = 0,
    CLUSTERNAV_CMD_SET = 1,
    CLUSTERNAV_CMD_CLEAR = 2,
} clusternav_cmd_t;

// Transmits one frame on the cluster side. Must not block; false means "try
// again next poll".
typedef bool (*clusternav_send_t)(const twai_message_t *msg);

void clusternav_init(clusternav_send_t send);

// Parses the argument of ATXNV (already lowercased and space-stripped by
// elm327_process_cmd; the text after "xnv"):
//   ""              query
//   "0"             clear (also forgets every text field)
//   "1<22 hex/frame>" set (layout above)
clusternav_cmd_t clusternav_parse_command(const char *arg, clusternav_state_t *out);

// Parses the argument of ATXNT: "" query, or <id: 4 hex><payload as hex>, the
// payload 3..CLUSTERNAV_TEXT_MAX bytes and its (id, SID) one of the slots.
clusternav_cmd_t clusternav_parse_text(const char *arg, clusternav_text_t *out);

// Safe to call from any task; NULL clears. Applied on the next clusternav_tick().
bool clusternav_request(const clusternav_state_t *state);
bool clusternav_request_text(const clusternav_text_t *text);

// CAN task only. tick applies requests (40 ms cadence is fine); poll sends
// 0x6E7 frames and wants calling on every pass of the CAN loop.
void clusternav_tick(int64_t now_us, bool link_up);
void clusternav_poll(int64_t now_us);
fwd_result_t clusternav_fwd(twai_message_t *to_send, int64_t now_us);

// Any task. The counters are 32-bit, so a read never tears.
void clusternav_get_status(clusternav_status_t *out);

#endif
