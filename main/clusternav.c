#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "clusternav.h"

// Maneuver frames are rewritten from the bytes and masks the app sends
// (layouts in the app's ClusterNavFeed.swift). 0x6E7 text is originated here
// (see clusternav_poll).

#define CLUSTERNAV_QUEUE_LEN 8

typedef enum {
    REQ_STATE,
    REQ_CLEAR,
    REQ_TEXT,
} request_kind_t;

// every request shares one FIFO, so a text can never overtake the clear
// that was sent before it
typedef struct {
    uint8_t kind;
    union {
        clusternav_state_t state;
        clusternav_text_t text;
    };
} clusternav_request_t;

static QueueHandle_t request_queue = NULL;
static clusternav_send_t send_fn = NULL;
// CAN task only
static clusternav_state_t current;
static bool has_state = false;
static int64_t updated_us = 0;
// read by clusternav_get_status from any task
static volatile bool live = false;
static volatile uint32_t updates, injected, texts_sent, text_frames, text_resends;

// ---- text slots (CAN task only) ----

typedef struct {
    uint16_t id;
    uint8_t sid;
    uint8_t blank_len;
} text_slot_t;

// The messages the built-in nav sends, and the form it blanks each with.
static const text_slot_t text_slots[CLUSTERNAV_TEXT_SLOTS] = {
    {0x6E7U, 0xF0U, 3}, {0x6E7U, 0xF2U, 3}, {0x6E7U, 0xF4U, 3},
    {0x6DFU, 0xF0U, 3}, {0x6DFU, 0xF2U, 3}, {0x6DFU, 0xF4U, 3},
    {0x680U, 0xF1U, 81}, {0x681U, 0xF1U, 81},
};
// what the app asked for; len 0 = nothing
static clusternav_text_t wanted[CLUSTERNAV_TEXT_SLOTS];
// fingerprint of the last payload fully sent -- a full copy of each would
// cost ~800 bytes of a heap with little to spare. Starts as the blank form's,
// since the cluster boots blank.
static uint32_t shown[CLUSTERNAV_TEXT_SLOTS];

static struct {
    bool active;
    uint8_t slot;
    uint8_t offset;
    uint8_t seq;
    clusternav_text_t msg;
} tx;
static int64_t last_frame_us = -CLUSTERNAV_TEXT_GAP_US;

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

static bool is_live(int64_t now_us)
{
    return has_state && (now_us - updated_us) < CLUSTERNAV_STALE_US;
}

static int slot_for(uint16_t id, uint8_t sid)
{
    for (int i = 0; i < CLUSTERNAV_TEXT_SLOTS; i++) {
        if (text_slots[i].id == id && text_slots[i].sid == sid) {
            return i;
        }
    }
    return -1;
}

static void blank_form(clusternav_text_t *t, int slot)
{
    t->id = text_slots[slot].id;
    t->len = text_slots[slot].blank_len;
    memset(t->data, 0, t->len);
    t->data[0] = text_slots[slot].sid;
}

// FNV-1a over the length and the bytes
static uint32_t fingerprint(const clusternav_text_t *t)
{
    uint32_t h = 2166136261U;
    h = (h ^ t->len) * 16777619U;
    for (uint8_t i = 0; i < t->len; i++) {
        h = (h ^ t->data[i]) * 16777619U;
    }
    return h;
}

static void forget_texts(void)
{
    for (int i = 0; i < CLUSTERNAV_TEXT_SLOTS; i++) {
        wanted[i].len = 0;
    }
}

void clusternav_init(clusternav_send_t send)
{
    send_fn = send;
    if (request_queue == NULL) {
        request_queue = xQueueCreate(CLUSTERNAV_QUEUE_LEN, sizeof(clusternav_request_t));
    }
    for (int i = 0; i < CLUSTERNAV_TEXT_SLOTS; i++) {
        clusternav_text_t blank;
        wanted[i].len = 0;
        blank_form(&blank, i);
        shown[i] = fingerprint(&blank);
    }
    tx.active = false;
}

// Only the cluster's nav frames. The hook sees every frame bound for the
// cluster side, powertrain included; a bad payload from the app must not be
// able to rewrite one of those.
static bool frame_allowed(uint16_t id)
{
    switch (id) {
    case 0x4CCU:
    case 0x63EU:
    case 0x640U:
    case 0x641U:
    case 0x4E8U:
    case 0x4EAU:   // d6 goes 00 -> 10 when the built-in nav starts guiding
        return true;
    default:
        return false;
    }
}

static int hex_byte(const char *p)
{
    int hi = hex_nibble(p[0]);
    if (hi < 0) {
        return -1;
    }
    int lo = hex_nibble(p[1]);
    if (lo < 0) {
        return -1;
    }
    return (hi << 4) | lo;
}

clusternav_cmd_t clusternav_parse_command(const char *arg, clusternav_state_t *out)
{
    if (arg[0] == '\0') {
        return CLUSTERNAV_CMD_QUERY;
    }
    if (arg[0] == '0' && arg[1] == '\0') {
        return CLUSTERNAV_CMD_CLEAR;
    }
    if (arg[0] != '1') {
        return CLUSTERNAV_CMD_INVALID;
    }

    const char *hex = &arg[1];
    const size_t per_frame = 2 * CLUSTERNAV_FRAME_BYTES;
    const size_t len = strlen(hex);
    if (len == 0 || len % per_frame != 0 || len / per_frame > CLUSTERNAV_MAX_FRAMES) {
        return CLUSTERNAV_CMD_INVALID;
    }

    const uint8_t count = (uint8_t)(len / per_frame);
    for (uint8_t i = 0; i < count; i++) {
        const char *p = &hex[i * per_frame];
        uint8_t b[CLUSTERNAV_FRAME_BYTES];
        for (int k = 0; k < CLUSTERNAV_FRAME_BYTES; k++) {
            int v = hex_byte(&p[2 * k]);
            if (v < 0) {
                return CLUSTERNAV_CMD_INVALID;
            }
            b[k] = (uint8_t)v;
        }
        uint16_t id = (uint16_t)((b[0] << 8) | b[1]);
        if (!frame_allowed(id)) {
            return CLUSTERNAV_CMD_INVALID;
        }
        out->frames[i].id = id;
        out->frames[i].mask = b[2];
        memcpy(out->frames[i].data, &b[3], 8);
    }
    out->count = count;
    return CLUSTERNAV_CMD_SET;
}

clusternav_cmd_t clusternav_parse_text(const char *arg, clusternav_text_t *out)
{
    if (arg[0] == '\0') {
        return CLUSTERNAV_CMD_QUERY;
    }
    // hex_byte stops at a terminator, so a short argument never reads past it
    int id_hi = hex_byte(&arg[0]);
    if (id_hi < 0) {
        return CLUSTERNAV_CMD_INVALID;
    }
    int id_lo = hex_byte(&arg[2]);
    if (id_lo < 0) {
        return CLUSTERNAV_CMD_INVALID;
    }
    out->id = (uint16_t)((id_hi << 8) | id_lo);

    const char *hex = &arg[4];
    uint8_t n = 0;
    while (hex[2 * n] != '\0') {
        if (n >= CLUSTERNAV_TEXT_MAX) {
            return CLUSTERNAV_CMD_INVALID;
        }
        // an odd length ends on the terminator here, which isn't hex
        int v = hex_byte(&hex[2 * n]);
        if (v < 0) {
            return CLUSTERNAV_CMD_INVALID;
        }
        out->data[n++] = (uint8_t)v;
    }
    if (n < 3 || slot_for(out->id, out->data[0]) < 0) {
        return CLUSTERNAV_CMD_INVALID;
    }
    out->len = n;
    return CLUSTERNAV_CMD_SET;
}

static bool enqueue(const clusternav_request_t *req)
{
    if (request_queue == NULL) {
        return false;
    }
    return xQueueSend(request_queue, req, 0) == pdTRUE;
}

bool clusternav_request(const clusternav_state_t *state)
{
    clusternav_request_t req;
    memset(&req, 0, sizeof(req));
    req.kind = (state != NULL) ? REQ_STATE : REQ_CLEAR;
    if (state != NULL) {
        req.state = *state;
    }
    return enqueue(&req);
}

bool clusternav_request_text(const clusternav_text_t *text)
{
    clusternav_request_t req;
    memset(&req, 0, sizeof(req));
    req.kind = REQ_TEXT;
    req.text = *text;
    return enqueue(&req);
}

void clusternav_tick(int64_t now_us, bool link_up)
{
    clusternav_request_t req;
    while (request_queue != NULL && xQueueReceive(request_queue, &req, 0) == pdTRUE) {
        switch (req.kind) {
        case REQ_STATE:
            current = req.state;
            has_state = true;
            updated_us = now_us;
            updates++;
            break;
        case REQ_CLEAR:
            // the end of a trip: text from it must not come back next time
            has_state = false;
            forget_texts();
            break;
        case REQ_TEXT: {
            int slot = slot_for(req.text.id, req.text.data[0]);
            if (slot >= 0) {
                wanted[slot] = req.text;
            }
            break;
        }
        default:
            break;
        }
    }
    // the app is the only source of truth; without it, stop immediately
    // rather than waiting out the staleness window
    if (!link_up) {
        has_state = false;
        forget_texts();
    }
    live = is_live(now_us);
}

// What slot i should be showing: the app's message while live, blank
// otherwise. A lapse keeps `wanted`, so it returns with the next ATXNV1.
static void target_text(int i, bool now_live, clusternav_text_t *out)
{
    if (now_live && wanted[i].len > 0U) {
        *out = wanted[i];
    } else {
        blank_form(out, i);
    }
}

void clusternav_poll(int64_t now_us)
{
    if (send_fn == NULL || (now_us - last_frame_us) < CLUSTERNAV_TEXT_GAP_US) {
        return;
    }
    if (!tx.active) {
        const bool now_live = is_live(now_us);
        for (int i = 0; i < CLUSTERNAV_TEXT_SLOTS; i++) {
            clusternav_text_t want;
            target_text(i, now_live, &want);
            if (fingerprint(&want) != shown[i]) {
                tx.active = true;
                tx.slot = (uint8_t)i;
                tx.offset = 0;
                tx.seq = 0;
                tx.msg = want;
                break;
            }
        }
        if (!tx.active) {
            return;
        }
    }

    twai_message_t m;
    memset(&m, 0, sizeof(m));
    m.identifier = tx.msg.id;
    m.data_length_code = 8;
    memset(m.data, 0xAA, 8);   // the head unit's padding byte

    uint8_t n;
    if (tx.offset == 0U && tx.msg.len <= 7U) {
        // single frame
        m.data[0] = tx.msg.len;
        memcpy(&m.data[1], tx.msg.data, tx.msg.len);
        n = tx.msg.len;
    } else if (tx.offset == 0U) {
        // first frame; CLUSTERNAV_TEXT_MAX keeps the length to one byte
        m.data[0] = 0x10U;
        m.data[1] = tx.msg.len;
        memcpy(&m.data[2], tx.msg.data, 6);
        n = 6;
    } else {
        // consecutive frame
        uint8_t left = (uint8_t)(tx.msg.len - tx.offset);
        n = left < 7U ? left : 7U;
        m.data[0] = (uint8_t)(0x20U | (tx.seq & 0x0FU));
        memcpy(&m.data[1], &tx.msg.data[tx.offset], n);
    }

    if (!send_fn(&m)) {
        return;   // TX pool full: the same frame goes out next poll
    }
    last_frame_us = now_us;
    text_frames++;
    tx.offset = (uint8_t)(tx.offset + n);
    tx.seq++;
    if (tx.offset >= tx.msg.len) {
        shown[tx.slot] = fingerprint(&tx.msg);
        tx.active = false;
        texts_sent++;
    }
}

// ISO-TP from the head unit on one of the text slots. Under CarPlay it still
// draws its own lane graphic near an interchange, once, then blanks it once
// past. `shown` only tracks what this code sent (TWAI never receives its own
// frames), so it went on believing the app's lanes were up and the HUD stayed
// blank until they changed. Its lanes are left alone; the app's go back the
// moment it blanks them. The first frame says which: text blanks as a single
// frame "SID 00 00", F1 as a first frame with a lane count of 0.
static void note_foreign_text(const twai_message_t *msg)
{
    const uint8_t pci = msg->data[0] >> 4;
    uint8_t sid;
    bool blank;
    if (pci == 0U) {
        sid = msg->data[1];
        blank = (msg->data[0] & 0x0FU) == 3U && msg->data[2] == 0U && msg->data[3] == 0U;
    } else if (pci == 1U) {
        sid = msg->data[2];
        blank = (sid == 0xF1U) && msg->data[3] == 0U;
    } else {
        return;
    }
    const int slot = slot_for((uint16_t)msg->identifier, sid);
    if (slot < 0 || !blank || wanted[slot].len == 0U) {
        return;
    }
    if (tx.active && tx.slot == (uint8_t)slot) {
        return;   // ours is already on its way out
    }
    // anything but the wanted payload's own fingerprint makes poll resend it
    shown[slot] = ~fingerprint(&wanted[slot]);
    text_resends++;
}

fwd_result_t clusternav_fwd(twai_message_t *to_send, int64_t now_us)
{
    if (!is_live(now_us) || to_send->extd || to_send->data_length_code < 8U) {
        return FWD_PASSTHROUGH;
    }
    note_foreign_text(to_send);
    for (uint8_t i = 0; i < current.count; i++) {
        const clusternav_frame_t *f = &current.frames[i];
        if (f->id != to_send->identifier || f->mask == 0U) {
            continue;
        }
        for (int b = 0; b < 8; b++) {
            if (f->mask & (1U << b)) {
                to_send->data[b] = f->data[b];
            }
        }
        injected++;
        return FWD_MODIFIED;
    }
    return FWD_PASSTHROUGH;
}

void clusternav_get_status(clusternav_status_t *out)
{
    out->active = live;
    out->updates = updates;
    out->injected = injected;
    out->texts_sent = texts_sent;
    out->text_frames = text_frames;
    out->text_resends = text_resends;
}
