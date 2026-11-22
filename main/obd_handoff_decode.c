#include "obd_handoff_decode.h"
#include <ctype.h>
#include <string.h>

static int hex(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    c = (char)toupper((unsigned char)c);
    return c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
}

static size_t bytes(const char *s, size_t length, uint8_t *out, size_t capacity)
{
    if (!length || length % 2 || length / 2 > capacity) return 0;
    for (size_t i = 0; i < length / 2; i++) {
        int hi = hex(s[2 * i]), lo = hex(s[2 * i + 1]);
        if (hi < 0 || lo < 0) return 0;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return length / 2;
}

size_t obd_handoff_bytes(const char *raw, uint8_t *out, size_t capacity)
{
    if (!raw || !out || !capacity) return 0;
    enum { NONE, NUMBERED, ISO, PLAIN } format = NONE;
    size_t count = 0, declared = 0, segment = 0;
    unsigned next_sequence = 1;
    bool complete = false;
    while (*raw) {
        char line[260];
        size_t length = 0;
        while (*raw && *raw != '\r' && *raw != '\n' && *raw != '>') {
            char c = *raw++;
            if (c == ' ' || c == '\t') continue;
            if (length >= sizeof(line) - 1 || (unsigned char)c < 32) return 0;
            line[length++] = c;
        }
        line[length] = 0;
        bool prompt = *raw == '>';
        if (*raw) raw++;
        if (length) {
            if (complete) return 0; // Extra frames / a second ECU are ambiguous.
            char *colon = strchr(line, ':');
            if (colon) {
                size_t prefix = (size_t)(colon - line);
                if (!declared || (format != NONE && format != NUMBERED)
                        || prefix < 1 || prefix > 2) return 0;
                unsigned index = 0;
                for (size_t i = 0; i < prefix; i++) {
                    int digit = hex(line[i]);
                    if (digit < 0) return 0;
                    index = index * 16 + (unsigned)digit;
                }
                if (index != segment++) return 0; // Missing, duplicate, reordered.
                uint8_t part[7];
                size_t n = bytes(colon + 1, length - prefix - 1, part, sizeof(part));
                size_t wanted = segment == 1 ? 6 : 7;
                size_t remaining = declared - count;
                if (!n || n > wanted || (remaining > wanted && n != wanted)
                        || n < (remaining < wanted ? remaining : wanted)) return 0;
                size_t take = n < remaining ? n : remaining;
                memcpy(out + count, part, take);
                count += take;
                format = NUMBERED;
                complete = count == declared;
            } else if (length == 3) {
                if (format != NONE || declared) return 0;
                for (size_t i = 0; i < length; i++) {
                    int digit = hex(line[i]);
                    if (digit < 0) return 0;
                    declared = declared * 16 + (size_t)digit;
                }
                if (!declared || declared > capacity) return 0;
            } else {
                if (format == NUMBERED || (declared && format != ISO)) return 0;
                uint8_t part[128];
                size_t n = bytes(line, length, part, sizeof(part));
                if (!n) return 0;
                if (format == NONE && (part[0] == 0x62 || part[0] == 0x49)) {
                    if (n > capacity) return 0;
                    memcpy(out, part, n);
                    count = n;
                    format = PLAIN;
                    complete = true;
                } else if (format == NONE && (part[0] & 0xf0) == 0) {
                    size_t payload = part[0] & 0x0f;
                    if (!payload || payload > 7 || n > 8 || n < payload + 1
                            || payload > capacity) return 0;
                    memcpy(out, part + 1, payload);
                    count = payload;
                    format = ISO;
                    complete = true;
                } else if (format == NONE && (part[0] & 0xf0) == 0x10) {
                    if (n != 8) return 0;
                    declared = ((size_t)(part[0] & 0x0f) << 8) | part[1];
                    if (declared <= 7 || declared > capacity) return 0;
                    memcpy(out, part + 2, 6);
                    count = 6;
                    format = ISO;
                } else if (format == ISO && (part[0] & 0xf0) == 0x20) {
                    if (n < 2 || n > 8 || (part[0] & 0x0f) != next_sequence) return 0;
                    next_sequence = (next_sequence + 1) & 0x0f;
                    size_t remaining = declared - count;
                    if (n - 1 < (remaining < 7 ? remaining : 7)) return 0;
                    size_t take = remaining < n - 1 ? remaining : n - 1;
                    memcpy(out + count, part + 1, take);
                    count += take;
                    complete = count == declared;
                } else return 0;
            }
        }
        if (prompt) {
            while (*raw) { if (!isspace((unsigned char)*raw++)) return 0; }
        }
    }
    return complete ? count : 0;
}

bool obd_handoff_vin(const char *raw, const char *expected)
{
    if (!expected || strlen(expected) != 17) return false;
    for (size_t i = 0; i < 17; i++) {
        char c = expected[i];
        if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z'))
                || c == 'I' || c == 'O' || c == 'Q') return false;
    }
    uint8_t b[96];
    size_t n = obd_handoff_bytes(raw, b, sizeof(b));
    return n == 20 && b[0] == 0x49 && b[1] == 2 && b[2] == 1
        && memcmp(b + 3, expected, 17) == 0;
}

bool obd_handoff_safety(const char *raw, bool *on, bool *blocked, bool *rear_locked)
{
    if (!on || !blocked || !rear_locked) return false;
    uint8_t b[128];
    size_t n = obd_handoff_bytes(raw, b, sizeof(b));
    if (n < 9 || b[0] != 0x62 || b[1] != 0xbc || b[2] != 3) return false;
    uint8_t d = b[7], s = b[8];
    *on = (s & 0x60) != 0;
    *blocked = *on || (d & 0xb5) != 0 || (s & 7) != 0;
    *rear_locked = (d & 0x0a) == 0;
    return true;
}

bool obd_handoff_opening(const char *raw, bool *open)
{
    if (!open) return false;
    uint8_t b[128];
    size_t n = obd_handoff_bytes(raw, b, sizeof(b));
    if (n < 9 || b[0] != 0x62 || b[1] != 0xbc || b[2] != 3) return false;
    *open = (b[7] & 0xb5) != 0;
    return true;
}

bool obd_handoff_front(const char *raw, bool *locked)
{
    if (!locked) return false;
    uint8_t b[128];
    size_t n = obd_handoff_bytes(raw, b, sizeof(b));
    if (n < 8 || b[0] != 0x62 || b[1] != 0xbc || b[2] != 4) return false;
    *locked = (b[7] & 0x0c) == 0;
    return true;
}
