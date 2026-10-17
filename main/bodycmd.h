// Head-unit body requests on M-CAN 0x4A2 for the myIONIQ app: seat heat and
// ventilation (ATXST) and door lock (ATXLK). One sequencer, because both live
// in the same frame and a burst of one must never interleave with the other.
// See bodycmd.c for the frame layouts.
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BODYCMD_CMD_INVALID = 0,
    BODYCMD_CMD_QUERY,
    BODYCMD_CMD_SET,
} bodycmd_cmd_t;

typedef struct {
    uint32_t seat_requests;   // ATXST set commands accepted
    uint32_t lock_requests;   // ATXLK1 commands accepted
    uint32_t frames;          // 0x4A2 frames handed to the bus
    bool busy;                // a burst is going out
} bodycmd_status_t;

// Parses what follows "ATXST" (lower-cased, spaces stripped): nothing is a
// query; four hex digits are D5 D6 (driver, passenger, rear left, rear right).
bodycmd_cmd_t bodycmd_parse_seat(const char *args, uint16_t *levels);

// Parses what follows "ATXLK": nothing is a query, "1" is lock. There is no
// unlock.
bodycmd_cmd_t bodycmd_parse_lock(const char *args);

// Queues a seat burst. A request that arrives while another is waiting merges
// into it: every seat that isn't F replaces the waiting one's value.
void bodycmd_request_seats(uint16_t levels);

// Queues the lock sequence. Goes ahead of a waiting seat burst.
void bodycmd_request_lock(void);

// Every 40 ms precondition tick, on the CAN task.
void bodycmd_tick(bool link_up);

void bodycmd_get_status(bodycmd_status_t *out);
