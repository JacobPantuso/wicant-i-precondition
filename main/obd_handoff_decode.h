#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
// ELM with headers/spaces off: accepts numbered multiline CAF replies and
// rejects odd/malformed hex rather than manufacturing a clear observation.
size_t obd_handoff_bytes(const char *raw, uint8_t *out, size_t capacity);
bool obd_handoff_vin(const char *raw, const char *expected);
bool obd_handoff_safety(const char *raw, bool *on, bool *blocked, bool *rear_locked);
bool obd_handoff_front(const char *raw, bool *locked);
// 22BC03: a door, the hood or the trunk is open (the 0xb5 mask of byte 7).
// Belts and ignition are deliberately not included: an unbuckled driver
// still sitting in the car must not read as having left it.
bool obd_handoff_opening(const char *raw, bool *open);
