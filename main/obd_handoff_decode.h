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
