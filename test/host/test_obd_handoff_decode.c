#include <stdbool.h>
#include <string.h>
#include "test_support.h"
#include "obd_handoff_decode.c"

static void formatted(const uint8_t *payload, size_t size, char *out, size_t cap)
{
    size_t used = (size_t)snprintf(out, cap, "%03X\r", (unsigned)size);
    size_t offset = 0;
    for (unsigned index = 0; offset < size; index++) {
        used += (size_t)snprintf(out + used, cap - used, "%X:", index);
        size_t end = offset + (index == 0 ? 6 : 7);
        if (end > size) end = size;
        while (offset < end) used += (size_t)snprintf(out + used, cap - used, "%02X", payload[offset++]);
        used += (size_t)snprintf(out + used, cap - used, "\r");
    }
    snprintf(out + used, cap - used, ">");
}

static void raw_iso(const uint8_t *payload, size_t size, char *out, size_t cap)
{
    size_t used = (size_t)snprintf(out, cap, "1%03X", (unsigned)size);
    size_t offset = 0;
    unsigned sequence = 1;
    while (offset < size) {
        size_t end = offset + (offset == 0 ? 6 : 7);
        if (offset) used += (size_t)snprintf(out + used, cap - used, "2%X", sequence++ & 15);
        if (end > size) end = size;
        while (offset < end) used += (size_t)snprintf(out + used, cap - used, "%02X", payload[offset++]);
        used += (size_t)snprintf(out + used, cap - used, "\r");
    }
    snprintf(out + used, cap - used, ">");
}

int main(void)
{
    uint8_t out[128];
    // Both common ELM output styles produce exactly the DID payload, with
    // sequence/length checked before any safety bit is consulted.
    const char *caf = "009\r0:62BC03000000\r1:000000\r>";
    const char *iso = "100962BC03000000\r21000000AAAAAAAA\r>";
    CHECK(obd_handoff_bytes(caf, out, sizeof(out)) == 9);
    CHECK(out[0] == 0x62 && out[1] == 0xbc && out[2] == 3);
    CHECK(obd_handoff_bytes(iso, out, sizeof(out)) == 9);
    CHECK(obd_handoff_bytes("07 62 BC 04 00 00 00 00\r>", out, sizeof(out)) == 7);
    CHECK(obd_handoff_bytes("62BC03000000000000\r>", out, sizeof(out)) == 9);
    const char *bad[] = {
        "009\r0:62BC03000000\r2:000000\r>", // missing frame
        "009\r0:62BC03000000\r0:000000\r>", // duplicate frame
        "009\r1:000000\r0:62BC03000000\r>", // reordered
        "00A\r0:62BC03000000\r1:000000\r>", // declared length not met
        "009\r0:62BC03000000\r>",
        "009\r0:62BC03000000\r1:00000Z\r>",
        "009\r0:62BC03000000\r1:00000\r>",
        "0:62BC03000000\r1:000000\r>", // missing length
        "009\r009\r0:62BC03000000\r1:000000\r>",
        "009\r0:62BC030000\r1:00000000\r>", // malformed first segment
        "100962BC03000000\r22000000AAAAAAAA\r>", // raw sequence gap
        "100962BC03000000\r>",
        "100962BC03000000\r210000\r>",
        "100962BC03000000\r21000000AAAAAAAA\r22000000AAAAAAAA\r>",
        "62BC03000000000000\r62BC03000000000000\r>", // multiple responders
        "62BC03000000000000>62BC03000000000000",
        "NO DATA\r>", "CAN ERROR\r>", "SEARCHING...\r>",
        "7F2231\r>", "009\r62BC03000000000000\r>",
        "0662BC03\r>", "000\r>", "FFF\r>", "\r>", ""
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        CHECK_MSG(obd_handoff_bytes(bad[i], out, sizeof(out)) == 0, "case %zu", i);
    }
    CHECK(obd_handoff_bytes(caf, out, 8) == 0);
    CHECK(obd_handoff_bytes(NULL, out, sizeof(out)) == 0);

    // Exercise exact buffer limits and consecutive-frame sequence rollover.
    uint8_t payload[128];
    char long_wire[768];
    for (size_t i = 0; i < sizeof(payload); i++) payload[i] = (uint8_t)i;
    for (size_t size = 8; size <= sizeof(payload); size++) {
        formatted(payload, size, long_wire, sizeof(long_wire));
        CHECK(obd_handoff_bytes(long_wire, out, sizeof(out)) == size);
        CHECK(memcmp(out, payload, size) == 0);
        raw_iso(payload, size, long_wire, sizeof(long_wire));
        CHECK(obd_handoff_bytes(long_wire, out, sizeof(out)) == size);
        CHECK(memcmp(out, payload, size) == 0);
        CHECK(obd_handoff_bytes(long_wire, out, size - 1) == 0);
    }

    // All existing blocking masks, plus every front/rear lock bit. Setting
    // an unrelated bit must not fabricate an open door or an unlocked front.
    uint8_t body[9] = {0x62, 0xbc, 3, 0, 0, 0, 0, 0, 0};
    char wire[256];
    bool on = true, blocked = true, rear = false, front = false;
    formatted(body, sizeof(body), wire, sizeof(wire));
    CHECK(obd_handoff_safety(wire, &on, &blocked, &rear));
    CHECK(!on && !blocked && rear);
    for (unsigned bit = 0; bit < 8; bit++) {
        body[7] = (uint8_t)(1U << bit);
        formatted(body, sizeof(body), wire, sizeof(wire));
        CHECK(obd_handoff_safety(wire, &on, &blocked, &rear));
        CHECK(blocked == ((body[7] & 0xb5) != 0));
        CHECK(rear == ((body[7] & 0x0a) == 0));
        CHECK(!on);
    }
    body[7] = 0;
    for (unsigned bit = 0; bit < 8; bit++) {
        body[8] = (uint8_t)(1U << bit);
        formatted(body, sizeof(body), wire, sizeof(wire));
        CHECK(obd_handoff_safety(wire, &on, &blocked, &rear));
        CHECK(on == ((body[8] & 0x60) != 0));
        CHECK(blocked == ((body[8] & 0x67) != 0));
    }
    body[2] = 4;
    for (unsigned bit = 0; bit < 8; bit++) {
        body[7] = (uint8_t)(1U << bit);
        formatted(body, sizeof(body), wire, sizeof(wire));
        CHECK(obd_handoff_front(wire, &front));
        CHECK(front == ((body[7] & 0x0c) == 0));
        CHECK(!obd_handoff_safety(wire, &on, &blocked, &rear));
    }
    on = true; blocked = true; rear = false;
    CHECK(!obd_handoff_safety("62BC03000000\r>", &on, &blocked, &rear));
    CHECK(on && blocked && !rear); // Invalid input leaves caller's hold intact.
    CHECK(!obd_handoff_front(caf, &front));

    const char *vin = "1HGCM82633A004352";
    uint8_t identity[20] = {0x49, 2, 1};
    memcpy(identity + 3, vin, 17);
    formatted(identity, sizeof(identity), wire, sizeof(wire));
    CHECK(obd_handoff_vin(wire, vin));
    CHECK(!obd_handoff_vin(wire, "1HGCM82633A004353"));
    CHECK(!obd_handoff_vin(wire, "1HGCM82633A00435"));
    identity[2] = 2;
    formatted(identity, sizeof(identity), wire, sizeof(wire));
    CHECK(!obd_handoff_vin(wire, vin));
    identity[2] = 1;
    formatted(identity, sizeof(identity) - 1, wire, sizeof(wire));
    CHECK(!obd_handoff_vin(wire, vin));

    return test_report("obd_handoff_decode");
}
