// Unlock authorisation for ATXUL (door unlock) on the Numen firmware.
//
// Lock is harmless in the wrong hands; unlock isn't. So ATXUL only goes
// through with proof of a 32-byte key the Numen app wrote once with ATXUK and
// keeps in its Keychain: the WiCAN hands out a single-use challenge, and the
// app answers with HMAC-SHA256(key, label || challenge). A bonded BLE link
// alone, or anything on the Wi-Fi side, can't unlock the car.
//
// The key lives in NVS, so it survives firmware updates. It can't be replaced
// over BLE: a lost key means erasing the WiCAN's nvs partition over USB, which
// also drops its BLE bonds.
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UNLOCKAUTH_KEY_LEN   32
#define UNLOCKAUTH_NONCE_LEN 16
#define UNLOCKAUTH_MAC_LEN   32

typedef enum {
    UNLOCKAUTH_UNLOCK = 0,   // label "numen-unlock": sends the unlock sequence
    UNLOCKAUTH_VERIFY,       // label "numen-verify": proves the key, unlocks nothing
} unlockauth_purpose_t;

// Loads the key from NVS. Once at boot, after nvs_flash_init().
void unlockauth_init(void);

bool unlockauth_has_key(void);

// Stores the key. Refused (false) once a key is stored.
bool unlockauth_set_key(const uint8_t key[UNLOCKAUTH_KEY_LEN]);

// A fresh challenge as lower-case hex into hex_out (2 x UNLOCKAUTH_NONCE_LEN
// + 1 bytes). It replaces any earlier one and is good for one answer within
// 10 s.
void unlockauth_new_challenge(char *hex_out);

// True when mac is HMAC-SHA256(key, label || challenge) for the current
// challenge. The challenge is used up either way.
bool unlockauth_check(unlockauth_purpose_t purpose, const uint8_t mac[UNLOCKAUTH_MAC_LEN]);

// Exactly 2 x len hex digits, then the end of the string.
bool unlockauth_parse_hex(const char *hex, uint8_t *out, size_t len);
