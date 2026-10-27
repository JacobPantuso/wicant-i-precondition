// Unlock authorisation; see unlockauth.h.

#include "unlockauth.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "mbedtls/md.h"
#include "nvs.h"

#define NVS_NAMESPACE      "unlockauth"
#define NVS_KEY            "key"
#define CHALLENGE_LIFE_US  (10LL * 1000 * 1000)
#define LABEL_MAX          16

static const char *TAG = "unlockauth";

static uint8_t s_key[UNLOCKAUTH_KEY_LEN];
static bool s_has_key;
static uint8_t s_nonce[UNLOCKAUTH_NONCE_LEN];
static int64_t s_nonce_at;
static bool s_nonce_live;

void unlockauth_init(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        ESP_LOGW(TAG, "unlock key not set");
        return;   // nothing stored yet
    }
    size_t len = sizeof(s_key);
    s_has_key = nvs_get_blob(handle, NVS_KEY, s_key, &len) == ESP_OK && len == sizeof(s_key);
    nvs_close(handle);
    ESP_LOGW(TAG, "unlock key %s", s_has_key ? "stored" : "not set");
}

bool unlockauth_has_key(void)
{
    return s_has_key;
}

bool unlockauth_set_key(const uint8_t key[UNLOCKAUTH_KEY_LEN])
{
    if (s_has_key) {
        return false;
    }
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return false;
    }
    esp_err_t err = nvs_set_blob(handle, NVS_KEY, key, UNLOCKAUTH_KEY_LEN);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err != ESP_OK) {
        return false;
    }
    memcpy(s_key, key, UNLOCKAUTH_KEY_LEN);
    s_has_key = true;
    ESP_LOGW(TAG, "unlock key stored");
    return true;
}

void unlockauth_new_challenge(char *hex_out)
{
    // BLE is up whenever this runs, so this is the hardware RNG.
    esp_fill_random(s_nonce, sizeof(s_nonce));
    s_nonce_at = esp_timer_get_time();
    s_nonce_live = true;
    for (int i = 0; i < UNLOCKAUTH_NONCE_LEN; i++) {
        sprintf(&hex_out[i * 2], "%02x", s_nonce[i]);
    }
}

bool unlockauth_check(unlockauth_purpose_t purpose, const uint8_t mac[UNLOCKAUTH_MAC_LEN])
{
    bool live = s_nonce_live && esp_timer_get_time() - s_nonce_at < CHALLENGE_LIFE_US;
    s_nonce_live = false;
    if (!live || !s_has_key) {
        return false;
    }
    const char *label = purpose == UNLOCKAUTH_UNLOCK ? "numen-unlock" : "numen-verify";
    size_t label_len = strlen(label);
    uint8_t message[LABEL_MAX + UNLOCKAUTH_NONCE_LEN];
    memcpy(message, label, label_len);
    memcpy(&message[label_len], s_nonce, UNLOCKAUTH_NONCE_LEN);

    uint8_t expected[UNLOCKAUTH_MAC_LEN];
    if (mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), s_key, UNLOCKAUTH_KEY_LEN,
                        message, label_len + UNLOCKAUTH_NONCE_LEN, expected) != 0) {
        return false;
    }
    // Constant time, so the reply's timing says nothing about the bytes.
    uint8_t diff = 0;
    for (int i = 0; i < UNLOCKAUTH_MAC_LEN; i++) {
        diff |= expected[i] ^ mac[i];
    }
    return diff == 0;
}

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool unlockauth_parse_hex(const char *hex, uint8_t *out, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        int hi = hex_value(hex[i * 2]);
        if (hi < 0) return false;
        int lo = hex_value(hex[i * 2 + 1]);
        if (lo < 0) return false;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return hex[len * 2] == 0;
}
