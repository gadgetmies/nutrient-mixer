// ESP-IDF qrcode component (esp_qrcode_*). Like the original, it encodes with Nayuki's QR code generator.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef const uint8_t *esp_qrcode_handle_t;

#define ESP_QRCODE_ECC_LOW 0
#define ESP_QRCODE_ECC_MED 1
#define ESP_QRCODE_ECC_QUART 2
#define ESP_QRCODE_ECC_HIGH 3

typedef struct {
  void (*display_func)(esp_qrcode_handle_t qrcode);
  int max_qrcode_version;
  int qrcode_ecc_level;
} esp_qrcode_config_t;

void esp_qrcode_print_console(esp_qrcode_handle_t qrcode);

#define ESP_QRCODE_CONFIG_DEFAULT() {esp_qrcode_print_console, 10, ESP_QRCODE_ECC_LOW}

esp_err_t esp_qrcode_generate(esp_qrcode_config_t *cfg, const char *text);
int esp_qrcode_get_size(esp_qrcode_handle_t qrcode);
bool esp_qrcode_get_module(esp_qrcode_handle_t qrcode, int x, int y);

#ifdef __cplusplus
}
#endif
