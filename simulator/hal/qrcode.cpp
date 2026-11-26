#include "qrcode.h"

#include <stdio.h>

#include "qrcodegen.h"

extern "C" void esp_qrcode_print_console(esp_qrcode_handle_t qrcode) {
  const int size = qrcodegen_getSize(qrcode);
  for (int y = -2; y < size + 2; y++) {
    for (int x = -2; x < size + 2; x++) {
      fputs(qrcodegen_getModule(qrcode, x, y) ? "##" : "  ", stdout);
    }
    fputs("\n", stdout);
  }
}

extern "C" esp_err_t esp_qrcode_generate(esp_qrcode_config_t *cfg, const char *text) {
  int maxVersion = cfg->max_qrcode_version;
  if (maxVersion < qrcodegen_VERSION_MIN || maxVersion > qrcodegen_VERSION_MAX) maxVersion = qrcodegen_VERSION_MAX;
  qrcodegen_Ecc ecc;
  switch (cfg->qrcode_ecc_level) {
    case ESP_QRCODE_ECC_MED:
      ecc = qrcodegen_Ecc_MEDIUM;
      break;
    case ESP_QRCODE_ECC_QUART:
      ecc = qrcodegen_Ecc_QUARTILE;
      break;
    case ESP_QRCODE_ECC_HIGH:
      ecc = qrcodegen_Ecc_HIGH;
      break;
    default:
      ecc = qrcodegen_Ecc_LOW;
      break;
  }
  static uint8_t qrcode[qrcodegen_BUFFER_LEN_MAX];
  static uint8_t buffer[qrcodegen_BUFFER_LEN_MAX];
  if (!qrcodegen_encodeText(text, buffer, qrcode, ecc, qrcodegen_VERSION_MIN, maxVersion, qrcodegen_Mask_AUTO, true)) {
    return ESP_FAIL;
  }
  if (cfg->display_func) cfg->display_func(qrcode);
  return ESP_OK;
}

extern "C" int esp_qrcode_get_size(esp_qrcode_handle_t qrcode) { return qrcodegen_getSize(qrcode); }

extern "C" bool esp_qrcode_get_module(esp_qrcode_handle_t qrcode, int x, int y) {
  return qrcodegen_getModule(qrcode, x, y);
}
