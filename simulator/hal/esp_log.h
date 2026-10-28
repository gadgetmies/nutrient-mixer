// ESP-IDF and arduino-esp32 log macros. Core debug output is off in the default Arduino build, so they print nothing.
#pragma once
#define ESP_LOGE(tag, ...) ((void)(tag))
#define ESP_LOGW(tag, ...) ((void)(tag))
#define ESP_LOGI(tag, ...) ((void)(tag))
#define ESP_LOGD(tag, ...) ((void)(tag))
#define ESP_LOGV(tag, ...) ((void)(tag))
#define log_e(...)
#define log_w(...)
#define log_i(...)
#define log_d(...)
#define log_v(...)
