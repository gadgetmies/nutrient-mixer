// Flash is memory mapped on the ESP32, so program memory reads are plain reads
#pragma once
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#define PROGMEM
#define PGM_P const char *
#define PGM_VOID_P const void *
#define PSTR(s) (s)
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#define pgm_read_dword(addr) (*(const uint32_t *)(addr))
#define pgm_read_float(addr) (*(const float *)(addr))
#define pgm_read_ptr(addr) (*(void *const *)(addr))
#define pgm_read_pointer(addr) (*(void *const *)(addr))
#define pgm_read_byte_near(addr) pgm_read_byte(addr)
#define pgm_read_word_near(addr) pgm_read_word(addr)
#define pgm_read_dword_near(addr) pgm_read_dword(addr)
#define pgm_read_byte_far(addr) pgm_read_byte(addr)
#define pgm_read_word_far(addr) pgm_read_word(addr)
#define memcpy_P memcpy
#define memcmp_P memcmp
#define strcpy_P strcpy
#define strncpy_P strncpy
#define strcmp_P strcmp
#define strncmp_P strncmp
#define strlen_P strlen
#define strcat_P strcat
#define sprintf_P sprintf
#define snprintf_P snprintf
#define vsnprintf_P vsnprintf
#ifdef __cplusplus
class __FlashStringHelper;
#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(string_literal))
#define FPSTR(p) (reinterpret_cast<const __FlashStringHelper *>(p))
#endif
