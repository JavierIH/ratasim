// Program memory: on the host, ordinary memory.
#pragma once
#include <stdio.h>
#include <string.h>
#define PROGMEM
#define PSTR(s) (s)
#define pgm_read_byte(p) (*(const uint8_t *)(p))
#define pgm_read_word(p) (*(const uint16_t *)(p))
#define pgm_read_byte_near(p) pgm_read_byte(p)
#define pgm_read_word_near(p) (*(p))
#define pgm_read_dword(p) (*(const uint32_t *)(p))
#define pgm_read_float(p) (*(const float *)(p))
#define pgm_read_ptr(p) (*(void *const *)(p))
#define strncpy_P strncpy
#define strcpy_P strcpy
#define strcmp_P strcmp
#define strncmp_P strncmp
#define strlen_P strlen
#define sprintf_P sprintf
#define snprintf_P snprintf
#define printf_P printf
