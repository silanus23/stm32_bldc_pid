/**
 * @file    settings.h
 * @brief   Persistent settings record (validation is pure; flash I/O in settings_flash.c).
 */
#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdbool.h>
#include <stdint.h>

#define SETTINGS_MAGIC    0xDEADBEEFu
#define SETTINGS_VERSION  2u

typedef struct {
    uint32_t magic;
    uint32_t version;
    float kp;
    float ki;
    float kd;
    float max_rpm;
} settings_t;

_Static_assert(sizeof(settings_t) == 6 * 4, "settings_t must have no padding");

/** Builds a record with the current magic/version. */
settings_t settings_make(float kp, float ki, float kd, float max_rpm);

/** True if magic/version match and every value is finite and in range. */
bool settings_valid(const settings_t *s);

/** Erases the settings sector and writes @p s. Task context only; stalls the CPU ~1-2 s. */
bool settings_save(const settings_t *s);

/** Reads the record from flash. @return true only if it passes settings_valid(). */
bool settings_load(settings_t *out);

#endif /* SETTINGS_H */
