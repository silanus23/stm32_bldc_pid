/**
 * @file    settings.c
 * @brief   Settings record construction and validation (pure, no HAL).
 */
#include "settings.h"
#include "app_config.h"

#include <math.h>

settings_t settings_make(float kp, float ki, float kd, float max_rpm)
{
    settings_t s = {
        .magic = SETTINGS_MAGIC,
        .version = SETTINGS_VERSION,
        .kp = kp,
        .ki = ki,
        .kd = kd,
        .max_rpm = max_rpm,
    };
    return s;
}

bool settings_valid(const settings_t *s)
{
    return s->magic == SETTINGS_MAGIC &&
           s->version == SETTINGS_VERSION &&
           isfinite(s->kp) && s->kp >= 0.0f &&
           isfinite(s->ki) && s->ki >= 0.0f &&
           isfinite(s->kd) && s->kd >= 0.0f &&
           isfinite(s->max_rpm) &&
           s->max_rpm >= MAX_RPM_LIMIT_MIN && s->max_rpm <= MAX_RPM_LIMIT_MAX;
}
