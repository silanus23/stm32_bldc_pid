/**
 * @file    tacho.c
 * @brief   Tachometer period to RPM conversion (pure math, no HAL).
 */
#include "tacho.h"
#include "app_config.h"

bool tacho_rpm_from_period(uint32_t period_us, float *rpm_out)
{
    if (period_us == 0)
    {
        return false;
    }

    float rpm = 60000000.0f / ((float)period_us * TACHO_PULSES_PER_REV);
    if (rpm > TACHO_GLITCH_RPM)
    {
        return false;
    }

    *rpm_out = rpm;
    return true;
}
