/**
 * @file    tacho.h
 * @brief   Tachometer period to RPM conversion (pure math, no HAL).
 */
#ifndef TACHO_H
#define TACHO_H

#include <stdbool.h>
#include <stdint.h>

/** Converts the time between two tach pulses to RPM; false if it is noise. */
bool tacho_rpm_from_period(uint32_t period_us, float *rpm_out);

#endif /* TACHO_H */
