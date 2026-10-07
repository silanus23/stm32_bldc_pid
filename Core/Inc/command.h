/**
 * @file    command.h
 * @brief   Serial command line assembly and parsing (pure, no HAL).
 *
 * Commands (case-insensitive, one per line):
 *   s<rpm>   setpoint, >= 0 (also clamped to max_rpm when applied)
 *   m<pct>   manual PWM, clamped to 0..100
 *   p<val>   Kp, >= 0
 *   i<val>   Ki, >= 0
 *   d<val>   Kd, >= 0
 *   r<rpm>   max RPM limit, clamped to MAX_RPM_LIMIT_MIN..MAX_RPM_LIMIT_MAX
 *   save     store Kp/Ki/Kd/max_rpm in flash
 */
#ifndef COMMAND_H
#define COMMAND_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CMD_INVALID = 0,
    CMD_SETPOINT,
    CMD_MANUAL,
    CMD_KP,
    CMD_KI,
    CMD_KD,
    CMD_MAX_RPM,
    CMD_SAVE,
} command_type_t;

typedef struct {
    command_type_t type;
    float value;
} command_t;

#define COMMAND_LINE_MAX 64

typedef struct {
    char buf[COMMAND_LINE_MAX];
    uint32_t len;
    bool overflow;
} command_line_t;

/** Parses one complete line (without line terminator). */
command_t command_parse(const char *line);

/** Feeds one byte; returns true when a complete line is ready in @p line_out. */
bool command_line_feed(command_line_t *lb, uint8_t c, const char **line_out);

#endif /* COMMAND_H */
