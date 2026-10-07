/**
 * @file    command.c
 * @brief   Serial command line assembly and parsing (pure, no HAL).
 */
#include "command.h"
#include "app_config.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Parses a float, allowing surrounding whitespace. Rejects NaN/inf/overflow. */
static bool parse_float(const char *str, float *out)
{
    while (*str == ' ' || *str == '\t') str++;
    if (*str == '\0')
    {
        return false;
    }

    char *endptr;
    float val = strtof(str, &endptr);
    if (endptr == str || !isfinite(val))
    {
        return false;
    }

    while (*endptr == ' ' || *endptr == '\t') endptr++;
    if (*endptr != '\0')
    {
        return false;
    }

    *out = val;
    return true;
}

static float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

command_t command_parse(const char *line)
{
    command_t cmd = { CMD_INVALID, 0.0f };
    char lc[COMMAND_LINE_MAX];

    while (*line == ' ' || *line == '\t') line++;

    size_t n = 0;
    for (; line[n] != '\0' && n < sizeof(lc) - 1; n++)
    {
        lc[n] = (char)tolower((unsigned char)line[n]);
    }
    while (n > 0 && (lc[n - 1] == ' ' || lc[n - 1] == '\t')) n--;
    lc[n] = '\0';

    if (n == 0)
    {
        return cmd;
    }
    if (strcmp(lc, "save") == 0)
    {
        cmd.type = CMD_SAVE;
        return cmd;
    }

    float val;
    if (!parse_float(&lc[1], &val))
    {
        return cmd;
    }

    switch (lc[0])
    {
    case 's': cmd.type = CMD_SETPOINT; cmd.value = val < 0.0f ? 0.0f : val; break;
    case 'm': cmd.type = CMD_MANUAL;   cmd.value = clampf(val, 0.0f, MANUAL_PWM_MAX); break;
    case 'p': cmd.type = CMD_KP;       cmd.value = val < 0.0f ? 0.0f : val; break;
    case 'i': cmd.type = CMD_KI;       cmd.value = val < 0.0f ? 0.0f : val; break;
    case 'd': cmd.type = CMD_KD;       cmd.value = val < 0.0f ? 0.0f : val; break;
    case 'r': cmd.type = CMD_MAX_RPM;  cmd.value = clampf(val, MAX_RPM_LIMIT_MIN, MAX_RPM_LIMIT_MAX); break;
    default:  break;
    }
    return cmd;
}

bool command_line_feed(command_line_t *lb, uint8_t c, const char **line_out)
{
    if (c == '\r' || c == '\n')
    {
        if (lb->len == 0 && !lb->overflow)
        {
            return false;               /* empty line, or second half of CRLF */
        }
        lb->buf[lb->len] = '\0';
        *line_out = lb->overflow ? "" : lb->buf;
        lb->len = 0;
        lb->overflow = false;
        return true;
    }

    if (lb->len < sizeof(lb->buf) - 1)
    {
        lb->buf[lb->len++] = (char)c;
    }
    else
    {
        lb->overflow = true;
    }
    return false;
}
