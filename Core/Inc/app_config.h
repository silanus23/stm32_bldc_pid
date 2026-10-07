/**
 * @file    app_config.h
 * @brief   Application tuning constants and limits.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* PID loop */
#define PID_SAMPLE_TIME_MS      25
#define PID_SAMPLE_TIME_S       (PID_SAMPLE_TIME_MS / 1000.0f)
#define PID_OUTPUT_MAX          95.0f
#define PID_OUTPUT_MIN          10.0f

/* Default settings, used when flash holds no valid record */
#define DEFAULT_KP              0.1f
#define DEFAULT_KI              0.7f
#define DEFAULT_KD              0.001f
#define DEFAULT_SETPOINT_RPM    1600.0f
#define DEFAULT_MAX_RPM         3100.0f

/* Command limits */
#define MANUAL_PWM_MAX          100.0f
#define MAX_RPM_LIMIT_MIN       100.0f
#define MAX_RPM_LIMIT_MAX       10000.0f

/* Tachometer */
#define TACHO_PULSES_PER_REV    2.0f
#define TACHO_TIMEOUT_MS        1000
#define TACHO_GLITCH_RPM        10000.0f

#define TELEMETRY_PERIOD_MS     250
#define WATCHDOG_TASK_TIMEOUT_MS 3000

#endif /* APP_CONFIG_H */
