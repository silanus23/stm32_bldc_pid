/**
 * @file    pid.c
 * @brief   Feedforward + PID speed controller (pure math, no HAL).
 *
 * output = FF(setpoint) + Kp*e + Ki*integral - Kd*d(measured)/dt
 */
#include "pid.h"
#include "app_config.h"
#include <stdbool.h>

static const float ff_rpm_points[] = { 740, 1250, 1600, 1860, 2100, 2325, 2515, 2675, 2830 };
static const float ff_pwm_points[] = { 10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f, 70.0f, 80.0f, 90.0f };
#define FF_NUM_POINTS ((int)(sizeof(ff_rpm_points) / sizeof(ff_rpm_points[0])))

_Static_assert(sizeof(ff_rpm_points) == sizeof(ff_pwm_points),
               "feedforward tables must have the same length");

void pid_reset(pid_state_t *state)
{
    state->integral = 0.0f;
    state->last_measured = 0.0f;
}

float pid_feedforward(float rpm)
{
    if (rpm <= ff_rpm_points[0])
    {
        return ff_pwm_points[0];
    }
    if (rpm >= ff_rpm_points[FF_NUM_POINTS - 1])
    {
        return ff_pwm_points[FF_NUM_POINTS - 1];
    }

    for (int i = 0; i < FF_NUM_POINTS - 1; i++)
    {
        if (rpm <= ff_rpm_points[i + 1])
        {
            float x0 = ff_rpm_points[i];
            float y0 = ff_pwm_points[i];
            float x1 = ff_rpm_points[i + 1];
            float y1 = ff_pwm_points[i + 1];
            return y0 + (rpm - x0) * (y1 - y0) / (x1 - x0);
        }
    }

    return ff_pwm_points[FF_NUM_POINTS - 1];
}

static float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/* Integral limits so that FF + Ki*integral stays within the output range. */
static float clamp_integral(float integral, float ff_term, float ki)
{
    return clampf(integral, (PID_OUTPUT_MIN - ff_term) / ki, (PID_OUTPUT_MAX - ff_term) / ki);
}

float pid_update(pid_state_t *state, const pid_gains_t *gains,
                 float setpoint, float measured, float dt)
{
    if (setpoint <= 0.0f)
    {
        pid_reset(state);
        return 0.0f;
    }

    float error = setpoint - measured;
    float ff_term = pid_feedforward(setpoint);
    float p_term = gains->kp * error;

    float derivative = 0.0f;
    if (measured > 0.0f && state->last_measured > 0.0f)
    {
        derivative = -(measured - state->last_measured) / dt;
    }
    float d_term = gains->kd * derivative;
    state->last_measured = measured;

    if (gains->ki > 0.0f)
    {
        float candidate = state->integral + error * dt;
        float unsaturated = ff_term + p_term + gains->ki * candidate + d_term;
        bool winding_up = (unsaturated > PID_OUTPUT_MAX && error > 0.0f) ||
                          (unsaturated < PID_OUTPUT_MIN && error < 0.0f);
        if (!winding_up)
        {
            state->integral = candidate;
        }
        state->integral = clamp_integral(state->integral, ff_term, gains->ki);
    }
    else
    {

        state->integral = 0.0f;
    }

    float output = ff_term + p_term + gains->ki * state->integral + d_term;
    return clampf(output, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
}

void pid_bumpless_transfer(pid_state_t *state, const pid_gains_t *gains,
                           float setpoint, float measured, float current_output)
{
    state->last_measured = measured;

    if (gains->ki <= 0.0f || setpoint <= 0.0f)
    {
        state->integral = 0.0f;
        return;
    }

    /* output = ff + p + Ki*integral  =>  integral = (output - ff - p) / Ki */
    float ff_term = pid_feedforward(setpoint);
    float p_term = gains->kp * (setpoint - measured);
    float integral = (current_output - ff_term - p_term) / gains->ki;
    state->integral = clamp_integral(integral, ff_term, gains->ki);
}
