/**
 * @file    pid.h
 * @brief   Feedforward + PID speed controller (pure math, no HAL).
 */
#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
} pid_gains_t;

typedef struct {
    float integral;
    float last_measured;
} pid_state_t;

/** Clears the integral and derivative history. */
void pid_reset(pid_state_t *state);

/** Open-loop PWM (%) expected to reach @p rpm, from the calibrated lookup table. */
float pid_feedforward(float rpm);

/** Runs one controller step and returns the PWM duty in %. */
float pid_update(pid_state_t *state, const pid_gains_t *gains,
                 float setpoint, float measured, float dt);

/** Sets the integral so control continues from @p current_output (bumpless transfer). */
void pid_bumpless_transfer(pid_state_t *state, const pid_gains_t *gains,
                           float setpoint, float measured, float current_output);

#endif /* PID_H */
