# STM32 PID Fan Controller

A closed-loop fan speed controller built on STM32F407VG. The goal was to implement a real PID control system on bare metal with FreeRTOS, and actually understand what I was building rather than copy-pasting examples.

## Why FreeRTOS

This project could have been done bare-metal in a single loop. FreeRTOS was a deliberate choice to learn it — task priorities, semaphores, and shared state protection under preemption. The three-task structure reflects real concerns: the tachometer needs to respond to hardware events immediately, the PID loop needs to run at a fixed rate, and telemetry shouldn't interfere with either. Using FreeRTOS forced me to think about race conditions and synchronization that a single-loop approach would have hidden.

## How It Works

The fan speed is measured via a tachometer signal on PA0 — two pulses per revolution, falling edge interrupt. Each pulse timestamp is captured from TIM2 (1µs resolution, 32-bit free-running counter), and the RPM is calculated from the interval between consecutive pulses. If no pulse arrives within 1 second, the fan is considered stalled and RPM is set to zero.

The PID loop runs at 40Hz. Output is feedforward + P + I + D, where the feedforward is a 9-point lookup table with linear interpolation, calibrated open-loop against this specific fan. Derivative is calculated on measurement rather than error, which avoids the kick on setpoint changes. Anti-windup clamps the integral relative to what headroom is left after the feedforward contribution, not against absolute output limits.

Transitions from manual PWM mode back to automatic control use bumpless transfer — the integral is back-calculated from the current output so the fan doesn't jerk.

Settings (Kp, Ki, Kd, max RPM) are saved to flash sector 11 with write verification and loaded on boot.

## Hardware

- **MCU**: STM32F407VG (168MHz)
- **Fan control**: 2.5kHz PWM on PD12 (TIM4 CH1)
- **Tachometer**: EXTI0 on PA0, falling edge, 2 pulses/rev
- **Interface**: USB CDC virtual COM port (PA11/PA12)

## Control Architecture

**PID Loop — 40Hz**
- Output range: 10–95% (0% only when setpoint is 0)
- Feedforward: 9-point LUT, 740–2830 RPM → 10–90% PWM
- Anti-windup: integral clamped accounting for feedforward contribution
- Derivative on measurement

**Task Structure**
- `tachoTask` (High priority): event-driven, wakes on semaphore from ISR
- `pidTask` (Normal priority): fixed 25ms period via `osDelayUntil`
- `usbTask` (Normal priority): 250ms telemetry transmit with retry

**Thread Safety**

Shared variables between tasks are protected with `osKernelLock`/`Unlock` wrappers. The PID integral gets a full critical section covering the read-modify-write and clamp. Simple float reads/writes use inline atomic helpers.

## Serial Commands

| Command | Description |
|---------|-------------|
| `s<value>` | Set RPM setpoint (0–4000) |
| `m<value>` | Manual PWM mode (0–100%) |
| `p<value>` | Set Kp |
| `i<value>` | Set Ki |
| `d<value>` | Set Kd |
| `r<value>` | Set max RPM limit |
| `save` | Save settings to flash |

## Telemetry Output

```
Time:   12.50, Set: 1600.0, Meas: 1598.3, PWM: 31.4
Time:   12.75, Set: 1600.0, Meas: 1601.7, PWM: 31.2
```

## Python Interface

`fan_controller.py` handles simultaneous telemetry display and command input using two threads. Update `SERIAL_PORT` if your device appears on a different port (default: `/dev/ttyACM1`).

## Building

Manual clock configuration for 168MHz operation. Do not regenerate from STM32CubeMX — it will break the working configuration.

## Possible Improvements

- IWDG is initialized but never fed — fault recovery is unfinished
- Bumpless transfer only handles manual→auto; auto→manual has no equivalent
- `tx_buffer` is a global but is only used in `usbTask`
- Auto-tuning PID parameters based on step response
- Data logging to SD card for long-term performance analysis
