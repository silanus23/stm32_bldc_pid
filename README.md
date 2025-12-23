# STM32 PID Fan Controller

A real-time fan speed controller using PID control with feedforward compensation, implemented on STM32F407VGT6 with FreeRTOS.

## Features

- **PID Control**: Precise RPM control with derivative-on-measurement and anti-windup
- **Feedforward Compensation**: 9-point lookup table for improved response
- **Tachometer Feedback**: Real-time RPM measurement with timeout detection
- **USB CDC Interface**: Live telemetry and interactive command control
- **Flash Persistence**: Settings saved across power cycles
- **Manual Override**: Direct PWM control for testing
- **Bumpless Transfer**: Smooth transition between manual and automatic modes
- **Thread-Safe Design**: Atomic operations and critical sections for data integrity

## Hardware

- **MCU**: STM32F407VGT6 (168MHz)
- **Fan Control**: 25kHz PWM on PD12 (TIM4_CH1)
- **Tachometer**: External interrupt on PA0 (falling edge, 2 pulses/rev)
- **USB**: CDC Virtual COM Port (PA11/PA12)

## Python Interface

The included `fan_controller.py` script provides a clean interface for interacting with the controller:

```bash
python3 fan_controller.py
```

**Features:**
- Continuous telemetry display (Time, Setpoint, Measured RPM, PWM%)
- Command input without interference from telemetry data
- Automatic port handling with error messages

**Note:** Update `SERIAL_PORT` in the script if your device appears on a different port (default: `/dev/ttyACM1`).

## Control Architecture

### PID Loop
- **Sample Rate**: 40Hz (25ms)
- **Output Range**: 10-95% (0% allowed only at 0 RPM setpoint)
- **Anti-Windup**: Dynamic clamping accounting for feedforward contribution
- **Derivative**: Calculated on measurement to prevent derivative kick

### Feedforward Table
Linear interpolation between calibration points:

| RPM  | 740 | 1250 | 1600 | 1860 | 2100 | 2325 | 2515 | 2675 | 2830 |
|------|-----|------|------|------|------|------|------|------|------|
| PWM% | 10  | 20   | 30   | 40   | 50   | 60   | 70   | 80   | 90   |

### Task Structure
- **pidTask** (Priority: Normal): 40Hz PID calculation and output
- **usbTask** (Priority: Normal): 4Hz telemetry transmission
- **tachoTask** (Priority: High): RPM measurement with 1000ms timeout

## Building

This project uses manually configured clock settings (168MHz operation). The `.ioc` file is **not maintained** and should be ignored - do not regenerate code from STM32CubeMX as it will break the working configuration.

The system is configured for stable USB CDC operation. Clock settings have been tested and should not be modified without thorough USB testing.

## Example Usage

```
# Set target speed
s1800

# Adjust PID gains for your fan
p0.15
i0.5
d0.002

# Save settings to flash
save

# Test manual mode
m50

# Return to automatic control
s1800
```

## Telemetry Output

```
Time:   12.50, Set: 1600.0, Meas: 1598.3, PWM: 31.4
Time:   12.75, Set: 1600.0, Meas: 1601.7, PWM: 31.2
Time:   13.00, Set: 1600.0, Meas: 1599.5, PWM: 31.3
```

## Notes

- First tachometer pulse after startup is ignored to eliminate transients
- Fan reports 0 RPM if no pulses received within 1 second
- Setpoint of 0 RPM completely stops the fan (0% PWM)
- Settings are stored in Flash Sector 11 (0x080E0000)
- Independent watchdog is configured but not currently active

## Future Work

- **UART Sensor Integration**: Add external sensor reading (temperature, pressure, etc.) via UART for closed-loop environmental control
- **Watchdog Implementation**: Enable and feed IWDG for fault detection and automatic recovery
- **Multi-Fan Control**: Expand to control multiple fans with independent PID loops
- **Adaptive Tuning**: Implement auto-tuning for PID parameters based on system response
- **Data Logging**: Add SD card logging for long-term performance analysis

## License

This is a portfolio project demonstrating embedded control systems design.