# openLoopControl

Differential-drive controller for ESP32-S3 + L298N without IMU/encoders.

## What it does

- Accepts high-level serial commands:
  - `MOVE <cm> [max_cm_s] [max_cm_s2]`
  - `TURN <deg> [max_deg_s] [max_deg_s2]`
  - `STOP`, `STATUS`, `CLEARFAULT`, `HELP`
- Uses trapezoidal motion profiles and open-loop feedforward to drive both wheels.
- Provides runtime tuning commands:
  - `TUNING`
  - `SET SPEED_MPS_PER_DUTY <value>`
  - `SET MIN_START_DUTY <0..1>`
  - `SET MAX_DUTY <0..1>`
  - `SET SLEW_PER_S <value>`

## Wiring

- Left motor: `ENA=9`, `IN1=10`, `IN2=11`
- Right motor: `ENB=14`, `IN3=12`, `IN4=13`
- Left motor polarity assumes `OUT2=left+`, `OUT1=left-`
- Right motor polarity assumes `OUT3=right+`, `OUT4=right-`
- ENA/ENB jumpers removed for PWM speed control.

## Notes

- Distance and angle are approximate because there is no feedback sensor.
- Tune feedforward constants on your floor/surface before relying on command accuracy.
