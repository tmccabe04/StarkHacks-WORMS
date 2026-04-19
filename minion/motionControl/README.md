# motionControl

ESP32-S3 minion sketch for differential-drive control with:
- MPU9250 IMU feedback
- L298N motor driver output
- Local serial command interface for motion commands

## Commands

- `MOVE <cm> [max_cm_s] [max_cm_s2]`
- `TURN <deg> [max_deg_s] [max_deg_s2]`
- `STOP`
- `STATUS`
- `CLEARFAULT`
- `HELP`

## Before running on hardware

1. Set your GPIO mapping in `robot_config.h` (`MotorPinMap`).
2. Confirm wheel geometry values in `RobotConfig`:
   - wheel diameter: `0.057 m`
   - wheelbase: `0.090 m`
3. Tune PID and motor model constants for your hardware.

## Notes

- v1 is encoderless; linear distance is model/IMU estimated and intended for prototype-grade short moves.
- Turn control uses closed-loop yaw feedback and a trapezoidal turn profile.
