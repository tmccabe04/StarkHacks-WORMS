# motorTest

Simple bring-up sketch for verifying L298N + motor wiring.

## What it does

- Runs an automatic stage sequence at different speeds and directions.
- Tests left motor alone, right motor alone, then both motors together.
- Inserts a stop delay between stages for safety and easy observation.

## Wiring used

- Left motor: `ENA=9`, `IN1=10`, `IN2=11`
- Right motor: `ENB=14`, `IN3=12`, `IN4=13`
- ENA/ENB jumpers removed (required for PWM speed control)

## Usage

1. Upload `motorTest`.
2. Open serial monitor at `115200`.
3. Observe stage logs and confirm motor behavior.

Reset the board to restart the sequence.
