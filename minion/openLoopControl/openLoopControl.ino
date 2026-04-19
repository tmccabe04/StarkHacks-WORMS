#include <Arduino.h>

#include "open_loop_controller.h"
#include "robot_config.h"
#include "serial_commands.h"

namespace {

RobotConfig kConfig = defaultRobotConfig();
OpenLoopController controller;
SerialCommands command_interface(&controller);

uint32_t last_control_tick_ms = 0;
uint32_t last_print_ms = 0;

const char* stateName(ControlState state) {
  switch (state) {
    case ControlState::Idle:
      return "IDLE";
    case ControlState::Executing:
      return "EXECUTING";
    case ControlState::Stopping:
      return "STOPPING";
    case ControlState::Fault:
      return "FAULT";
  }
  return "UNKNOWN";
}

const char* faultName(FaultCode fault) {
  switch (fault) {
    case FaultCode::None:
      return "NONE";
    case FaultCode::MotorInitFailed:
      return "MOTOR_INIT_FAILED";
    case FaultCode::CommandTimeout:
      return "COMMAND_TIMEOUT";
    case FaultCode::ControlLoopOverrun:
      return "CONTROL_OVERRUN";
    case FaultCode::QueueFull:
      return "QUEUE_FULL";
    case FaultCode::InvalidTuning:
      return "INVALID_TUNING";
  }
  return "UNKNOWN";
}

void setStatusLed(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(RGB_BUILTIN, g, r, b);
}

void updateStatusLed(ControlState state, uint32_t now_ms) {
  static uint32_t last_blink_ms = 0;
  static bool on = false;

  if (state == ControlState::Executing) {
    if (now_ms - last_blink_ms >= 140) {
      last_blink_ms = now_ms;
      on = !on;
    }
    if (on) {
      setStatusLed(0, 90, 0);
    } else {
      setStatusLed(0, 0, 0);
    }
    return;
  }

  if (state == ControlState::Fault) {
    setStatusLed(120, 0, 0);
    return;
  }

  if (state == ControlState::Stopping) {
    setStatusLed(80, 30, 0);
    return;
  }

  setStatusLed(0, 0, 30);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("minion open-loop motion controller boot");
  Serial.println("No IMU/encoder feedback: motion is time/feedforward based.");
  Serial.println("Set motor pins in robot_config.h before running on hardware.");

  if (!controller.begin(kConfig)) {
    Serial.println("controller init failed");
  }

  command_interface.printHelp();

  last_control_tick_ms = millis();
  last_print_ms = millis();
}

void loop() {
  command_interface.poll();

  const uint32_t now_ms = millis();

  if (now_ms - last_control_tick_ms >= kConfig.control_period_ms) {
    last_control_tick_ms = now_ms;
    controller.controlTick(now_ms);
  }

  const Telemetry telemetry = controller.getTelemetry();
  updateStatusLed(telemetry.state, now_ms);

  if (now_ms - last_print_ms >= 250) {
    last_print_ms = now_ms;
    Serial.printf(
      "state=%s fault=%s dist=%.3fm heading=%.1fdeg l=%.2f r=%.2f prog=%.2f\n",
      stateName(telemetry.state),
      faultName(telemetry.fault),
      telemetry.estimated_distance_m,
      telemetry.estimated_heading_deg,
      telemetry.left_motor_cmd,
      telemetry.right_motor_cmd,
      telemetry.command_progress
    );
  }
}
