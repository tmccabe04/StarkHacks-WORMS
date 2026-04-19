#include <Arduino.h>

#include "motion_controller.h"
#include "serial_commands.h"

namespace {

RobotConfig kConfig = defaultRobotConfig();
MotionController controller;
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
    case FaultCode::ImuUnavailable:
      return "IMU_UNAVAILABLE";
    case FaultCode::ImuStale:
      return "IMU_STALE";
    case FaultCode::CommandTimeout:
      return "COMMAND_TIMEOUT";
    case FaultCode::ControlLoopOverrun:
      return "CONTROL_OVERRUN";
    case FaultCode::QueueFull:
      return "QUEUE_FULL";
  }
  return "UNKNOWN";
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("minion motion controller boot");
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

  if (now_ms - last_print_ms >= 250) {
    last_print_ms = now_ms;
    const Telemetry telemetry = controller.getTelemetry();
    Serial.printf(
      "state=%s fault=%s yaw=%.1fdeg dist=%.3fm l=%.2f r=%.2f prog=%.2f\n",
      stateName(telemetry.state),
      faultName(telemetry.fault),
      telemetry.yaw_deg,
      telemetry.estimated_distance_m,
      telemetry.left_motor_cmd,
      telemetry.right_motor_cmd,
      telemetry.command_progress
    );
  }
}
