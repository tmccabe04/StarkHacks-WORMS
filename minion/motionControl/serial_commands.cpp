#include "serial_commands.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {

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

bool parseFloat(const char* token, float* out) {
  if (token == nullptr || out == nullptr) {
    return false;
  }
  char* end_ptr = nullptr;
  const float value = strtof(token, &end_ptr);
  if (end_ptr == token || *end_ptr != '\0') {
    return false;
  }
  *out = value;
  return true;
}

void toUpper(char* text) {
  while (*text != '\0') {
    *text = static_cast<char>(toupper(*text));
    ++text;
  }
}

}  // namespace

void SerialCommands::poll() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      line_[len_] = '\0';
      if (len_ > 0) {
        handleLine(line_);
      }
      len_ = 0;
      continue;
    }

    if (len_ + 1 < kLineBufferSize) {
      line_[len_++] = c;
    }
  }
}

void SerialCommands::printHelp() const {
  Serial.println("Commands:");
  Serial.println("  MOVE <cm> [max_cm_s] [max_cm_s2]");
  Serial.println("  TURN <deg> [max_deg_s] [max_deg_s2]");
  Serial.println("  STOP");
  Serial.println("  STATUS");
  Serial.println("  CLEARFAULT");
  Serial.println("  HELP");
}

void SerialCommands::handleLine(const char* line) {
  if (controller_ == nullptr || line == nullptr) {
    return;
  }

  char buffer[kLineBufferSize];
  strncpy(buffer, line, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';

  char* token = strtok(buffer, " \t");
  if (token == nullptr) {
    return;
  }

  toUpper(token);

  if (strcmp(token, "HELP") == 0) {
    printHelp();
    return;
  }

  if (strcmp(token, "STATUS") == 0) {
    const Telemetry telemetry = controller_->getTelemetry();
    Serial.printf(
      "STATE=%s FAULT=%s YAW=%.2fdeg RATE=%.3frad/s DIST_EST=%.3fm L=%.2f R=%.2f\n",
      stateName(telemetry.state),
      faultName(telemetry.fault),
      telemetry.yaw_deg,
      telemetry.yaw_rate_rad_s,
      telemetry.estimated_distance_m,
      telemetry.left_motor_cmd,
      telemetry.right_motor_cmd
    );
    return;
  }

  if (strcmp(token, "CLEARFAULT") == 0) {
    controller_->clearFault();
    Serial.println("OK CLEARFAULT");
    return;
  }

  if (strcmp(token, "STOP") == 0) {
    MotionCommand cmd;
    cmd.type = MotionCommandType::Stop;
    if (controller_->submitCommand(cmd)) {
      Serial.println("OK STOP");
    } else {
      Serial.println("ERR STOP");
    }
    return;
  }

  if (strcmp(token, "MOVE") == 0) {
    MotionCommand cmd;
    cmd.type = MotionCommandType::MoveCm;

    char* value_tok = strtok(nullptr, " \t");
    if (!parseFloat(value_tok, &cmd.value)) {
      Serial.println("ERR MOVE usage: MOVE <cm> [max_cm_s] [max_cm_s2]");
      return;
    }

    char* speed_tok = strtok(nullptr, " \t");
    char* accel_tok = strtok(nullptr, " \t");

    if (speed_tok != nullptr && !parseFloat(speed_tok, &cmd.max_speed)) {
      Serial.println("ERR MOVE invalid max_cm_s");
      return;
    }
    if (accel_tok != nullptr && !parseFloat(accel_tok, &cmd.max_accel)) {
      Serial.println("ERR MOVE invalid max_cm_s2");
      return;
    }

    if (controller_->submitCommand(cmd)) {
      Serial.printf("OK MOVE %.2fcm\n", cmd.value);
    } else {
      Serial.println("ERR MOVE submit failed");
    }
    return;
  }

  if (strcmp(token, "TURN") == 0) {
    MotionCommand cmd;
    cmd.type = MotionCommandType::TurnDeg;

    char* value_tok = strtok(nullptr, " \t");
    if (!parseFloat(value_tok, &cmd.value)) {
      Serial.println("ERR TURN usage: TURN <deg> [max_deg_s] [max_deg_s2]");
      return;
    }

    char* speed_tok = strtok(nullptr, " \t");
    char* accel_tok = strtok(nullptr, " \t");

    if (speed_tok != nullptr && !parseFloat(speed_tok, &cmd.max_speed)) {
      Serial.println("ERR TURN invalid max_deg_s");
      return;
    }
    if (accel_tok != nullptr && !parseFloat(accel_tok, &cmd.max_accel)) {
      Serial.println("ERR TURN invalid max_deg_s2");
      return;
    }

    if (controller_->submitCommand(cmd)) {
      Serial.printf("OK TURN %.2fdeg\n", cmd.value);
    } else {
      Serial.println("ERR TURN submit failed");
    }
    return;
  }

  Serial.println("ERR Unknown command. Use HELP.");
}
