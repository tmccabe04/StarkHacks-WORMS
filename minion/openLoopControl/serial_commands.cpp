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
  Serial.println("  TUNING");
  Serial.println("  SET SPEED_MPS_PER_DUTY <value>");
  Serial.println("  SET MIN_START_DUTY <0..1>");
  Serial.println("  SET MAX_DUTY <0..1>");
  Serial.println("  SET SLEW_PER_S <value>");
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
      "STATE=%s FAULT=%s DIST_EST=%.3fm HDG_EST=%.1fdeg L=%.2f R=%.2f PROG=%.2f\n",
      stateName(telemetry.state),
      faultName(telemetry.fault),
      telemetry.estimated_distance_m,
      telemetry.estimated_heading_deg,
      telemetry.left_motor_cmd,
      telemetry.right_motor_cmd,
      telemetry.command_progress
    );
    return;
  }

  if (strcmp(token, "TUNING") == 0) {
    const RuntimeTuning tuning = controller_->tuning();
    Serial.printf(
      "TUNING SPEED_MPS_PER_DUTY=%.4f MIN_START_DUTY=%.3f MAX_DUTY=%.3f SLEW_PER_S=%.3f\n",
      tuning.duty_to_speed_m_s,
      tuning.min_start_duty,
      tuning.max_duty,
      tuning.command_slew_per_s
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

  if (strcmp(token, "SET") == 0) {
    char* key = strtok(nullptr, " \t");
    char* value_tok = strtok(nullptr, " \t");
    if (key == nullptr || value_tok == nullptr) {
      Serial.println("ERR SET usage: SET <key> <value>");
      return;
    }

    toUpper(key);

    float value = 0.0f;
    if (!parseFloat(value_tok, &value)) {
      Serial.println("ERR SET invalid value");
      return;
    }

    bool ok = false;
    if (strcmp(key, "SPEED_MPS_PER_DUTY") == 0) {
      ok = controller_->setDutyToSpeedMpsPerDuty(value);
    } else if (strcmp(key, "MIN_START_DUTY") == 0) {
      ok = controller_->setMinStartDuty(value);
    } else if (strcmp(key, "MAX_DUTY") == 0) {
      ok = controller_->setMaxDuty(value);
    } else if (strcmp(key, "SLEW_PER_S") == 0) {
      ok = controller_->setCommandSlewPerS(value);
    } else {
      Serial.println("ERR SET unknown key");
      return;
    }

    if (!ok) {
      Serial.println("ERR SET out of range");
      return;
    }

    Serial.printf("OK SET %s=%.4f\n", key, value);
    return;
  }

  Serial.println("ERR Unknown command. Use HELP.");
}
