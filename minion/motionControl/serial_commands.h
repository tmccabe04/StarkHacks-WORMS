#pragma once

#include <Arduino.h>

#include "motion_controller.h"

class SerialCommands {
public:
  explicit SerialCommands(MotionController* controller) : controller_(controller) {}

  void poll();
  void printHelp() const;

private:
  static constexpr size_t kLineBufferSize = 96;

  void handleLine(const char* line);

  MotionController* controller_ {nullptr};
  char line_[kLineBufferSize] {};
  size_t len_ {0};
};
