#pragma once

#include <Arduino.h>

#include "open_loop_controller.h"

class SerialCommands {
public:
  explicit SerialCommands(OpenLoopController* controller) : controller_(controller) {}

  void poll();
  void printHelp() const;

private:
  static constexpr size_t kLineBufferSize = 112;

  void handleLine(const char* line);

  OpenLoopController* controller_ {nullptr};
  char line_[kLineBufferSize] {};
  size_t len_ {0};
};
