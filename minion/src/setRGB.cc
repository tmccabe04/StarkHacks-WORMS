#include <Arduino.h>

void setRGB(int r, int g, int b) {
  neopixelWrite(RGB_BUILTIN, g, r, b);
}
