#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "MPU9250.h"
#include <Arduino.h>
#include <HardwareSerial.h>
void setRGB(int r, int g, int b);
