#line 1 "/Users/tommy/Documents/projects/StarkHacks-WORMS/brain/src/brain.h"
#include <sys/types.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <Arduino.h>
#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "MPU9250.h"

void server();
void center();
void handleChild(int client_fd);
