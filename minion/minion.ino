#include "src/minion.h"
#include "src/secrets.h"
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  printf("Minion starting...\n");

  WiFi.begin(WORMS_WIFI_SSID, WORMS_WIFI_PSK);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        printf(".");
    }
    printf("\nWiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());

    while (!connectToBrain(WORMS_BRAIN_IP, WORMS_BRAIN_PORT)) {
        printf("Connecting to brain failed, retrying...\n");
        delay(2000);
    }
}

void loop() {
    if (!isConnected()) {
        connectToBrain(WORMS_BRAIN_IP, WORMS_BRAIN_PORT);
        delay(1000);
        return;
    }

    String goal = receiveGoal();
    if (goal.length() > 0) {
        Serial.printf("Goal: %s\n", goal.c_str());
    }
}
