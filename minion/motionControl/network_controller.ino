#include <WiFi.h>
#include "motion_controller.h"
#include "serial_commands.h"

// TODO: Replace with your actual credentials
const char* ssid = "StarkHacks-5";
const char* password = "StarkHacks2026";
const char* brain_ip = "0.0.0.00"; // Linux Brain IP
const int brain_port = 10101;

WiFiClient client;
RobotConfig kConfig = defaultRobotConfig();
MotionController controller;
SerialCommands command_interface(&controller);

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  if (!controller.begin(kConfig)) {
    Serial.println("Controller init failed");
  }
}

void loop() {
  if (!client.connected()) {
    if (client.connect(brain_ip, brain_port)) {
      Serial.println("Connected to brain");
    } else {
      delay(5000);
      return;
    }
  }

  // Poll for commands from the Brain
  if (client.available()) {
    String cmd = client.readStringUntil('\n');
    Serial.println("Received from Brain: " + cmd);
    // Here you would process the command, e.g., pass it to command_interface.handleLine(cmd.c_str())
  }
}
