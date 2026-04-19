#include "minion.h"
#include <WiFi.h>

WiFiClient client;

bool connectToBrain(const char* ip, uint16_t port) {
    return client.connect(ip, port);
}

String receiveGoal() {
    if (client.available()) {
        return client.readStringUntil('\n');
    }
    return "";
}

bool isConnected() {
    return client.connected();
}
