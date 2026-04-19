/*
 * Brain (UNO Q MCU): TCP server for minions via Linux networking (WiFi/Ethernet).
 *
 * Requires the "Arduino_RouterBridge" library (Library Manager). The UNO Q MPU
 * runs a router that exposes tcp/* RPCs; WiFi is joined on the Linux side
 * (system settings or nmcli), then this sketch accepts clients on all interfaces.
 * Repo helper: from brain/, copy mpu-wifi.env.example to mpu-wifi.env; see Makefile (wifi-push, deploy).
 */

#include <Arduino_RouterBridge.h>
#include <string.h>

static constexpr uint16_t kBrainPort = 8080;

// Listen on all addresses so minions reach the board over WiFi/LAN.
static IPAddress kBindAny(0, 0, 0, 0);

static BridgeTCPServer<> server(Bridge, kBindAny, kBrainPort);

void setup() {
  if (!Monitor.begin()) {
    return;
  }

  if (!Bridge.begin()) {
    Monitor.println("Brain: Bridge.begin failed (Linux router not ready?)");
    return;
  }

  server.begin();
  if (!server.is_listening()) {
    Monitor.println("Brain: tcp/listen failed");
    return;
  }

  Monitor.print("Brain: listening on port ");
  Monitor.println(kBrainPort);
}

void loop() {
  BridgeTCPClient<> client = server.accept();

  if (client.connected() == 1) {
    Monitor.print("Brain: minion connected id=");
    Monitor.println(client.getId());
  }

  while (client.connected()) {
    const int n = client.available();
    if (n > 0) {
      uint8_t buf[256];
      const size_t take =
          (size_t)n >= sizeof(buf) ? sizeof(buf) - 1 : (size_t)n;
      const int r = client.read(buf, take);
      if (r > 0) {
        buf[r] = '\0';
        Monitor.print("Brain: from minion: ");
        Monitor.println(reinterpret_cast<char *>(buf));
      }

      const char *goal = "GOAL: MOVE_FORWARD\n";
      client.write(reinterpret_cast<const uint8_t *>(goal), strlen(goal));
      delay(1000);
    }
  }

  server.disconnect();
}
