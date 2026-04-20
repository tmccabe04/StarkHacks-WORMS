#include <Arduino.h>
#include <Wire.h>

#include <MPU6500_WE.h>
#include <MPU9250_WE.h>

namespace {

constexpr uint8_t kSdaPin = 5;
constexpr uint8_t kSclPin = 4;
constexpr uint32_t kI2cClockHz = 400000;
constexpr uint32_t kI2cFallbackClockHz = 100000;
constexpr uint8_t kRegWhoAmI = 0x75;

struct ProbeResult {
  bool any_devices {false};
  bool ack_68 {false};
  bool ack_69 {false};
};

bool readRegister8(uint8_t addr, uint8_t reg, uint8_t* value) {
  if (value == nullptr) {
    return false;
  }

  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(static_cast<int>(addr), 1) != 1) {
    return false;
  }

  *value = Wire.read();
  return true;
}

bool pingAddress(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool scanBus() {
  Serial.println("I2C scan:");
  bool any = false;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    if (pingAddress(addr)) {
      Serial.printf("  found 0x%02X\n", addr);
      any = true;
    }
  }
  if (!any) {
    Serial.println("  no devices found");
  }
  return any;
}

void printWhoAmI(uint8_t addr) {
  uint8_t who = 0;
  if (!readRegister8(addr, kRegWhoAmI, &who)) {
    Serial.printf("addr 0x%02X: WHO_AM_I read failed\n", addr);
    return;
  }

  Serial.printf("addr 0x%02X: WHO_AM_I=0x%02X", addr, who);
  if (who == 0x71) {
    Serial.print(" (MPU9250)");
  } else if (who == 0x73) {
    Serial.print(" (MPU9255)");
  } else if (who == 0x70) {
    Serial.print(" (MPU6500)");
  } else if (who == 0x00 || who == 0xFF) {
    Serial.print(" (bus/device issue)");
  }
  Serial.println();
}

void tryLibraryInit(uint8_t addr) {
  uint8_t who = 0;
  if (!readRegister8(addr, kRegWhoAmI, &who)) {
    Serial.printf("addr 0x%02X: skipping library init (no register response)\n", addr);
    return;
  }

  if (who == 0x71 || who == 0x73) {
    MPU9250_WE imu(addr);
    const bool ok = imu.init();
    Serial.printf("addr 0x%02X: MPU9250_WE init=%s whoAmI=0x%02X\n", addr, ok ? "OK" : "FAIL", imu.whoAmI());
    return;
  }

  if (who == 0x70) {
    MPU6500_WE imu(addr);
    const bool ok = imu.init();
    Serial.printf("addr 0x%02X: MPU6500_WE init=%s whoAmI=0x%02X\n", addr, ok ? "OK" : "FAIL", imu.whoAmI());
    return;
  }

  Serial.printf("addr 0x%02X: unknown WHO_AM_I 0x%02X, no library init attempted\n", addr, who);
}

void printLineLevels(const char* tag) {
  if (tag == nullptr) {
    tag = "line-levels";
  }

  pinMode(kSdaPin, INPUT);
  pinMode(kSclPin, INPUT);
  delay(2);
  const int sda_raw = digitalRead(kSdaPin);
  const int scl_raw = digitalRead(kSclPin);

  pinMode(kSdaPin, INPUT_PULLUP);
  pinMode(kSclPin, INPUT_PULLUP);
  delay(2);
  const int sda_pullup = digitalRead(kSdaPin);
  const int scl_pullup = digitalRead(kSclPin);

  Serial.printf(
    "%s: SDA raw=%d pullup=%d | SCL raw=%d pullup=%d\n",
    tag,
    sda_raw,
    sda_pullup,
    scl_raw,
    scl_pullup
  );
}

ProbeResult runProbeAtClock(uint32_t clock_hz) {
  ProbeResult result;
  Wire.setClock(clock_hz);
  delay(2);

  Serial.printf("--- Probe at %lu Hz ---\n", static_cast<unsigned long>(clock_hz));
  result.any_devices = scanBus();
  result.ack_68 = pingAddress(0x68);
  result.ack_69 = pingAddress(0x69);
  printWhoAmI(0x68);
  printWhoAmI(0x69);
  tryLibraryInit(0x68);
  tryLibraryInit(0x69);
  return result;
}

void runProbe() {
  const ProbeResult fast = runProbeAtClock(kI2cClockHz);
  if (!fast.ack_68 && !fast.ack_69) {
    Serial.println("No ACK on 0x68/0x69 at 400kHz, retrying at 100kHz...");
    (void)runProbeAtClock(kI2cFallbackClockHz);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  const uint32_t start = millis();
  while (!Serial && (millis() - start) < 2000) {
    delay(10);
  }

  printLineLevels("Before Wire.begin");
  Wire.begin(kSdaPin, kSclPin);
  Wire.setClock(kI2cClockHz);
  printLineLevels("After Wire.begin");

  Serial.println();
  Serial.println("mpuWeTest (MPU9250_WE / MPU6500_WE)");
  Serial.printf(
    "Using I2C SDA=%u SCL=%u @ %lu Hz (fallback %lu Hz)\n",
    kSdaPin,
    kSclPin,
    static_cast<unsigned long>(kI2cClockHz),
    static_cast<unsigned long>(kI2cFallbackClockHz)
  );
  Serial.println("Probing 0x68 and 0x69 ...");

  runProbe();
  Serial.println("Done. Send 'p' + ENTER to probe again.");
}

void loop() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == 'p' || c == 'P') {
      Serial.println();
      Serial.println("Re-running probe...");
      runProbe();
      Serial.println("Done.");
    }
  }
}
