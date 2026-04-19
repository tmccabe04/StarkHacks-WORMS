FQBN = esp32:esp32:esp32s3
PORT := $(shell arduino-cli board list | grep "esp32" | awk '{print $$1}' | head -n 1)

# arduino-cli upload -p /dev/cu.usbmodem1101 --fqbn esp32:esp32:esp32s3 neopixelTest
clean: blank/*
	arduino-cli compile --fqbn $(FQBN) blank
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) blank

neopixelTest: neopixelTest/*
	arduino-cli compile --fqbn $(FQBN) neopixelTest
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) neopixelTest

mpuTest: mpuTest/*
	arduino-cli compile --fqbn $(FQBN) mpuTest --build-property "build.extra_flags=-DARDUINO_USB_CDC_ON_BOOT=1"
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) mpuTest

motionControl: motionControl/*
	arduino-cli compile --fqbn $(FQBN) motionControl --build-property "build.extra_flags=-DARDUINO_USB_CDC_ON_BOOT=1"
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) motionControl

motorTest: motorTest/*
	arduino-cli compile --fqbn $(FQBN) motorTest --build-property "build.extra_flags=-DARDUINO_USB_CDC_ON_BOOT=1"
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) motorTest
