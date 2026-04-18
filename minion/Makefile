FQBN = esp32:esp32:esp32s3
PORT := $(shell arduino-cli board list | grep "esp32" | awk '{print $$1}' | head -n 1)


# arduino-cli upload -p /dev/cu.usbmodem1101 --fqbn esp32:esp32:esp32s3 neopixelTest
clean: blank/*
	arduino-cli compile --fqbn $(FQBN) blank
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) blank
