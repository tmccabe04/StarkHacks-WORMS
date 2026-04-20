FQBN = esp32:esp32:esp32s3
PORT ?= $(shell arduino-cli board list | awk '/^\/dev\/(cu|tty)\.(usb|wchusb|SLAB|ttyUSB|ttyACM)/ {print $$1; exit}')
UPLOAD_SPEED ?= 115200

.PHONY: clean neopixelTest mpuTest motionControl motorTest mpuWeTest openLoopControl

# arduino-cli upload -p /dev/cu.usbmodem1101 --fqbn esp32:esp32:esp32s3 neopixelTest
clean: blank/*
	arduino-cli compile --fqbn $(FQBN) blank
	@if [ -z "$(PORT)" ]; then echo "No serial port detected. Run: arduino-cli board list"; echo "Then rerun with: make clean PORT=/dev/cu.usbmodemXXXX"; exit 1; fi
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) blank

neopixelTest: neopixelTest/*
	arduino-cli compile --fqbn $(FQBN) neopixelTest
	@if [ -z "$(PORT)" ]; then echo "No serial port detected. Run: arduino-cli board list"; echo "Then rerun with: make neopixelTest PORT=/dev/cu.usbmodemXXXX"; exit 1; fi
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) neopixelTest

mpuTest: mpuTest/*
	arduino-cli compile --fqbn $(FQBN) mpuTest --build-property "build.extra_flags=-DARDUINO_USB_CDC_ON_BOOT=1"
	@if [ -z "$(PORT)" ]; then echo "No serial port detected. Run: arduino-cli board list"; echo "Then rerun with: make mpuTest PORT=/dev/cu.usbmodemXXXX"; exit 1; fi
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) mpuTest --upload-property upload.speed=$(UPLOAD_SPEED)

motionControl: motionControl/*
	arduino-cli compile --fqbn $(FQBN) motionControl --build-property "build.extra_flags=-DARDUINO_USB_CDC_ON_BOOT=1"
	@if [ -z "$(PORT)" ]; then echo "No serial port detected. Run: arduino-cli board list"; echo "Then rerun with: make motionControl PORT=/dev/cu.usbmodemXXXX"; exit 1; fi
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) motionControl --upload-property upload.speed=$(UPLOAD_SPEED)

motorTest: motorTest/*
	arduino-cli compile --fqbn $(FQBN) motorTest --build-property "build.extra_flags=-DARDUINO_USB_CDC_ON_BOOT=1"
	@if [ -z "$(PORT)" ]; then echo "No serial port detected. Run: arduino-cli board list"; echo "Then rerun with: make motorTest PORT=/dev/cu.usbmodemXXXX"; exit 1; fi
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) motorTest --upload-property upload.speed=$(UPLOAD_SPEED)

mpuWeTest: mpuWeTest/*
	arduino-cli compile --fqbn $(FQBN) mpuWeTest --build-property "build.extra_flags=-DARDUINO_USB_CDC_ON_BOOT=1"
	@if [ -z "$(PORT)" ]; then echo "No serial port detected. Run: arduino-cli board list"; echo "Then rerun with: make mpuWeTest PORT=/dev/cu.usbmodemXXXX"; exit 1; fi
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) mpuWeTest --upload-property upload.speed=$(UPLOAD_SPEED)

openLoopControl: openLoopControl/*
	arduino-cli compile --fqbn $(FQBN) openLoopControl --build-property "build.extra_flags=-DARDUINO_USB_CDC_ON_BOOT=1"
	@if [ -z "$(PORT)" ]; then echo "No serial port detected. Run: arduino-cli board list"; echo "Then rerun with: make openLoopControl PORT=/dev/cu.usbmodemXXXX"; exit 1; fi
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) openLoopControl --upload-property upload.speed=$(UPLOAD_SPEED)
