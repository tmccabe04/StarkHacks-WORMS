# Makefile for Brain (Arduino UNO Q — Zephyr MCU sketch via arduino-cli).
#
# Typical flow:
#   make install-lib     # once per machine
#   cp mpu-wifi.env.example mpu-wifi.env && edit   # once (SSH + Wi‑Fi creds)
#   make flash         # upload sketch over USB
#   make wifi-push     # join Linux (MPU) to Wi‑Fi over SSH (optional if already on Wi‑Fi)

FQBN = arduino:zephyr:unoq
DETECT_PORT := ../scripts/detect-arduino-port.py
AUTO_UNO_PORT := $(shell python3 $(DETECT_PORT) "$(FQBN)" 2>/dev/null)
PORT ?= $(AUTO_UNO_PORT)

SRC = brain.ino

.PHONY: all compile upload flash install-lib wifi-push deploy help clean

all: compile

help:
	@echo "Targets:"
	@echo "  make install-lib   Install Arduino_RouterBridge (Library Manager equivalent)"
	@echo "  make compile       Build sketch for UNO Q"
	@echo "  make flash         compile + upload over USB (needs UNO Q port)"
	@echo "  make wifi-push     SSH to MPU: nmcli join Wi‑Fi (needs mpu-wifi.env)"
	@echo "  make deploy        flash then wifi-push"

deploy: flash wifi-push

install-lib:
	arduino-cli lib install "Arduino_RouterBridge"

compile: $(SRC)
	arduino-cli compile --fqbn $(FQBN) .

upload: compile
	@test -n "$(PORT)" || (echo "No UNO Q USB port found. Plug the board or run: PORT=/dev/cu.usbmodemXXXX make flash" >&2 && exit 1)
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

flash: upload

wifi-push:
	@chmod +x scripts/mpu-wifi-nmcli.sh
	@./scripts/mpu-wifi-nmcli.sh

clean:
	rm -rf build/
	rm -f brain_app
