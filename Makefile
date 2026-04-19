# WORMS — one entry point for setup, builds, and flashing.
# Run `make` or `make help` first.

.DEFAULT_GOAL := help

.PHONY: help setup compile compile-brain compile-minion \
	flash-brain flash-minion flash-minion-stubless flash-esp32 deploy-brain wifi-brain

help:
	@echo "WORMS — common targets"
	@echo ""
	@echo "  make setup          Install Arduino cores + RouterBridge; create env templates if missing"
	@echo "  make compile        Build brain (UNO Q) + minion (ESP32) without uploading"
	@echo "  make flash-brain    Build + upload brain to UNO Q (USB; auto port for UNO Q FQBN)"
	@echo "  make flash-minion   Build + upload minion to ESP32 (USB; auto port for FQBN in local.env)"
	@echo "  make flash-minion-stubless   Same, with ESP32_NO_STUB=1 (stub RAM checksum errors)"
	@echo "  make flash-esp32    Same as flash-minion (ESP32 is not the UNO Q brain)"
	@echo "  make deploy-brain   flash-brain then push Wi‑Fi to UNO Q Linux (needs brain/mpu-wifi.env)"
	@echo "  make wifi-brain     Only Wi‑Fi push (brain/mpu-wifi.env)"
	@echo ""
	@echo "First time: make setup  →  edit minion/local.env and brain/mpu-wifi.env  →  flash each board."

setup:
	@bash scripts/setup.sh

compile: compile-brain compile-minion

compile-brain:
	@$(MAKE) -C brain compile

compile-minion:
	@test -f minion/local.env || (echo "Missing minion/local.env — run: make setup" >&2 && exit 1)
	@$(MAKE) -C minion compile

flash-brain:
	@$(MAKE) -C brain flash

flash-minion:
	@test -f minion/local.env || (echo "Missing minion/local.env — run: make setup" >&2 && exit 1)
	@$(MAKE) -C minion minion

flash-esp32: flash-minion

# Same as flash-minion but skips esptool RAM stub (helps some USB checksum failures)
flash-minion-stubless:
	@test -f minion/local.env || (echo "Missing minion/local.env — run: make setup" >&2 && exit 1)
	@$(MAKE) -C minion minion ESP32_NO_STUB=1

deploy-brain:
	@$(MAKE) -C brain deploy

wifi-brain:
	@$(MAKE) -C brain wifi-push
