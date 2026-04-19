#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${WIFI_ENV_FILE:-$ROOT/mpu-wifi.env}"

if [[ ! -f "$ENV_FILE" ]]; then
  echo "Missing ${ENV_FILE}. Copy mpu-wifi.env.example -> mpu-wifi.env and edit." >&2
  exit 1
fi

# shellcheck source=/dev/null
set -a
source "$ENV_FILE"
set +a

: "${UNO_Q_SSH:?Set UNO_Q_SSH in mpu-wifi.env (e.g. arduino@192.168.2.1)}"
: "${WIFI_SSID:?Set WIFI_SSID in mpu-wifi.env}"
: "${WIFI_PSK:?Set WIFI_PSK in mpu-wifi.env}"

opts=(${UNO_Q_SSH_OPTS:-})

echo "Connecting UNO Q (${UNO_Q_SSH}) to Wi‑Fi SSID: ${WIFI_SSID}"

# Rescan helps right after boot; ignore failure on older nmcli.
ssh "${opts[@]}" "$UNO_Q_SSH" 'sudo nmcli device wifi rescan' 2>/dev/null || true
sleep 2

qssid=$(printf '%q' "$WIFI_SSID")
qpsk=$(printf '%q' "$WIFI_PSK")

ssh "${opts[@]}" "$UNO_Q_SSH" "sudo nmcli device wifi connect ${qssid} password ${qpsk}"

echo "Addresses on UNO Q:"
ssh "${opts[@]}" "$UNO_Q_SSH" 'hostname -I || true'
