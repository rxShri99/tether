#!/usr/bin/env bash
# Usage: tools/flash.sh a|b|hub [port]
# Builds with the right device identity (separate build dirs) and flashes.
set -euo pipefail

DEV="${1:?usage: flash.sh a|b|hub [port]}"
PORT="${2:-$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)}"

case "$DEV" in
  a)   ID=1;   ROLE=0; BUILD=build_a ;;
  b)   ID=2;   ROLE=0; BUILD=build_b ;;
  hub) ID=100; ROLE=1; BUILD=build_hub ;;
  *) echo "unknown device: $DEV (use a|b|hub)"; exit 1 ;;
esac

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJ="$SCRIPT_DIR/.."
source "$PROJ/../esp-idf/export.sh" >/dev/null

cd "$PROJ"
idf.py -B "$BUILD" -DTETHER_DEVICE_ID="$ID" -DTETHER_ROLE="$ROLE" build
idf.py -B "$BUILD" -p "$PORT" flash
echo "Flashed $DEV (id=$ID) to $PORT"
