#!/usr/bin/env bash
set -euo pipefail

REPO="$(git rev-parse --show-toplevel)"
MAIN_DIR="$REPO/C++/Main board C++"
BLE_DIR="$REPO/C++/DiscoX C++ BLE"
UF2CONV="$REPO/tools/uf2conv.py"
BUILD="$REPO/build"

build_main() {
    echo "=== Building main board ==="
    pio run --project-dir "$MAIN_DIR" -e adafruit_feather_m4_can
    mkdir -p "$BUILD"
    python3 "$UF2CONV" \
        "$MAIN_DIR/.pio/build/adafruit_feather_m4_can/firmware.bin" \
        --base 0x4000 --family 0x55114460 \
        --output "$BUILD/main.uf2"
    echo "-> $BUILD/main.uf2"
}

build_ble() {
    echo "=== Building BLE board ==="
    pio run --project-dir "$BLE_DIR"
    mkdir -p "$BUILD"
    python3 "$UF2CONV" \
        "$BLE_DIR/.pio/build/adafruit_itsybitsy_nrf52840/firmware.hex" \
        --family 0xada52840 --convert \
        --output "$BUILD/ble.uf2"
    echo "-> $BUILD/ble.uf2"
}

case "${1:-main}" in
    main) build_main ;;
    ble)  build_ble ;;
    all)  build_main; build_ble ;;
    *)    echo "Usage: $0 [main|ble|all]"; exit 1 ;;
esac
