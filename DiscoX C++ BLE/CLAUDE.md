# DiscoX BLE Cave Surveying Bridge

## Project Overview
C++ BLE bridge for cave surveying. Runs on **Adafruit ItsyBitsy nRF52840 Express**. Ported from CircuitPython SAP6 protocol. Bridges a master survey instrument (UART) to a phone/tablet (BLE).

## Build & Flash
- **Framework**: PlatformIO + Arduino + Adafruit Bluefruit nRF52
- **Board**: `adafruit_itsybitsy_nrf52840`
- **Upload port**: COM11 (may shift to COM17 during DFU upload)
- **Serial monitor**: 115200 baud on COM11
- **Important**: Close serial monitor before flashing (COM port lock causes PermissionError)

```bash
# Build and flash
platformio run --target upload --upload-port COM11 -e adafruit_itsybitsy_nrf52840

# Serial monitor
platformio device monitor --port COM11 --baud 115200
```

## Architecture

### Source Files
- `src/main.cpp` — Setup + main loop with 4 tasks: UART poll, BLE protocol poll, connection monitoring, dummy readings
- `src/sap6_protocol.cpp` — SAP6 BLE GATT service with reliable delivery (sequence-bit ACK/retry). 4 characteristics: service, protocol name ("SAP6"), command (write), leg data (read+notify, 17 bytes: 1 byte seq + 4 floats)
- `src/uart_handler.cpp` — DRDY-gated UART state machine. Parses `Compass:XXX,Clino:YYY,Distance:ZZZ` lines, `ALIVE` keepalives, `NAME:` commands
- `src/nvm_manager.cpp` — Flash-backed device name storage using LittleFS
- `include/config.h` — Pins, UUIDs, timing constants, command bytes

### Pin Assignments
- D7 (PIN_DRDY) — Input, pull-down. Master signals data ready
- D5 (PIN_BLE_CONNECTED) — Output. HIGH when BLE connected
- A2 (PIN_LZR_POWER) — Output. HIGH = laser power on

### BLE UUIDs (128-bit, base: 137c4435-8a64-4bcb-93f1-3792c6bdc9XX)
- Service: ...c965
- Protocol Name: ...c966
- Command: ...c967
- Leg Data: ...c968

## BLE Long Range (Coded PHY) Setup

### What was done
Three changes were needed to enable BLE Coded PHY (Long Range, S=8, 125 kbps, ~4x range):

1. **Increased event length** in `main.cpp` setup():
   ```cpp
   Bluefruit.configPrphConn(BLE_GATT_ATT_MTU_DEFAULT, 24, BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
   ```
   Default event length of 3 (3.75 ms) is too short for Coded PHY packets. Set to 24 (30 ms). Without this, `sd_ble_gap_phy_update()` returns `0x0013` (NRF_ERROR_RESOURCES = "event length too short for requested PHY").

2. **Patched Adafruit Bluefruit library** `BLEConnection.cpp` line 392:
   ```cpp
   // Was: ble_gap_phys_t phy = { BLE_GAP_PHY_AUTO, BLE_GAP_PHY_AUTO };
   ble_gap_phys_t phy = { BLE_GAP_PHY_CODED, BLE_GAP_PHY_CODED };
   ```
   Location: `~/.platformio/packages/framework-arduinoadafruitnrf52/libraries/Bluefruit52Lib/src/BLEConnection.cpp`
   The library's `BLE_GAP_EVT_PHY_UPDATE_REQUEST` handler auto-responds to PHY negotiations. Without this patch it responds with AUTO (which defaults to 1 Mbps).

3. **PHY request with retry in loop()**: The SoftDevice is busy right after connection, so `sd_ble_gap_phy_update()` is retried every 500 ms until accepted. Can't call it from the connect callback (returns 0x0013).

### Connection TX Power
`Bluefruit.setTxPower(+8)` only sets **advertising** TX power (stores value internally, doesn't call SoftDevice). For connection data packets, must call:
```cpp
conn->setTxPower(BLE_TX_POWER);  // BLEConnection::setTxPower calls sd_ble_gap_tx_power_set
```
Without this, data packets transmit at default 0 dBm instead of +8 dBm.

### Verification
Serial monitor shows on successful connection:
```
BLE Connected
PHY update request: OK
Connection TX power set to +8 dBm
Active PHY: Coded
```
If phone doesn't support Coded PHY, it gracefully falls back to 1 Mbps.

### Phone Compatibility
- Android (2019+, BLE 5.0): Generally supports Coded PHY. Tested with OnePlus Nord 3 (Dimensity 9000).
- iOS: Does NOT support Coded PHY.
- Use **nRF Connect** app on Android for testing.

## Dummy Reading Mode
Currently sends randomized dummy survey readings every 3 seconds when connected (for testing). Remove Task 4 in loop() for production use.

## Key Gotchas
- SoftDevice S140 v6.1.1 — supports Coded PHY but Adafruit library doesn't expose it properly
- The library patch is in the global PlatformIO packages dir — will be overwritten if framework is updated
- After DFU upload, board may re-enumerate on different COM port (COM11 -> COM17 -> back to COM11)
- UART baud to master instrument: 9600
- ACK timeout: 5 seconds
- Main loop interval: 10 ms
