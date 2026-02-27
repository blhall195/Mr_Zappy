# Mr Zappy — Cave Survey Device (C++ Firmware)

Handheld cave surveying instrument measuring azimuth, inclination, and laser distance.
Dual-MCU architecture communicating via UART.

## Build System

PlatformIO. Two independent projects — build/upload each separately.

- **Main board**: `Main board C++/platformio.ini` → `adafruit_feather_m4_can` (SAMD51), upload COM33
- **BLE board**: `DiscoX C++ BLE/platformio.ini` → `adafruit_itsybitsy_nrf52840`

## Architecture

### Main Board (Adafruit Feather M4 CAN — SAMD51)

Cooperative event loop in `main.cpp` — no RTOS. Polling-based with timing guards.

**Sensors** (all I2C):
- RM3100 magnetometer (custom driver, `rm3100.cpp`) — 400 cycle count
- ISM330DHCX IMU/accel (Adafruit LSM6DS library)
- MAX17048 battery gauge
- SH1107 128x128 OLED display

**Peripherals**:
- Egismos laser rangefinder via Serial1 (9600 baud, custom binary protocol)
- NeoPixel RGB LED (disco mode animations)
- 5 buttons (MEASURE, DISCO, CALIB, SHUTDOWN, FIRE) — active LOW, internal pull-ups
- QSPI flash with FAT filesystem for config, calibration, and pending readings

**Communication to BLE board**: Custom UART on SERCOM1 (pins D5/D6, 9600 baud) with DRDY handshake pulse on D12.

### BLE Board (ItsyBitsy nRF52840)

Three concurrent tasks:
1. **UART reader** — DRDY-gated state machine, parses `COMPASS:X,CLINO:Y,DIST:Z` lines
2. **SAP6 BLE protocol** — GATT service with sequence-bit ACK/retry, 17-byte leg data notifications
3. **Connection monitor** — toggles BLE_CONNECTED pin for main board

## Key Source Files

### Main Board (`Main board C++/`)

| File | Purpose |
|------|---------|
| `src/main.cpp` | setup(), loop(), all polling — the central orchestrator (~700 lines) |
| `include/config.h` | All pin definitions, timing constants, axis mappings, defaults |
| `include/device_context.h` | `DeviceContext`, `Config`, `Readings`, `SystemState` structs |
| `src/sensor_manager.cpp` | EMA filtering, circular EMA for azimuth, stability ring buffer |
| `src/calibration_mode.cpp` | On-device calibration UI (56-pt ellipsoid + 24-pt alignment) |
| `src/mag_cal/calibration.cpp` | Core calibration math — orientation matrix, angle extraction |
| `src/mag_cal/sensor.cpp` | Per-sensor: axis permutation, ellipsoid transform, RBF correction |
| `src/rm3100.cpp` | RM3100 magnetometer I2C driver |
| `src/ble_manager.cpp` | SERCOM1 UART to DiscoX, DRDY pulse, command parsing |
| `src/display_manager.cpp` | OLED rendering (compass, incline, distance, battery, alerts) |
| `src/laser_egismos.cpp` | Egismos binary protocol: on/off, measure, error handling |
| `src/config_manager.cpp` | QSPI flash FAT: load/save config.json, calibration.json, pending.txt |
| `src/menu_manager.cpp` | Settings menu (anomaly detection, laser timeout, etc.) |
| `src/fruity_menu.cpp` | Hierarchical menu UI framework |
| `src/disco_manager.cpp` | NeoPixel LED animations |
| `src/snake_game.cpp` | Easter egg: 16x16 snake game on OLED (hold CALIB 5s in menu) |

### BLE Board (`DiscoX C++ BLE/`)

| File | Purpose |
|------|---------|
| `src/main.cpp` | Setup + loop running 3 async tasks |
| `include/config.h` | Pin defs, BLE UUIDs, SAP6 command codes |
| `src/sap6_protocol.cpp` | GATT service, send queue (max 20), sequence-bit ACK/retry |
| `src/uart_handler.cpp` | DRDY-gated UART line parser, name change handling |
| `src/nvm_manager.cpp` | Device name persistence in internal flash |

## Data Flow

```
Sensors → SensorManager (calibrate → EMA → stability check)
       → DisplayManager (live compass/incline/distance)
       → BleManager → UART → DiscoX → BLE GATT → Phone app
```

**Measurement**: Press MEASURE → laser fires → wait for stability (azimuth + inclination within tolerance for 3 consecutive samples) → freeze display → send via BLE → wait for ACK.

## Key Data Structures (`device_context.h`)

- **SystemState**: `IDLE`, `TAKING_MEASUREMENT`, `MENU`
- **Readings**: azimuth, inclination, roll, distance, batteryLevel
- **Config**: tolerances (mag/grav/dip/stability/leg), EMA alpha, laser offset (0.14m), timeouts
- **DeviceContext**: central state — current state, readings, config, flags (laser, disco, BLE, quickShot)

## Calibration System (`mag_cal/`)

Ported from Python. Uses Eigen for linear algebra.

- **Ellipsoid fitting**: 56 points → hard-iron centre + soft-iron 3x3 transform
- **Alignment**: 24 points at 3 orientations → rotates transform to gravity reference
- **RBF non-linear correction**: Gaussian radial basis functions per magnetometer axis
- **Anomaly detection**: field strength ±2%, dip angle ±3° triggers warnings

Calibration JSON is embedded at compile-time in `main.cpp` as `CALIBRATION_JSON[]` and also persisted to QSPI flash.

Axis mappings: mag `"-X-Y-Z"`, accel `"-Y-X+Z"` (PCB orientation).

## Communication Protocols

### Main ↔ BLE UART (9600 baud)
- **Outbound**: `COMPASS:XXX.X,CLINO:YYY.Y,DIST:ZZZ.ZZ\n`, `ALIVE\n`, `NAME:...\n`
- **Inbound**: `ACK_RECEIVED\n`, command codes (`48`=stop cal, `49`=start cal, `52`=off, `54`/`55`=laser on/off, `56`=take shot)
- **Handshake**: DRDY pin pulsed HIGH before sending data

### SAP6 BLE GATT
- 17-byte leg data: `[seqBit][az float][inc float][roll float][dist float]`
- Sequence bit toggles 0x55/0x56 for reliable delivery with ACK/retry on 5s timeout

## Power Management
- Auto-shutdown: 30 min idle (configurable)
- Laser timeout: 2 min (configurable)
- Battery: check every 30s, shutdown at ≤5%
- LTC2952 shutdown controller on pin A2

## Important Libraries
- **ArduinoEigen** — matrix math for calibration
- **ArduinoJson** — config/calibration persistence
- **Adafruit SH110X/GFX** — OLED display
- **Adafruit LSM6DS** — IMU driver
- **Adafruit SPIFlash + SdFat** — QSPI flash filesystem
- **Bluefruit** (BLE board) — nRF52840 BLE stack

## Current Status
- Calibration working, accurate to within 1 degree (commit 8682252)
- Next: adjust sensor fusion weightings for improved accuracy
