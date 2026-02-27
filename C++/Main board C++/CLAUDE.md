# Mr_Zappy Main Board C++ Port

## Overview
Cave surveying device main board. Converting from CircuitPython to C++ (PlatformIO + Arduino).
- **Hardware**: Adafruit Feather M4 CAN (ATSAMD51, Cortex-M4 @ 120MHz, 256KB RAM, 512KB flash, HW FPU)
- **Python source**: `../Main board/` (CircuitPython, ~4000+ lines across 15+ modules)
- **BLE board**: Already converted — see `../DiscoX C++ BLE/` (separate board, communicates via UART)
- **Full plan**: See `CONVERSION_PLAN.md` in this directory

## Architecture Decisions
- **Framework**: PlatformIO + Arduino (matches DiscoX)
- **Concurrency**: Cooperative polling in `loop()` with state machines (no RTOS)
- **Linear algebra**: ArduinoEigen (fixed-size matrices for real-time path, dynamic allowed for calibration fitting)
- **JSON**: ArduinoJson for config/calibration persistence
- **Display**: Adafruit SH1107 + Adafruit GFX
- **NeoPixel**: Adafruit NeoPixel library

## Module Status

| Module | Status | Session | Notes |
|--------|--------|---------|-------|
| CONVERSION_PLAN.md | Done | 0 | Plan document in repo |
| CLAUDE.md | Done | 0 | This file |
| platformio.ini | Done | 0 | Build config, TinyUSB, all lib_deps |
| config.h | Done | 0 | Pin defs, constants, defaults, timing, filter params |
| device_context.h | Done | 0 | SystemState enum, Readings, Config, DeviceContext structs |
| main.cpp | **Updated** | 14 | Session 14: Battery fix, state machine integration |
| button_manager | **Done** | 1 | 5 buttons, hand-rolled debounce |
| rm3100 | **Done** | 2 | Custom I2C driver, DRDY pin, cycle count 400 |
| ISM330DHCX | **Done** | 2 | Adafruit LSM6DS, 104Hz, 4G/250dps |
| MAX17048 | **Fixed** | 14 | Subclassed to skip reset() — preserves ModelGauge state across reboots |
| display_manager | **Done** | 3 | SH1107 128x128 OLED, GFX-based, 4Hz refresh |
| laser_egismos | **Done** | 4 | Egismos laser UART protocol, 9600 baud on Serial1 |
| laser_ldj100 | Skipped | — | Not needed (device uses Egismos) |
| ble_manager | **Done** | 5 | UART bridge to DiscoX (SERCOM1) |
| mag_cal (calibration math) | **Done** | 6 | ArduinoEigenDense, apply path + anomaly detection. Fitting stubs for Session 11 |
| calibrate_roll | Deferred | 11 | Integrated into Calibration class, stub for now |
| sensor_manager | **Done** | 7 | Complementary gravity filter + EMA smoothing, stability detection |
| config_manager | **Done** | 8 | QSPI flash FAT filesystem, config/calibration/pending/flags persistence |
| disco_manager | **Done** | 9 | NeoPixel LED: static colors, disco rainbow, purple pulse |
| display_manager | **Updated** | 10 | Added getDisplay() accessor for menu rendering |
| fruity_menu | **Done** | 10 | Menu UI framework (direct port from Python) |
| menu_manager | **Done** | 10 | Mr_Zappy settings menu with callbacks |
| calibration_mode | **Done** | 11 | Full calibration workflow (ellipsoid + alignment) |
| main state machine | **Done** | 12 | Full integration: cooperative polling, measurement workflow, BLE, power mgmt |
| main debugging | **Done** | 13 | QSPI crash fixed via RAM buffer + deferred writes |
| polish & edge cases | **In progress** | 14 | Battery fix done. Remaining: debug prints cleanup, buzzer sequences, full BLE/timeout/leg test |
| calibration_mode display | **Fixed** | 15 | All calibration screens now draw directly via getDisplay() instead of measurement-layout refresh(). Button double-update bug fixed. |
| calibration_mode testing | **In progress** | 15 | Ellipsoid collection hangs during point recording — likely QSPI flash crash (same root cause as Session 13). Needs RAM-buffer approach for calibration data or deferred flash writes. |
| calibration double precision | **Done** | 17 | All fitting math (fitEllipsoid, alignAlongAxis, alignSensorRoll, getLstsqNonLinearParams, accuracy, etc.) upgraded to double-precision Eigen internally. Runtime path stays float. Flash +32KB (243KB/508KB). |

## Pin Mappings (Feather M4 CAN)

| Function | Pin | Type | Notes |
|----------|-----|------|-------|
| Button 1 (Measure) | A3 | INPUT, PULL_UP | Active LOW |
| Button 2 (Disco) | A4 | INPUT, PULL_UP | Active LOW |
| Button 3 (Calibration) | A0 | INPUT, PULL_UP | Active LOW |
| Button 4 (Shutdown) | A1 | INPUT, PULL_UP | Active LOW |
| Fire Button | D4 | INPUT, PULL_UP | Active LOW |
| Power Control | A2 | OUTPUT | LTC2952, LOW = shutdown |
| BLE Status | D11 | INPUT, PULL_DOWN | HIGH = BLE connected |
| BLE UART TX | D5 | UART TX | 9600 baud to DiscoX |
| BLE UART RX | D6 | UART RX | 9600 baud from DiscoX |
| BLE DRDY | D12 | OUTPUT | HIGH = data ready for BLE board |
| Mag DRDY | D9 | INPUT | RM3100 data ready |
| Config Pin | SCK | INPUT, PULL_UP | Mode selector |
| I2C SDA | SDA | I2C | Sensors + Display |
| I2C SCL | SCL | I2C | Sensors + Display |
| Laser TX | TX | UART | 9600 baud (Egismos) or 115200 (LDJ100) |
| Laser RX | RX | UART | Laser response |
| NeoPixel | NEOPIXEL | WS2812 | 1 pixel |

## I2C Addresses

| Device | Address | Driver |
|--------|---------|--------|
| RM3100 magnetometer | 0x20 | Custom (rm3100.h) |
| ISM330DHCX accel/gyro | 0x6A (default) | Adafruit LSM6DS |
| MAX17048 battery | 0x36 | Adafruit MAX1704X |
| SH1107 OLED | 0x3D | Adafruit SH1107 |

## Default Configuration (from settings.toml)

```
mag_tolerance = 10.0         // Magnetic anomaly (degrees)
grav_tolerance = 10.0        // Gravity anomaly (degrees)
dip_tolerance = 10.0         // Dip anomaly (degrees)
anomaly_detection = true
auto_shutdown_timeout = 1800 // 30 min (seconds)
laser_timeout = 120          // 2 min (seconds)
laser_distance_offset = 0.14 // 14cm device length
leg_angle_tolerance = 1.7    // degrees
leg_distance_tolerance = 0.05 // meters
stability_tolerance = 0.4    // degrees
stability_buffer_length = 3
EMA_alpha = 0.5
```

## Calibration Axes

```
mag_axes = "-X-Y-Z"    // RM3100 axis mapping
grav_axes = "-Y-X+Z"   // ISM330DHCX axis mapping
```

## BLE Protocol (SAP6)

Messages from main board to DiscoX (UART @ 9600):
- Survey data: `COMPASS:XXX.X,CLINO:YYY.Y,DIST:ZZ.ZZ\n`
- Keep-alive: `ALIVE\n`
- Name change: `NAME:NewDeviceName\n`

Commands from DiscoX to main board:
- `0x55` ACK0, `0x56` ACK1
- `0x30` STOP_CAL, `0x31` START_CAL
- `0x34` DEVICE_OFF
- `0x36` LASER_ON, `0x37` LASER_OFF
- `0x38` TAKE_SHOT

## Known Issues / TODOs
- COM port changes after upload (CircuitPython = COM18/VID:80CE, C++ TinyUSB = COM21/VID:80CD). Use COM21 for monitor/upload.
- Boot messages missed on serial monitor due to TinyUSB CDC re-enumeration delay (~3s). Not a real problem — just a monitoring artifact.
- **MAX17048 battery SOC (FIXED Session 14)**: Stock Adafruit library `begin()` calls `reset()` which wipes the ModelGauge learned state, causing SOC to only report 0% or 100% (voltage-only first guess). Fixed by subclassing as `MAX17048_Persistent` — skips reset, calls `wake()` to exit hibernation. IC still PORs naturally on battery removal.

## Testing Notes
- Session 0: Build succeeds. Upload succeeds via sam-ba/bossac. Serial heartbeat confirmed at 115200 baud.
- Session 1: Build succeeds. Button events print on press/release. Heartbeat shows held buttons.
- Session 2: All 3 I2C sensors reading correctly. MAG ~(39.5, 12.3, -46.7) µT, ACCEL ~(1.06, -0.28, 9.96) m/s², GYRO ~(0.004, -0.012, 0.000) rad/s. BAT 0% expected (USB power, no LiPo).
- Session 2 — RAM: 3.1% (6,100 / 196,608 bytes), Flash: 8.1% (41,028 / 507,904 bytes)
- Session 3: Hardware verified — OLED showing live heading/tilt/battery at 4Hz. Tilt axis misaligned (expected — raw atan2 without axis remap, fixed in Session 7).
- Session 3 — RAM: 3.3% (6,396 / 196,608 bytes), Flash: 10.0% (50,768 / 507,904 bytes)
- Session 4: Hardware verified — Egismos laser driver. Measure/Fire btn takes shot, Disco btn toggles laser.
- Session 4 — RAM: 3.3% (6,436 / 196,608 bytes), Flash: 10.3% (52,180 / 507,904 bytes)
- Session 5: Build succeeds. BLE UART on SERCOM1 (D5 TX / D6 RX). Test harness: MEASURE=send survey data, FIRE=keep-alive, CALIB=set name, DISCO=laser toggle. Incoming commands parsed and printed.
- Session 5 — RAM: 3.7% (7,328 / 196,608 bytes), Flash: 10.7% (54,492 / 507,904 bytes)
- Session 6: Build succeeds. Calibration math library (mag_cal/) with ArduinoEigenDense. JSON calibration loaded from embedded string. Test harness: MEASURE=debug dump (raw/cal vectors, field strengths, dip, orientation matrix, angles). Display shows calibrated azimuth/inclination at 4Hz. Anomaly detection active.
- Session 6 — RAM: 4.0% (7,892 / 196,608 bytes), Flash: 14.5% (73,472 / 507,904 bytes)
- Session 7: Initially implemented Madgwick AHRS 9-DOF quaternion filter. Hardware tested — had ~0.3°/s azimuth drift when stationary (gyro bias correction too slow for near-static use). Replaced with Python-style approach: complementary gravity filter (gyro predicts, accel corrects with dynamic alpha based on motion), direct angle computation via calibration orientation matrix, EMA smoothing on output angles. This matches the Python pipeline: device is held steady, so trust mag/accel for absolute orientation and use gyro only for gravity smoothing. Online gyro bias learning when stationary. Circular EMA for azimuth wraparound handling.
- Session 7 — RAM: 4.1% (8,020 / 196,608 bytes), Flash: 14.8% (75,352 / 507,904 bytes)
- Session 8: Code complete. ConfigManager: QSPI flash + FAT filesystem. Settings, calibration, pending readings, and boot flags. Test harness: MEASURE=dump config, FIRE=toggle anomaly+save, CALIB=add pending, DISCO=clear pending. Reboot test: toggle setting, reboot, verify persistence.
- Session 8 — RAM: 4.7% (9,212 / 196,608 bytes), Flash: 18.1% (91,836 / 507,904 bytes)
- Session 9: Code complete. DiscoManager: NeoPixel LED effects via non-blocking state machine. Static colors (red/green/blue/white), disco rainbow (HSV cycling with shake-triggered wild mode at full brightness, auto-return to dim after 5s timeout), purple pulse (sine-wave brightness modulation at ~33Hz). Test harness: MEASURE=cycle static colors, FIRE=toggle disco, CALIB=toggle purple pulse, DISCO=off.
- Session 9 — RAM: 4.7% (9,268 / 196,608 bytes), Flash: 17.8% (90,220 / 507,904 bytes)
- Session 12: Code complete. Uploaded and partially tested. Boot sequence works: all sensors OK, calibration loaded, laser fires. MEASURE button triggers measurement workflow correctly — red LED, stability wait, laser shot. Laser BAD_READING error handled correctly (no target surface). **Fix applied**: low battery check now skips 0% (USB-only power, no LiPo reads 0% on MAX17048). **Fix applied (Session 12b)**: Egismos laser module crashes on rapid-fire UART commands — added 20ms+ delays between all consecutive laser UART calls (boot beep, measure/fire re-enable, measurement entry beep, measurement success beep, leg-complete triple buzz, alert error beep). Also simplified measurement success sequence from 5 laser commands to 2 (buzzer on/off only — laser is already on from measurement). Still needs hardware retest after laser timing fix: (1) successful measurement with target surface; (2) DISCO toggle + hold; (3) CALIB hold → menu; (4) BLE send/flush; (5) timeouts; (6) leg completion.
- Session 12 — RAM: 5.5% (10,720 / 196,608 bytes), Flash: 41.5% (210,584 / 507,904 bytes)

## Build Notes
- Requires `-DUSE_TINYUSB` build flag for Adafruit SAMD boards
- PlatformIO's packaged framework (1.10716.0) includes feather_m4_can variant — do NOT use git source override (breaks dependency resolution)
- `PIN_NEOPIXEL` is a board-defined macro — do not redefine as constexpr
- Use `<ArduinoEigenDense.h>` NOT `<ArduinoEigen.h>` — the full include pulls in SparseLU which has macro name conflicts (BM, SM, RK, RN) with SAMD register defines
- Removed `-DEIGEN_NO_MALLOC` from build_flags — calibration fitting (Session 11) needs dynamic Eigen matrices. Real-time path uses only fixed-size types (Matrix3f, Vector3f)

## Session Log
- **Session 0** (complete): Project scaffold, pin defs, build config. Build + upload + serial verified on hardware.
- **Session 1** (complete): ButtonManager — hand-rolled debounce for 5 buttons. Serial test harness prints PRESSED/RELEASED events + heartbeat shows held buttons.
- **Session 2** (complete): I2C sensor drivers — custom RM3100 mag driver (cycle count 400, DRDY on D9), ISM330DHCX via Adafruit LSM6DS (104Hz, 4G/250dps), MAX17048 battery gauge. All verified on hardware at 10Hz.
- **Session 3** (complete): Display manager — SH1107 128x128 OLED via Adafruit SH110X + GFX. Layout: BT label (top-left, size 2), battery bar (top-right), distance/azimuth/inclination (size 3). Display refreshes at 4Hz with rough heading (atan2 from raw mag) and tilt (atan2 from raw accel). Also: blankScreen(), showStartingMenu(), showInitialisingMessage(). I2C kept at 400kHz before and after display transactions. Hardware verified — tilt axis misaligned as expected (raw values, corrected in Session 7). Build: RAM 3.3% (6396B), Flash 10.0% (50768B).
- **Session 4** (complete): Egismos laser driver — UART protocol at 9600 baud on Serial1. LaserError enum for error handling (no exceptions). Frame: [0xAA][addr][cmd][data...][chk][0xA8]. Commands: setLaser(), setBuzzer(), stopMeasuring(), measure(). Test harness: MEASURE btn toggles laser, FIRE btn takes single measurement. Distance shown on OLED and serial. LDJ100 laser skipped (not used). Build: RAM 3.3% (6436B), Flash 10.3% (52180B).
- **Session 5** (complete): BLE manager — UART bridge to DiscoX board. Created hardware UART on SERCOM1 (repurposed from unused SPI) with D5/PA16 as TX (PAD[0]) and D6/PA18 as RX (PAD[2]) at 9600 baud. DRDY pin (D12) pulsed HIGH around each send. BleCommand enum for inbound commands from DiscoX (ACK_RECEIVED, STOP_CAL, START_CAL, DEVICE_OFF, LASER_ON, LASER_OFF, TAKE_SHOT). Outbound: sendSurveyData() formats "COMPASS:x,CLINO:y,DIST:z\n", sendKeepAlive(), setName(). BLE status pin (D11) read via isConnected(). Test harness: MEASURE=survey data, FIRE=keep-alive, CALIB=name change, DISCO=laser toggle. Build: RAM 3.7% (7328B), Flash 10.7% (54492B).
- **Session 6** (code complete — needs hardware test): Calibration math library — ported Python mag_cal/ to C++ with ArduinoEigenDense. Files: utils.h (inline normalize), axes.h/.cpp (axis permutation), rbf.h/.cpp (Gaussian RBF), sensor.h/.cpp (Sensor class: apply, fromJson, anomaly), calibration.h/.cpp (Calibration class: getAngles, getOrientationMatrix, matrixToAngles, getDip, checkAnomaly, fromJson). Embedded calibration_dict.json as PROGMEM string (~600B). Test harness: MEASURE btn prints full debug dump (raw vectors, calibrated vectors, field strengths, dip, 3x3 orientation matrix, angles). OLED shows calibrated azimuth/inclination at 4Hz. Fitting functions (fitEllipsoid, fitToAxis, fitNonLinearQuick, alignSensorRoll) declared as stubs — implemented in Session 11. Key fix: must use ArduinoEigenDense.h (not ArduinoEigen.h) to avoid SparseLU macro conflicts with SAMD. Build: RAM 4.0% (7892B), Flash 14.5% (73472B).
- **Session 7** (code complete — needs hardware test): Sensor fusion — replaced Madgwick AHRS (had ~0.3°/s azimuth drift when stationary) with Python-style complementary gravity filter + EMA smoothing. SensorManager class: (1) complementary filter on gravity vector — gyro predicts rotation via cross-product integration, raw accel corrects with dynamic alpha (stationary: α=0.03 trusts gyro, moving: α=0.8 trusts accel); (2) online gyro bias learning when stationary (EMA α=0.001); (3) direct angle computation via Calibration::getAngles() (axis remap, ellipsoid correction, orientation matrix, ZXY Euler); (4) EMA smoothing on output azimuth/inclination with circular wraparound handling; (5) stability ring buffer unchanged. This matches the Python pipeline — device is held steady so mag/accel are authoritative, gyro only smooths gravity. Sensor read rate stays 50Hz. Build: RAM 4.1% (8020B), Flash 14.8% (75352B).
- **Session 8** (code complete — needs hardware test): Config & NVM persistence — ConfigManager class using Adafruit SPIFlash + SdFat (FAT filesystem on 2MB QSPI flash). Provides: (1) settings persistence as /config.json — all Config struct fields serialized with ArduinoJson, snake_case keys, defaults on missing keys; (2) calibration data as /calibration.json — raw JSON read/write, initCalibration() now tries flash first, falls back to PROGMEM; (3) pending readings queue as /pending.txt — append "az,inc,dist\n" lines, count, flush with callback, clear; (4) boot-mode flag files in /flags/ directory (calibration, menu, snake). FAT chosen over LittleFS because Adafruit_LittleFS is nRF52-only. Flash/FS objects are file-scope statics (transport must not be copied). SdFat FILE_WRITE opens at pos 0 so seekEnd() needed for append. Sensor fusion init now uses ctx.config values (may differ from compiled defaults after flash load). Test harness: MEASURE=dump config, FIRE=toggle anomaly+save, CALIB=add pending reading, DISCO=clear pending. Build: RAM 4.7% (9212B), Flash 18.1% (91836B).
- **Session 9** (code complete — needs hardware test): Disco Manager — NeoPixel LED effects as non-blocking state machine (replaces Python asyncio tasks). DiscoEffect enum: OFF, STATIC, DISCO, PURPLE_PULSE. Static colors: setRed/Green/Blue/White with software brightness scaling (default 30%). Disco rainbow: HSV hue cycling (20° steps at 5Hz), shake detection via raw accel > 11 m/s² triggers wild mode (random hue, full brightness, 10Hz), auto-returns to dim after 5s. Purple pulse: sinusoidal brightness (0.55 ± 0.45) at ~33Hz on magenta (255,0,255). All effects driven by update() called from main loop — no timers or interrupts. Test harness: MEASURE=cycle colors, FIRE=disco toggle, CALIB=purple toggle, DISCO=off. Build: RAM 4.7% (9268B), Flash 17.8% (90220B).
- **Session 10** (code complete — needs hardware test): Menu System — direct port of Python fruity_menu framework to C++ with Adafruit GFX rendering. Evaluated GEM library but rejected (assumes cancel/back button, inline value editing UX mismatch, const char* labels fight dynamic updates). FruityMenu class: scrollable list with pagination (14px/line, 8 items visible), title bar (inverted white), selected item highlight (inverted), circular scroll wrapping, submenu delegation (scroll/click/show auto-delegate to active submenu). FruityMenuItem: action items (callback+int arg) and submenu items. MenuManager class: builds exact Python menu structure — Enter Calibration (Yes/No), Anomaly Detection toggle (On/Off with rebuild), Delete Saved Shots (Yes/No), Laser Timeout (6 options: 30s–30min), Auto Shutdown (6 options: 5min–2hr), Exit (NVIC_SystemReset). Static callbacks via singleton pointer. Dynamic labels rebuilt on setting changes (snprintf into member char buffers). Settings persisted via ConfigManager::saveConfig(). Button mapping: MEASURE=up, DISCO=down, CALIB=select, SHUTDOWN=power off. Auto-shutdown in menu mode. Entry: hold CALIB at boot or set menu flag file. DisplayManager: added getDisplay() accessor. main.cpp: menu mode short-circuits loop() — skips sensor/display/BLE processing. Build: RAM 5.3% (10424B), Flash 18.9% (96060B).
- **Session 12** (code complete — needs hardware test): Main state machine integration — replaced test harness in main.cpp with full production firmware matching Python code.py behavior. Cooperative polling architecture: 9 polling functions called each loop iteration. **Measurement workflow**: MEASURE/FIRE button → state=TAKING_MEASUREMENT → red LED → wait for stability → laser shot → anomaly check → success (green LED, buzzer sequence, BLE send or offline queue) → leg consistency check (3 readings within tolerance → triple buzz + white flash + purple latch) → IDLE. **Button handling**: Button 1 (MEASURE) takes measurement or re-enables laser; Button 2 (DISCO) short press toggles disco, hold 5s launches snake mode via flag+reset; Button 3 (CALIB) hold 2s enters menu via flag+reset; Button 4 (SHUTDOWN) powers off with 1.5s post-measurement guard; FIRE button same as MEASURE. **BLE integration**: connection state monitoring with auto-flush of pending readings on reconnect (blue LED, 1s settle, send each reading with 50ms pacing, clear file). BLE commands: TAKE_SHOT triggers measurement, LASER_ON/OFF, DEVICE_OFF, START_CAL enters menu. **Power management**: low battery check on boot + periodic (≤5% → red LED, 3s delay, shutdown), auto-shutdown after configurable inactivity timeout, laser auto-off after configurable timeout. **Startup sequence**: laser on + buzzer beep, BLE name set to "SAP6_Unicorn", pending count displayed. Sensors polled continuously at 50Hz regardless of state (improved over Python which paused in IDLE). Display updates at 4Hz. Build: RAM 5.5% (10720B), Flash 41.4% (210520B).
- **Session 13** (complete): Fixed QSPI flash write crash. `file.write()` on QSPI hard-faulted when called during measurement flow (after laser UART + sensor I2C activity). Tried: Serial1 drain+delay (no fix), `noInterrupts()`/`interrupts()` around write (no fix — not an ISR conflict). **Solution**: RAM buffer approach — `appendPendingReading()` stores in a 20-entry RAM array (instant, no flash I/O), `syncPendingToFlash()` writes buffered readings to flash from main loop during IDLE state, `doShutdown()` also flushes buffer before pulling PIN_POWER LOW (LTC2952 controlled shutdown = no data loss). `countPendingReadings()`, `flushPendingReadings()`, `clearPendingReadings()` all handle both flash + RAM entries. Hardware tested: measurement flow completes, flash sync works from IDLE. Root cause of QSPI write crash still unknown (not ISR, possibly QSPI peripheral state corruption from concurrent SERCOM/I2C usage or stale CircuitPython FAT). Build: RAM 5.6% (10968B), Flash 41.7% (211580B). **Still untested from Session 12**: DISCO toggle+hold, CALIB hold→menu, BLE send/flush, timeouts, leg completion (3 consistent readings). **Remaining debug prints** in handleMeasurementSuccess should be cleaned up.
- **Session 14** (in progress): Battery gauge fix. MAX17048 only reported 0% or 100% because the Adafruit library's `begin()` calls `reset()` on every boot, wiping the ModelGauge learned state. Fix: subclassed `Adafruit_MAX17048` as `MAX17048_Persistent` — overrides `begin()` to skip `reset()`, preserving SOC tracking across MCU reboots. Also calls `wake()` to exit hibernation, adds 250ms post-init delay for SOC register validity, and logs voltage + SOC + hibernation state on boot and every 30s for diagnostics. Hardware tested: battery readings now track correctly. Build: RAM 5.6% (10968B), Flash 41.8% (212060B). **Still remaining**: debug print cleanup, buzzer sequences, full BLE/timeout/leg test.
- **Session 15** (in progress): Calibration mode display + button fixes. **Problem 1**: All calibration display screens (choice, collection, results, saving) used `DisplayManager::refresh()` which draws the full measurement layout (battery bar, BT label, azimuth 0.0°, inclination 0.0°) — user just saw normal screen with "Calibrate" in the distance slot. **Fix**: Rewrote all 6 `show*Screen()` methods + 2 inline display calls in `updateCalculating()`/`updateSaving()` to draw directly to OLED via `disp_->getDisplay()` with custom layouts (titles, instructions, coverage bar, save/discard hints). **Problem 2**: Buttons didn't work on choice screen — `buttons.update()` called twice per loop (once in `loop()` line 264, again in `calMode.update()` line 53), second call cleared edge flags before `updateChoosing()` could read them. **Fix**: Removed `btns_->update()` from `CalibrationMode::update()`. **Problem 3 (UNRESOLVED)**: Ellipsoid collection hangs during point recording — likely the same QSPI flash crash from Session 13. Calibration mode doesn't write to flash during collection (data stays in RAM vectors), but the hang may be triggered by sensor reads (I2C) + laser buzzer (UART) happening concurrently, or possibly the `delay(200)` after green LED in `updateCollecting()`. Needs investigation — check serial output to see exactly where it hangs. Build: RAM 5.8% (11336B), Flash 44.3% (224848B).
- **Session 17** (complete): Calibration fitting double-precision upgrade. All fitting math (ellipsoid LSQ, axis alignment, roll alignment, non-linear RBF least-squares, accuracy, uniformity, field strength statistics) now uses `double`/`Eigen::MatrixXd`/`Vector3d` internally for numerical stability — matching Python's default float64 precision. Runtime path (apply, getAngles, getOrientationMatrix, sensor fusion) stays `float`/`Vector3f` for performance. Results cast back to float for storage in transform_/centre_ members. Changes: `utils.h` added `normalized()` double overload, `axes.h/.cpp` added `Vector3d fixAxes()`, `rbf.h/.cpp` added `double getGaussians()`, `sensor.cpp` upgraded fitEllipsoid/findPlane/alignToVector/alignAlongAxis/uniformity/setExpectedFieldStrengths, `calibration.cpp` upgraded accuracy/getRawAndExpectedMagData/getLstsqNonLinearParams/alignSensorRoll/setFieldCharacteristics/getDipBatch + added double RAD2DEG_D/DEG2RAD_D constants. Build: RAM 5.7% (11304B), Flash 47.9% (243296B). Flash increase ~32KB from double-precision Eigen template instantiations.
