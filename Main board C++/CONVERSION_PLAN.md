# Mr_Zappy Main Board: CircuitPython to C++ Conversion Plan

## Context

The Mr_Zappy cave surveying device main board (Adafruit Feather M4 CAN / ATSAMD51) currently runs CircuitPython with ~4,000+ lines across 15+ modules. The BLE board (DiscoX) has already been successfully converted to C++/PlatformIO. This plan covers converting the entire main board firmware to C++ using the same toolchain, with live hardware testing at each stage.

**Target directory**: `Main board C++/` — ALL new C++ code goes here.

**Key decisions made:**
- **Build system**: PlatformIO + Arduino framework (matches DiscoX)
- **Concurrency**: Cooperative polling in `loop()` with state machines (matches DiscoX)
- **Linear algebra**: ArduinoEigen (full Eigen coverage, fixed-size matrices, no heap allocation, proven on Cortex-M4)
- **Testing**: Incremental — device plugged in via USB, each module tested via serial before moving on
- **JSON**: ArduinoJson for config/calibration persistence
- **Display**: Adafruit SH1107 + Adafruit GFX libraries (Arduino equivalents of CircuitPython displayio)

---

## File Locations

- **Conversion plan**: `Main board C++/CONVERSION_PLAN.md` (this file)
- **Cross-session context**: `Main board C++/CLAUDE.md` (updated every session)

---

## Multi-Session Conversion Strategy

### Session 0: Project Scaffold & Context Setup
**Goal**: Create PlatformIO project, pin definitions, build config, and CLAUDE.md for cross-session context.

**Tasks**:
1. Initialize PlatformIO project in `Main board C++/` targeting Feather M4 CAN
2. Create `platformio.ini` with:
   - `platform = atmelsam`, `board = adafruit_feather_m4_can`, `framework = arduino`
   - `lib_deps`: ArduinoEigen, ArduinoJson, Adafruit SH1107, Adafruit GFX, Adafruit NeoPixel, Adafruit MAX1704X, Adafruit LSM6DS (ISM330DHCX), Adafruit BusIO
   - `build_flags`: `-DEIGEN_NO_MALLOC -Os`
   - Upload/monitor port config
3. Create `include/config.h` with all pin definitions (from Python `board.*` mappings):
   - Buttons: A0, A1, A3, A4, D4
   - BLE UART: D5 (TX), D6 (RX)
   - Laser UART: TX, RX (hardware Serial1)
   - Mag DRDY: D9, BLE status: D11, Power: A2, DRDY out: D12
   - I2C addresses: RM3100=0x20, SH1107=0x3d, MAX17048=0x36
   - All timing constants, tolerance defaults from `settings.toml`
4. Create `include/device_context.h` — the central state struct (replaces Python's DeviceContext/Readings)
5. Create `src/main.cpp` — minimal skeleton that compiles and prints to serial
6. Create `CLAUDE.md` at project root with architecture overview, module status tracker, and conversion notes
7. **Test**: Upload to board, verify serial output and successful boot

**Target directory structure**:
```
Main board C++/
├── CLAUDE.md                    # Cross-session context
├── CONVERSION_PLAN.md           # This file
├── platformio.ini               # Build config
├── include/                     # Headers
│   ├── config.h                 # Pin defs, constants
│   ├── device_context.h         # Central state struct
│   ├── button_manager.h
│   ├── rm3100.h
│   ├── display_manager.h
│   ├── laser_egismos.h
│   ├── ble_manager.h
│   ├── sensor_manager.h
│   ├── config_manager.h
│   ├── disco_manager.h
│   ├── menu_manager.h
│   ├── calibration_mode.h
│   ├── mag_cal/                 # Calibration math
│   │   ├── axes.h
│   │   ├── rbf.h
│   │   ├── sensor.h
│   │   ├── calibration.h
│   │   ├── lstsq.h
│   │   └── utils.h
│   └── fruity_menu/             # Menu UI framework
│       ├── menu.h
│       ├── abstract.h
│       ├── adjust.h
│       └── builder.h
└── src/                         # Implementation
    ├── main.cpp
    ├── button_manager.cpp
    ├── rm3100.cpp
    ├── display_manager.cpp
    ├── laser_egismos.cpp
    ├── ble_manager.cpp
    ├── sensor_manager.cpp
    ├── config_manager.cpp
    ├── disco_manager.cpp
    ├── menu_manager.cpp
    ├── calibration_mode.cpp
    ├── mag_cal/*.cpp
    └── fruity_menu/*.cpp
```

---

### Session 1: Button Manager
**Goal**: Port button debouncing. First hardware interaction test.

**Tasks**:
1. Create `include/button_manager.h` and `src/button_manager.cpp`
2. Port the 5-button setup with debouncing (use Bounce2 library or hand-roll ~45 lines)
3. Implement: `update()`, `isPressed(name)`, `wasPressed(name)`, `wasReleased(name)`
4. Wire into `main.cpp` with a serial test harness that prints button events
5. **Test**: Upload, user presses each button, verify serial output shows correct events
6. Update CLAUDE.md with module status

**Files**: `include/button_manager.h`, `src/button_manager.cpp`
**Depends on**: Session 0
**Python reference**: `lib/button_manager.py` (45 lines)

---

### Session 2: I2C Sensor Drivers
**Goal**: Get raw readings from all I2C sensors via serial.

**Tasks**:
1. **RM3100 magnetometer**: Create `include/rm3100.h`, `src/rm3100.cpp`
   - Port I2C register read/write, cycle count config, single/continuous reading, DRDY pin support
   - 24-bit two's complement conversion
   - Python reference: `lib/rm3100.py` (273 lines)
2. **ISM330DHCX accelerometer/gyro**: Use Adafruit LSM6DS Arduino library directly
   - Configure data rates, ranges
   - Verify accel + gyro readings
3. **MAX17048 battery gauge**: Use Adafruit MAX1704X Arduino library directly
   - Read battery percentage
4. Serial test harness: print all raw sensor values at 10Hz
5. **Test**: Upload, verify magnetometer XYZ, accel XYZ, gyro XYZ, battery % all read sensible values

**Files**: `include/rm3100.h`, `src/rm3100.cpp` (custom), rest use library APIs
**Depends on**: Session 0
**Python reference**: `lib/rm3100.py`, `lib/sensor_manager.py` (sensor init portions)

---

### Session 3: Display Manager
**Goal**: Get the SH1107 OLED showing sensor readings.

**Tasks**:
1. Create `include/display_manager.h`, `src/display_manager.cpp`
2. Initialize SH1107 128x128 via Adafruit SH1107 + GFX libraries
3. Port display layout: battery bar (top-right), BT symbol (top-left), 3 data lines (distance, azimuth, inclination)
4. Implement: `init()`, `updateSensorReadings(dist, az, inc)`, `updateBattery(pct)`, `updateBTLabel(connected)`, `updateBTNumber(pending)`, `blankScreen()`, `showStartingMenu(text)`
5. Wire into main loop with live sensor readings from Session 2
6. **Test**: Upload, verify display shows real-time sensor data, battery level

**Files**: `include/display_manager.h`, `src/display_manager.cpp`
**Depends on**: Session 0 (Session 2 for live data integration)
**Python reference**: `lib/display_manager.py` (179 lines)

---

### Session 4: Laser Driver(s)
**Goal**: Control laser module, take distance measurements.

**Tasks**:
1. Create `include/laser_egismos.h`, `src/laser_egismos.cpp`
   - Port UART frame protocol (0xAA start, 0xA8 end, checksum)
   - Commands: laser on/off, buzzer on/off, single measure
   - Error handling: TooDim, TooBright, BadReading, Timeout
   - Python reference: `lib/laser_egismos.py` (395 lines)
2. Optionally create `include/laser_ldj100.h`, `src/laser_ldj100.cpp`
   - Port register-based protocol (115200 baud, more complex)
   - Python reference: `lib/laser_ldj100.py` (820 lines)
3. Common laser interface (abstract base or compile-time selection)
4. Serial test harness: laser on, measure, print distance, laser off
5. **Test**: Upload, verify laser toggles, distance readings are accurate

**Files**: `include/laser_egismos.h`, `src/laser_egismos.cpp` (and optionally ldj100)
**Depends on**: Session 0
**Python reference**: `lib/laser_egismos.py`, `lib/laser_ldj100.py`

---

### Session 5: BLE Manager (UART Bridge)
**Goal**: Send/receive messages to the DiscoX BLE board.

**Tasks**:
1. Create `include/ble_manager.h`, `src/ble_manager.cpp`
2. Port UART protocol on D5/D6 at 9600 baud with DRDY handshake on D12
3. Implement: `sendMessage(compass, clino, dist)`, `readMessage()`, `setName(name)`, `sendKeepAlive()`
4. Message framing: `COMPASS:XXX.X,CLINO:YYY.Y,DIST:ZZ.ZZ\n`
5. Command parsing: ACK0/1 (0x55/0x56), START_CAL, STOP_CAL, LASER_ON/OFF, TAKE_SHOT, DEVICE_OFF
6. Serial test: send test messages, verify BLE board receives them (check DiscoX serial monitor)
7. **Test**: Upload both boards, verify bidirectional communication

**Files**: `include/ble_manager.h`, `src/ble_manager.cpp`
**Depends on**: Session 0 (DiscoX already running)
**Python reference**: `lib/ble_manager.py` (59 lines)

---

### Session 6: Calibration Math Library (Core)
**Goal**: Port mag_cal to C++ with ArduinoEigen. This is the hardest session.

**Tasks**:
1. Create `include/mag_cal/` directory with:
   - `axes.h` / `axes.cpp` — axis permutation/sign flip (85 lines Python)
   - `rbf.h` / `rbf.cpp` — radial basis functions (60 lines Python)
   - `sensor.h` / `sensor.cpp` — Sensor class with ellipsoid fitting, apply(), as_dict/from_dict (286 lines Python)
   - `calibration.h` / `calibration.cpp` — main Calibration class with get_angles(), get_calibrated(), orientation matrix, angle extraction (697 lines Python)
   - `lstsq.h` / `lstsq.cpp` — least squares via Eigen QR (102 lines Python, much simpler with Eigen)
   - `utils.h` / `utils.cpp` — cross product, normalize, solve_least_squares (116 lines Python)
2. Key Eigen mappings:
   - `np.linalg.qr()` -> `Eigen::HouseholderQR`
   - `np.linalg.eig()` -> `Eigen::EigenSolver` or `SelfAdjointEigenSolver`
   - `np.linalg.inv()` -> `.inverse()`
   - `np.linalg.lstsq()` -> `.colPivHouseholderQr().solve()`
   - `np.dot()` -> `*` operator
   - `np.cross()` -> `.cross()` (Vector3f)
   - `np.linalg.norm()` -> `.norm()`
3. JSON serialization with ArduinoJson: load/save `calibration_dict.json`
4. **Test strategy**: Load the existing `calibration_dict.json` from the Python project. Feed known raw sensor values through the C++ calibration pipeline. Compare output angles against the Python version. Print results via serial.
5. Port `calibrate_roll.py` (113 lines) — rotation matrix fitting for roll correction

**Files**: `include/mag_cal/*.h`, `src/mag_cal/*.cpp` (~8 files)
**Depends on**: Session 0, ArduinoEigen
**Python reference**: `lib/mag_cal/` (1,573 lines total), `lib/calibrate_roll.py` (113 lines)

**This is the largest session — may need to split across 2 sessions (6a: sensor/axes/rbf/utils/lstsq, 6b: calibration/roll)**

---

### Session 7: Sensor Manager (Fusion & Filtering)
**Goal**: Calibrated sensor readings with complementary filter and EMA smoothing.

**Tasks**:
1. Create `include/sensor_manager.h`, `src/sensor_manager.cpp`
2. Port complementary filter for gravity estimation:
   - Gyro prediction: `pred = prev + (gyro x prev) * dt`
   - Adaptive alpha: 0.03 (still) to 0.8 (moving), based on gyro magnitude
   - Gyro bias learning when stationary (alpha = 0.001)
3. Port EMA smoothing for azimuth/inclination
4. Integrate calibration library: raw sensors -> `calibration.getAngles(mag, grav)` -> smoothed angles
5. Stability detection: 3-sample buffer, check variance < 0.4 deg
6. Wire into main loop: print calibrated az/inc/roll at 10Hz
7. **Test**: Upload, move device around, verify angles are stable and accurate. Compare against known bearings if possible.

**Files**: `include/sensor_manager.h`, `src/sensor_manager.cpp`
**Depends on**: Sessions 2, 6
**Python reference**: `lib/sensor_manager.py` (157 lines), EMA/stability code from `code.py`

---

### Session 8: Config & NVM Persistence
**Goal**: Persistent settings and calibration storage.

**Tasks**:
1. Create `include/config_manager.h`, `src/config_manager.cpp`
2. Implement settings storage (flash or SD — evaluate options for SAMD51):
   - All parameters from `settings.toml`
   - Save/load with ArduinoJson
   - Flash storage via Adafruit SPIFlash + LittleFS, or EEPROM emulation
3. Calibration dict persistence: save/load `calibration_dict.json`
4. Pending readings queue: write/read/flush offline measurement buffer
5. Flag file system: calibration_mode, menu_mode triggers (or use NVM flags instead of files)
6. **Test**: Change a setting, reboot, verify it persists

**Files**: `include/config_manager.h`, `src/config_manager.cpp`
**Depends on**: Session 0
**Python reference**: `lib/config.py` (27 lines), `settings.toml`, `boot.py`

---

### Session 9: Disco Manager (LED Effects)
**Goal**: NeoPixel LED feedback — colors, disco mode, purple pulse.

**Tasks**:
1. Create `include/disco_manager.h`, `src/disco_manager.cpp`
2. Port static colors: red, green, blue, white, purple, off
3. Port disco rainbow effect (HSV cycling with gyro-driven wild mode)
4. Port purple pulse (sine wave brightness modulation)
5. Integration with cooperative loop (non-blocking state machine for effects)
6. **Test**: Upload, trigger each effect via serial commands, verify LED behavior

**Files**: `include/disco_manager.h`, `src/disco_manager.cpp`
**Depends on**: Session 0
**Python reference**: `lib/disco_manager.py` (162 lines)

---

### Session 10: Menu System
**Goal**: Settings menu on OLED display with button navigation.

**Tasks**:
1. Create `include/fruity_menu/` — port the menu framework:
   - `menu.h/cpp` — core menu with pagination, scrolling, selection (332 lines Python)
   - `abstract.h` — base classes (39 lines Python)
   - `adjust.h/cpp` — BoolMenu, NumberMenu, OptionMenu (201 lines Python)
   - `builder.h/cpp` — declarative menu construction (70 lines Python)
2. Port `menu_manager.h/cpp` — the specific Mr_Zappy menu structure:
   - Enter Calibration, Anomaly Detection toggle, Delete Saved Shots
   - Laser Off Timeout (30s-30min), Auto Shutdown (5min-2hr), Exit
3. Rendering via Adafruit GFX (replacing displayio)
4. **Test**: Upload, navigate full menu tree with buttons, verify settings save correctly

**Files**: `include/fruity_menu/*.h`, `src/fruity_menu/*.cpp`, `include/menu_manager.h`, `src/menu_manager.cpp`
**Depends on**: Sessions 1, 3, 8
**Python reference**: `lib/fruity_menu/` (642 lines), `lib/menu_manager.py` (304 lines)

---

### Session 11: Calibration Mode Workflow
**Goal**: Full calibration data collection and processing on device.

**Tasks**:
1. Create `include/calibration_mode.h`, `src/calibration_mode.cpp`
2. Port ellipsoid calibration workflow: 56-point collection with stability checking
3. Port alignment calibration workflow: 24-point collection (3 directions x 8 rotations)
4. Coverage bar visualization on display
5. Consistency checking (5-sample buffer at each orientation)
6. Calculate and save calibration coefficients
7. Integration with calibration math library (Session 6)
8. Nelder-Mead optimization port (`nm.h/cpp`) if needed for non-linear fitting
9. **Test**: Full calibration run with device, verify saved coefficients produce accurate angles

**Files**: `include/calibration_mode.h`, `src/calibration_mode.cpp`, `include/nm.h`, `src/nm.cpp`
**Depends on**: Sessions 1, 2, 3, 6, 9
**Python reference**: `lib/calibration_mode.py` (129 lines), `lib/calibration_manager.py` (248 lines)

---

### Session 12: Main State Machine Integration
**Goal**: Wire everything together into the full application.

**Tasks**:
1. Port the 7 async tasks as cooperative polling functions in `loop()`:
   - `pollSensors()` — sensor read + EMA + stability + display update
   - `pollButtons()` — button events -> state transitions
   - `pollBattery()` — every 30s check
   - `pollBLEPin()` — connection state monitoring
   - `pollBLEUart()` — command processing
   - `checkAutoShutoff()` — inactivity timeout
   - `checkLaserTimeout()` — laser power save
2. State machine: IDLE -> TAKING_MEASUREMENT -> success/error -> IDLE
3. Measurement pipeline: stability detection -> 3-sample confirmation -> BLE send or queue
4. Circular bearing difference handling (360 deg wraparound)
5. Power control via LTC2952 (A2 pin)
6. Boot mode detection (calibration/menu/normal)
7. **Test**: Full end-to-end — take measurements, see them on display, transmit via BLE, verify in SexyTopo or similar app

**Files**: `src/main.cpp` (major expansion)
**Depends on**: All previous sessions
**Python reference**: `code.py` (650 lines)

---

### Session 13: Polish, Edge Cases & Power Management
**Goal**: Handle all remaining edge cases and optimizations.

**Tasks**:
1. Offline queue: pending readings file management, flush on BLE reconnect
2. Auto-shutdown timeout (configurable, default 30 min)
3. Laser timeout (configurable, default 2 min)
4. Low battery shutdown (< 5%)
5. Anomaly detection (mag/grav/dip error display)
6. Button guard timing (1.5s lockout after measurement to prevent accidental shutdown)
7. Error recovery: laser reset, sensor reinit
8. Memory usage audit (verify no heap fragmentation)
9. Flash usage audit (verify fits in 512KB)
10. **Test**: Stress test — extended operation, BLE disconnect/reconnect, low battery simulation

**Depends on**: Session 12

---

## Verification Strategy

Each session follows the same test pattern:
1. **Compile check**: `platformio run` succeeds with no errors/warnings
2. **Upload**: `platformio run --target upload`
3. **Serial verification**: `platformio device monitor --baud 115200`
4. **Hardware test**: User interaction (press buttons, move device, check display, measure distances)
5. **Comparison**: Where possible, compare C++ output against Python version running same inputs

For the calibration math (Session 6), additionally:
- Load the existing `calibration_dict.json` Python coefficients
- Feed identical raw sensor vectors through both Python and C++ pipelines
- Verify angles match within 0.01 degrees

---

## Estimated Scope

| Session | Lines (est.) | Complexity | Can skip? |
|---------|-------------|------------|-----------|
| 0: Scaffold | 200 | Low | No |
| 1: Buttons | 100 | Low | No |
| 2: Sensors | 350 | Medium | No |
| 3: Display | 250 | Medium | No |
| 4: Laser | 500-900 | Medium-High | No (need one driver minimum) |
| 5: BLE UART | 150 | Low | No |
| 6: Calibration math | 800-1200 | **Very High** | No |
| 7: Sensor fusion | 250 | High | No |
| 8: Config/NVM | 200 | Medium | No |
| 9: Disco LED | 150 | Low | Could defer |
| 10: Menu system | 500-700 | High | Could defer (use serial config initially) |
| 11: Calibration mode | 400 | High | No |
| 12: Main integration | 400 | High | No |
| 13: Polish | 200 | Medium | No |

**Total estimated**: ~4,000-5,500 lines of C++
