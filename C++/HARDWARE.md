# Mr Zappy — Hardware Design

## What Is It?

Mr Zappy is a handheld electronic cave surveying instrument. It measures compass bearing (azimuth), inclination, and laser distance between survey stations underground, then transmits each "leg" of survey data to a phone app over Bluetooth Low Energy for real-time cave map construction.

The device is built from off-the-shelf Adafruit breakout boards pin-headered onto a custom PCB. This semi-modular approach was a deliberate choice — cave survey instruments live hard lives, and when something inevitably gets cracked against a rock or drowned in a sump, the user can swap out individual boards without replacing the whole unit.

## System Overview

At its core, Mr Zappy is a dual-microcontroller design: a main processor handles all the sensing, calibration, display, and measurement logic, while a second board provides the Bluetooth radio link to the outside world. The two boards communicate over a simple 9600-baud UART with a handshake line.

```
                            ┌─────────────────────────────────────────────────────────┐
                            │              MAIN BOARD                                 │
                            │      Adafruit Feather M4 CAN (ATSAME51J19)              │
                            │      ARM Cortex-M4F @ 120 MHz │ 512KB Flash │ 192KB RAM │
                            │                                                         │
  ┌──────────────┐   I2C    │  ┌─────────────┐  ┌─────────────┐  ┌──────────────┐    │
  │   RM3100     │◄────────►│  │             │  │             │  │              │    │
  │ Magnetometer │  0x20    │  │             │  │   Sensor    │  │   Display    │    │
  │  (PNI)       │          │  │             │  │   Manager   │  │   Manager    │    │
  └──────────────┘          │  │             │  │  (EMA +     │  │  (4 Hz       │    │
                            │  │             │  │  stability) │  │  refresh)    │    │
  ┌──────────────┐   I2C    │  │             │  │             │  │              │    │
  │  ISM330DHCX  │◄────────►│  │   ATSAME51  │  └──────┬──────┘  └──────┬───────┘    │
  │  IMU 6-DoF   │  0x6A    │  │             │         │                │            │
  │  (ST Micro)  │          │  │             │         ▼                ▼            │
  └──────────────┘          │  │             │  ┌─────────────────────────────┐      │
                            │  │             │  │     Calibration Engine      │      │
  ┌──────────────┐   I2C    │  │             │  │  (Eigen, ellipsoid fit,     │      │
  │   MAX17048   │◄────────►│  │             │  │   RBF, F/B correction)     │      │
  │ Fuel Gauge   │  0x36    │  │             │  └─────────────────────────────┘      │
  │(Analog Dev.) │          │  │             │                                       │
  └──────────────┘          │  │             │                                       │
                            │  │             │                                       │
  ┌──────────────┐   I2C    │  │             │         ┌──────────────┐              │
  │   SH1107     │◄────────►│  │             │         │  2MB QSPI    │              │
  │ 128x128 OLED │  0x3D    │  │             │◄───────►│  Flash (FAT) │              │
  │  1.12" mono  │          │  │             │  QSPI   │  config/cal  │              │
  └──────────────┘          │  │             │         └──────────────┘              │
                            │  │             │                                       │
  ┌──────────────┐  UART    │  │             │         ┌──────────────┐              │
  │   Egismos    │◄────────►│  │             │────────►│  WS2812B     │              │
  │ Laser Range  │ Serial1  │  │             │  GPIO   │  NeoPixel    │              │
  │  Module M2   │ 9600 bd  │  │             │         │  RGB LED     │              │
  └──────────────┘          │  │             │         └──────────────┘              │
                            │  │             │                                       │
  ┌──────────────┐  GPIO    │  │             │         ┌──────────────┐              │
  │   LTC2952    │◄────────►│  │             │◄────────│  5 Buttons   │              │
  │ Power Ctrl   │  A2      │  │             │  GPIO   │  (active LOW │              │
  │(Analog Dev.) │          │  │             │         │  + pull-ups) │              │
  └──────────────┘          │  └─────────────┘         └──────────────┘              │
                            │         │                                              │
                            └─────────┼──────────────────────────────────────────────┘
                                      │
                              UART (SERCOM1, 9600 baud)
                              D5 (TX) / D6 (RX)
                              D12 (DRDY handshake)
                              D11 (BLE status input)
                                      │
                            ┌─────────┼──────────────────────────────────────────────┐
                            │         ▼                                              │
                            │              BLE BOARD                                 │
                            │   Adafruit ItsyBitsy nRF52840 Express                  │
                            │   ARM Cortex-M4F @ 64 MHz │ 1MB Flash │ 256KB RAM      │
                            │                                                        │
                            │  ┌─────────────┐   ┌──────────────┐                    │
                            │  │  UART       │   │  SAP6 BLE    │                    │
                            │  │  Handler    │──►│  Protocol    │                    │
                            │  │  (DRDY-     │   │  (GATT,      │    ┌─────────┐     │
                            │  │   gated)    │   │  seq-bit     │───►│  Phone  │     │
                            │  └─────────────┘   │  ACK/retry)  │    │  App    │     │
                            │                    └──────────────┘    │ (SAP6)  │     │
                            │  ┌─────────────┐               BLE   └─────────┘     │
                            │  │  NVM Mgr    │   Coded PHY (Long                    │
                            │  │ (LittleFS)  │   Range) + 8 dBm TX                  │
                            │  └─────────────┘                                       │
                            │                                                        │
                            └────────────────────────────────────────────────────────┘
```

## Why Two Microcontrollers?

This is probably the first question anyone looks at the block diagram and asks, so it's worth addressing up front.

The project started life in CircuitPython — a fork of MicroPython that provides a full Python 3 interpreter running directly on a microcontroller. It's a wonderful language for rapid prototyping: you edit a `.py` file, save it to the board's USB drive, and it runs immediately with no compile step. The downside is that an interpreted language consumes significantly more RAM and CPU cycles than compiled native code. The sensor fusion, calibration maths, and BLE stack together exceeded what a single mid-range board could comfortably handle in CircuitPython, so the workload was split across two chips — an ARM Cortex-M4 for the heavy computation, and a Nordic nRF52840 dedicated to the Bluetooth radio.

The firmware was later rewritten in C++ for performance, dropping memory usage from near the limits to around 6% of available RAM. At that point, a single nRF52840 could probably handle everything, and future hardware revisions will likely consolidate to one chip. But the dual-board architecture has proven its worth in practice — it keeps the BLE stack completely isolated from the timing-sensitive sensor loop, and it means the radio firmware can be updated independently from the measurement firmware.

## The Sensors

### PNI RM3100 — Magnetometer

The compass bearing is the most critical measurement in cave surveying, and the magnetometer is where the accuracy budget is won or lost. The RM3100 uses PNI's magneto-inductive (MagI) sensing technology rather than the Hall-effect sensors found in most consumer-grade compass chips. The practical difference is significant: 10 nT resolution, 23x better than competing Hall-effect parts, with 33x lower noise and — crucially — no hysteresis or temperature-dependent offset drift.

This isn't an exotic choice in the cave surveying world. The DistoX2, which is probably the most widely used electronic cave survey instrument, uses the same sensor family. When you're trying to achieve sub-degree compass accuracy deep underground — surrounded by ferrous rock, with no GPS to cross-reference — you need a magnetometer that earns its keep. The RM3100 does.

The sensor communicates over I2C at address 0x20, with a hardware data-ready pin to signal when a new reading is available. It's configured with a cycle count of 400, which trades some sample rate for maximum resolution. Raw readings are passed through a multi-stage calibration pipeline: ellipsoid fitting for hard and soft-iron correction, Gaussian radial basis function (RBF) non-linear correction, and an optional foresight/backsight field check to compensate for residual errors from the calibration environment.

### ST ISM330DHCX — 6-DoF IMU

The IMU provides the accelerometer for inclination measurement and the gyroscope for gravity vector smoothing. The ISM330DHCX is an industrial-grade 6-DoF sensor from ST — it's configured here at ±4g acceleration and ±250 dps angular rate, both sampled at 104 Hz.

This was chosen as the best consumer-grade IMU available for under £10. It's comparable to the grade of IMU used in both the SAP6 and the DistoX2. The industrial temperature range (-40 to +105°C) is a useful bonus for a device that will see everything from tropical cave systems to alpine potholes, though it wasn't the primary selection criterion. Future revisions may move to a higher-end part like the Murata SCA3300, but during rapid prototyping iterations the priority was keeping per-board cost down while still achieving the required accuracy.

The accelerometer's gravity vector gives an absolute tilt reference for inclination, while the gyroscope feeds a complementary filter that smooths out hand tremor and vibration. The filter uses a dynamic alpha — trusting the gyroscope more when the device is stationary (for stability) and the accelerometer more during motion (to prevent drift). The gyroscope also enables shake detection for the LED disco mode, which is entirely non-essential but excellent for cave survey camp morale.

### Maxim MAX17048 — Battery Fuel Gauge

Rather than estimating battery state from raw voltage (which is unreliable for LiPo cells due to their flat discharge curve), Mr Zappy uses a dedicated fuel gauge IC. The MAX17048 uses Maxim's ModelGauge algorithm to continuously track the battery's state of charge to ±1% accuracy, drawing only 23 µA in active mode.

There's one notable firmware quirk worth mentioning: the Adafruit Arduino library for this chip calls `reset()` every time `begin()` is invoked, which wipes the IC's learned battery model. After a reset, the chip falls back to a voltage-only first guess and reports only 0% or 100% until it re-learns the cell's characteristics. The solution was to subclass the library and override `begin()` to skip the reset, preserving the ModelGauge state across reboots. A small fix, but the kind of thing that costs you a day of head-scratching when the battery gauge just doesn't work.

## The Laser

### Egismos Model M2 — Laser Rangefinder

Distance measurement uses an Egismos M2 time-of-flight laser module — a compact (37.5 x 45.3 x 19.2 mm) unit with ±3 mm accuracy and a measurement range of up to 30 metres (a 100 m variant is also available). It communicates over UART at 9600 baud using a binary protocol with framed commands.

There are cheaper laser rangefinder modules on the market, but the Egismos was chosen for its longer range and proven track record — it's the same module used in the BRIC cave survey device. In cave surveying, you're often shooting distances down long, dark passages where a budget module would struggle to get a return. The Egismos has consistently delivered reliable readings in those conditions.

The module also includes a controllable laser pointer and a piezo buzzer, both of which the firmware co-opts for user feedback: the laser blinks on point acceptance during calibration, and the buzzer provides audio confirmation for measurements, errors, and the distinctive triple-beep when a survey leg is complete. A configurable offset of 0.162 m is subtracted from each reading to account for the physical distance from the laser module to the rear of the instrument where it contacts the survey station.

One discovered quirk: the Egismos module crashes if you send UART commands too rapidly in succession. The firmware enforces a minimum 20 ms delay between consecutive commands, which solved the issue but required reworking several sequences where the original code fired off laser-on, buzzer-on, and measure commands back-to-back.

## The Main Processor

### Adafruit Feather M4 CAN (ATSAME51J19)

The main board runs on an Adafruit Feather M4 CAN, which is based on Microchip's ATSAME51J19 — an ARM Cortex-M4F clocked at 120 MHz with 512 KB flash, 192 KB RAM, and a hardware floating-point unit. The FPU is essential: the calibration engine uses the Eigen linear algebra library for matrix operations including ellipsoid fitting, axis alignment, and radial basis function interpolation — all floating-point intensive work that would be painfully slow in software emulation.

The board also provides 2 MB of QSPI flash (used as a FAT filesystem for configuration, calibration data, and offline survey storage), multiple SERCOM peripherals (one repurposed as a second UART for the BLE board), and a USB-C connector.

Why this specific board? Honestly, it was the only Adafruit breakout with an M4-class chip and a USB-C connector. The CAN transceiver that gives the board its name goes completely unused — it's just along for the ride. The firmware currently occupies about 51% of program flash and 6% of RAM, leaving comfortable headroom for future features.

## The BLE Radio

### Adafruit ItsyBitsy nRF52840 Express

The BLE board is an Adafruit ItsyBitsy nRF52840 Express — a tiny (1.4" x 0.7") board built around the Nordic nRF52840, which combines a 64 MHz Cortex-M4F with a Bluetooth 5.0 Low Energy radio.

Its sole job is bridging the main board's UART output to a phone app. It implements the SAP6 BLE protocol — a custom GATT service with reliable delivery using a sequence-bit ACK/retry mechanism. Each survey leg is packed into a 17-byte notification containing azimuth, inclination, roll, and distance as four IEEE 754 floats plus a sequence bit. If the phone doesn't acknowledge within 5 seconds, the data is retransmitted.

The radio is configured for Coded PHY (BLE Long Range) where supported, using S=8 encoding at 125 kbps with +8 dBm transmit power. This provides approximately 4x the range of standard 1 Mbps BLE — useful when the surveyor carrying the phone is a few passages away from the instrument operator. Getting Coded PHY working required patching the Adafruit Bluefruit library at the SoftDevice level, increasing the connection event length, and implementing a retry loop for PHY negotiation after connection. If the phone doesn't support Coded PHY (which includes all iOS devices), it falls back gracefully to standard 1 Mbps.

## Display

### SH1107 — 128x128 Monochrome OLED

The display is a 1.12" monochrome OLED running the SH1107 controller at I2C address 0x3D. At 128x128 pixels it's compact but highly readable in the dark — OLEDs are self-emitting, so there's no backlight to wash out in dim conditions, and the white-on-black pixel arrangement means the display draws minimal power when showing sparse UI elements.

The firmware refreshes the display at 4 Hz, which provides a responsive live readout of compass heading, inclination, and laser distance without consuming excessive I2C bandwidth (the display shares the bus with all three sensors). The same screen handles a hierarchical settings menu, calibration workflow UI, and — if you hold the right button for 5 seconds in the menu — a fully playable 16x16 snake game.

## Power Management

### LTC2952 — Pushbutton Power Controller

Cave instruments need to turn fully off. Not sleep, not deep-sleep — off, drawing effectively zero current, so they can sit in a tackle bag for months between expeditions without draining the battery.

The LTC2952 from Analog Devices provides exactly this: pushbutton-controlled hard power switching with a microprocessor handshake. When the user presses the shutdown button, the MCU receives an interrupt and has time to flush any pending survey data from its RAM buffer to flash, close the filesystem cleanly, and then pull pin A2 LOW to signal that it's safe to cut power. The LTC2952 also prevents the device from being shut down during boot or while flash writes are in progress, protecting against data corruption from impatient button presses.

The device also implements auto-shutdown after 30 minutes of inactivity (configurable), a 2-minute laser timeout to conserve the laser module, and a forced shutdown at 5% battery with a red LED warning and 3-second grace period.

### Battery

A single-cell 3.7V LiPo, monitored by the MAX17048 fuel gauge. Battery percentage is displayed on the OLED as a status bar and checked every 30 seconds.

## User Interface

Five tactile push buttons with internal pull-up resistors and 10 ms debounce handle all user input:

| Button | Primary Function | Hold Function |
|--------|-----------------|---------------|
| MEASURE | Take a survey reading | Re-enable laser if timed out |
| DISCO | Quick shot | 2s: disco mode toggle |
| CALIB | — | 1s: settings menu |
| SHUTDOWN | Power off | — |
| FIRE | Same as MEASURE (trigger position) | — |

A single WS2812B NeoPixel RGB LED provides colour-coded status feedback: red during measurement, green on success, blue during BLE data flush, white flash on leg completion, and a purple pulse when a survey leg is latched. It also does a rather good rainbow disco mode with accelerometer-driven shake detection for "wild mode" — because even in a serious survey instrument, you need a party trick.

## Communication Summary

| Bus | Speed | Purpose |
|-----|-------|---------|
| I2C (SDA/SCL) | 400 kHz | All sensors (RM3100, ISM330DHCX, MAX17048) + OLED display |
| UART Serial1 | 9600 baud | Egismos laser module (binary framed protocol) |
| UART SERCOM1 (D5/D6) | 9600 baud | Main board to BLE board (ASCII lines + DRDY handshake) |
| BLE (Coded PHY / 1M) | 125 kbps / 1 Mbps | BLE board to phone app (SAP6 GATT, 17-byte leg notifications) |

## Sources

- [Adafruit Feather M4 CAN Express](https://www.adafruit.com/product/4759)
- [Adafruit ItsyBitsy nRF52840 Express](https://www.adafruit.com/product/4481)
- [PNI RM3100 Magnetometer](https://www.pnisensor.com/rm3100/)
- [ST ISM330DHCX Datasheet](https://www.st.com/resource/en/datasheet/ism330dhcx.pdf)
- [Adafruit ISM330DHCX Breakout](https://www.adafruit.com/product/4502)
- [MAX17048 Datasheet (Analog Devices)](https://www.analog.com/en/products/max17048.html)
- [Adafruit 128x128 OLED Display](https://www.adafruit.com/product/5297)
- [Egismos Laser Distance Module M2](https://www.egismos.com/laser-distance-module-2M)
- [LTC2952 Datasheet (Analog Devices)](https://www.analog.com/en/products/ltc2952.html)
