#pragma once

#include "defaults.h"
#include <Arduino.h>

// ── Button pins (all active LOW with internal pull-ups) ─────────────
constexpr uint8_t PIN_BTN_MEASURE = A3;  // Button 1 — take measurement
constexpr uint8_t PIN_BTN_DISCO = A4;    // Button 2 — quick shot / hold 3s for disco
constexpr uint8_t PIN_BTN_CALIB = A0;    // Button 3 — hold for calibration menu
constexpr uint8_t PIN_BTN_SHUTDOWN = A1; // Button 4 — power off
constexpr uint8_t PIN_BTN_FIRE = 4;      // D4 — fire/trigger button

// ── Power control (LTC2952) ─────────────────────────────────────────
constexpr uint8_t PIN_POWER = A2; // LOW = request shutdown

// ── BLE UART to DiscoX board ────────────────────────────────────────
constexpr uint8_t PIN_BLE_TX = 5;      // D5 — TX to DiscoX
constexpr uint8_t PIN_BLE_RX = 6;      // D6 — RX from DiscoX
constexpr uint8_t PIN_BLE_DRDY = 12;   // D12 — data-ready signal to DiscoX
constexpr uint8_t PIN_BLE_STATUS = 11; // D11 — HIGH when BLE connected (input)
constexpr uint32_t BLE_UART_BAUD = 9600;

// ── Laser UART (hardware Serial1 on Feather M4 CAN) ────────────────
// TX/RX are the default Serial1 pins on Feather M4 CAN
constexpr uint32_t LASER_UART_BAUD_EGISMOS = 9600;
constexpr uint32_t LASER_UART_BAUD_LDJ100 = 115200;

// ── Magnetometer RM3100 ─────────────────────────────────────────────
constexpr uint8_t PIN_MAG_DRDY = 9; // D9 — RM3100 data ready (input)
constexpr uint8_t RM3100_I2C_ADDR = 0x20;
constexpr uint16_t RM3100_CYCLE_COUNT = 400;

// ── Config / mode-select pin ────────────────────────────────────────
constexpr uint8_t PIN_CFG = SCK; // A5/SCK — pull-up, mode selector

// ── I2C addresses ───────────────────────────────────────────────────
constexpr uint8_t ISM330DHCX_ADDR = 0x6A; // Accel/Gyro (default)
constexpr uint8_t MAX17048_ADDR = 0x36;   // Battery gauge
constexpr uint8_t SH1107_ADDR = 0x3D;     // OLED display
constexpr uint8_t SH1107_WIDTH = 128;
constexpr uint8_t SH1107_HEIGHT = 128;

// ── NeoPixel ────────────────────────────────────────────────────────
// PIN_NEOPIXEL is defined by the board variant (pin 8 on Feather M4 CAN)
constexpr uint8_t NEOPIXEL_COUNT = 1;

// ── Calibration axis mappings ───────────────────────────────────────
// Magnetometer: -X, -Y, -Z  (RM3100 orientation on PCB)
// Gravity:      -Y, -X, +Z  (ISM330DHCX orientation on PCB)
constexpr char MAG_AXES[] = "-X-Y-Z";
constexpr char GRAV_AXES[] = "-Y-X+Z";

// ── Sensor fusion constants ─────────────────────────────────────────
// (Complementary gravity filter params are in sensor_manager.h as class constants)
// EMA alpha and stability buffer length come from Defaults:: (defaults.h)
