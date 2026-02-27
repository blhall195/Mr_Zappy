#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Pin assignments — Adafruit ItsyBitsy nRF52840 Express
// ---------------------------------------------------------------------------
constexpr int PIN_DRDY          = 7;   // D7  input  pull-down — master signals data ready
constexpr int PIN_BLE_CONNECTED = 5;   // D5  output — HIGH when BLE connected
constexpr int PIN_LZR_POWER     = A2;  // A2  output — HIGH = laser power on

// ---------------------------------------------------------------------------
// UART (to/from master survey instrument)
// ---------------------------------------------------------------------------
constexpr uint32_t UART_BAUD = 9600;

// ---------------------------------------------------------------------------
// BLE UUIDs — 128-bit, stored as little-endian byte arrays for Bluefruit
//
//   String form (big-endian):
//     Service : 137c4435-8a64-4bcb-93f1-3792c6bdc965
//     Proto   : 137c4435-8a64-4bcb-93f1-3792c6bdc966
//     Command : 137c4435-8a64-4bcb-93f1-3792c6bdc967
//     Leg     : 137c4435-8a64-4bcb-93f1-3792c6bdc968
// ---------------------------------------------------------------------------
extern const uint8_t SAP6_SERVICE_UUID[16];
extern const uint8_t SAP6_PROTO_NAME_UUID[16];
extern const uint8_t SAP6_COMMAND_UUID[16];
extern const uint8_t SAP6_LEG_DATA_UUID[16];

// ---------------------------------------------------------------------------
// SAP6 protocol command bytes
// ---------------------------------------------------------------------------
constexpr uint8_t CMD_ACK0       = 0x55;  // Acknowledge leg with sequence bit 0
constexpr uint8_t CMD_ACK1       = 0x56;  // Acknowledge leg with sequence bit 1
constexpr uint8_t CMD_STOP_CAL   = 0x30;  // Finish calibration
constexpr uint8_t CMD_START_CAL  = 0x31;  // Start calibration
constexpr uint8_t CMD_DEVICE_OFF = 0x34;  // Turn device off
constexpr uint8_t CMD_LASER_ON   = 0x36;  // Turn laser on
constexpr uint8_t CMD_LASER_OFF  = 0x37;  // Turn laser off
constexpr uint8_t CMD_TAKE_SHOT  = 0x38;  // Take a reading

// ACK lookup table — mirrors Python's  ACK = [0x56, 0x55]
//
// After sending with bit X the code toggles _lastSentBit to X^1.
// The *expected* ACK for the packet we just sent is ACK_TABLE[_lastSentBit]
// (i.e. indexed by the already-toggled value).
//   ACK_TABLE[0] = 0x56 (ACK1)
//   ACK_TABLE[1] = 0x55 (ACK0)
constexpr uint8_t ACK_TABLE[2] = { 0x56, 0x55 };

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
constexpr uint32_t ACK_TIMEOUT_MS   = 5000;  // resend after 5 s with no ACK
constexpr uint32_t LOOP_INTERVAL_MS = 10;    // main-loop period
constexpr uint32_t CONN_CHECK_MS    = 100;   // connection-state poll interval
constexpr uint32_t DRDY_DEBOUNCE_MS = 10;    // debounce for DRDY rising edge
constexpr int      SEND_QUEUE_MAX   = 20;    // max queued leg readings

// ---------------------------------------------------------------------------
// Non-volatile storage
// ---------------------------------------------------------------------------
constexpr uint8_t NVM_MAGIC        = 0xBE;
constexpr uint8_t MAX_NAME_LEN     = 20;
constexpr char    DEFAULT_NAME[]   = "SAP6_Basic";
constexpr char    NVM_FILENAME[]   = "/name.dat";

// ---------------------------------------------------------------------------
// BLE radio
// ---------------------------------------------------------------------------
constexpr int8_t BLE_TX_POWER = 8;  // +8 dBm
