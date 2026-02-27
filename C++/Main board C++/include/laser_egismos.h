#pragma once

#include <Arduino.h>

// ── Laser error codes (replaces Python exception hierarchy) ─────────
enum class LaserError : uint8_t {
    OK = 0,
    TIMEOUT,          // No response within timeout period
    COMMAND_FAILED,   // Bad frame, checksum mismatch, or unexpected response
    TOO_DIM,          // ERR255 — laser spot too dim
    TOO_BRIGHT,       // ERR256 — too much ambient light or too close
    BAD_READING,      // ERR204 — unable to measure
};

// ── Egismos laser distance module driver ────────────────────────────
// UART protocol at 9600 baud. Frame: [0xAA][addr][cmd][data...][chk][0xA8]
// Ported from CircuitPython laser_egismos.py (sync Laser class only).
class LaserEgismos {
public:
    bool begin(HardwareSerial& serial, uint8_t address = 0x01,
               float timeoutSec = 5.0f);

    LaserError setLaser(bool on);
    LaserError setBuzzer(bool on);
    LaserError stopMeasuring();

    // Single measurement. Distance returned in mm via out param.
    LaserError measure(int32_t& distanceMm);

    // Convenience: returns distance in meters, or -1.0f on error.
    float distanceMeters();

    LaserError lastError() const { return _lastError; }

    static const char* errorString(LaserError err);

private:
    // ── Protocol constants ──────────────────────────────────────────
    static constexpr uint8_t FRAME_START            = 0xAA;
    static constexpr uint8_t FRAME_END              = 0xA8;
    static constexpr uint8_t CMD_READ_SW_VERSION    = 0x01;
    static constexpr uint8_t CMD_READ_DEV_TYPE      = 0x02;
    static constexpr uint8_t CMD_READ_SLAVE_ADDRESS = 0x04;
    static constexpr uint8_t CMD_READ_DEVICE_ERR    = 0x08;
    static constexpr uint8_t CMD_SET_SLAVE_ADDRESS  = 0x41;
    static constexpr uint8_t CMD_LASER_ON           = 0x42;
    static constexpr uint8_t CMD_LASER_OFF          = 0x43;
    static constexpr uint8_t CMD_SINGLE_MEASURE     = 0x44;
    static constexpr uint8_t CMD_CONTINUOUS_MEASURE  = 0x45;
    static constexpr uint8_t CMD_STOP_CONTINUOUS     = 0x46;
    static constexpr uint8_t CMD_BUZZER_CONTROL     = 0x47;

    static constexpr size_t MAX_FRAME = 16;

    // ── State ───────────────────────────────────────────────────────
    HardwareSerial* _serial = nullptr;
    uint8_t  _address   = 0x01;
    uint32_t _timeoutMs = 5000;
    LaserError _lastError = LaserError::OK;

    // ── Internal helpers ────────────────────────────────────────────
    // Build a command frame into buf, returns frame length.
    size_t buildFrame(uint8_t* buf, uint8_t command,
                      const uint8_t* data = nullptr, size_t dataLen = 0);

    // Read a complete frame (FRAME_START ... FRAME_END) from UART.
    // Returns number of bytes read, or 0 on timeout.
    size_t readFrame(uint8_t* buf, size_t bufSize);

    // Parse a received frame: validates start/end/checksum, extracts
    // command, address, and data region. dataStart/dataLen point into buf.
    bool parseFrame(const uint8_t* buf, size_t len,
                    uint8_t& command, uint8_t& address,
                    const uint8_t*& dataStart, size_t& dataLen);

    // Send command frame, read response, validate command/address match.
    // On success, responseData/responseLen point into internal buffer.
    LaserError sendAndReceive(uint8_t command,
                              const uint8_t*& responseData, size_t& responseLen,
                              const uint8_t* sendData = nullptr, size_t sendLen = 0);

    // Send command and check that response data[0] == 0x01.
    LaserError sendCommandExpectAck(uint8_t command,
                                    const uint8_t* data = nullptr, size_t dataLen = 0);

    // Check measurement response data for error strings.
    LaserError checkMeasurementResult(const uint8_t* data, size_t len,
                                      int32_t& distanceMm);

    // Receive buffer (shared across calls — not reentrant, fine for single-threaded)
    uint8_t _rxBuf[MAX_FRAME];
};
