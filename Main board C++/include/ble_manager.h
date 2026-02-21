#pragma once

#include <Arduino.h>

// ── BLE command codes received from DiscoX ──────────────────────────
// DiscoX sends "ACK_RECEIVED\n" for ACKs, or decimal strings for
// other commands (e.g. "48\n" = STOP_CAL, "54\n" = LASER_ON).
enum class BleCommand : uint8_t {
    NONE = 0,
    ACK_RECEIVED,     // "ACK_RECEIVED\n" from DiscoX
    STOP_CAL,         // "48\n"  (0x30)
    START_CAL,        // "49\n"  (0x31)
    DEVICE_OFF,       // "52\n"  (0x34)
    LASER_ON,         // "54\n"  (0x36)
    LASER_OFF,        // "55\n"  (0x37)
    TAKE_SHOT,        // "56\n"  (0x38)
    UNKNOWN
};

// ── BLE Manager — UART bridge to DiscoX board ──────────────────────
// Uses SERCOM1 on D5 (PA16, TX) / D6 (PA18, RX) at 9600 baud.
// DRDY pin (D12) is pulsed HIGH around each outbound message.
// BLE status pin (D11) is read to detect connection state.
class BleManager {
public:
    bool begin();     // Init SERCOM1 UART, pin mux, DRDY
    void update();    // Non-blocking poll: drain RX, parse lines

    // Outbound (each pulses DRDY around the write)
    void sendSurveyData(float compass, float clino, float distance);
    void sendKeepAlive();
    void setName(const char* name);

    // Inbound
    bool hasCommand() const;
    BleCommand readCommand();    // Returns & clears pending command

    // Status
    bool isConnected() const;    // Reads PIN_BLE_STATUS

    static const char* commandName(BleCommand cmd);

private:
    static constexpr size_t RX_BUF_SIZE = 128;
    char       _rxBuf[RX_BUF_SIZE];
    uint8_t    _rxLen      = 0;
    BleCommand _pendingCmd = BleCommand::NONE;

    void       pulseDrdyAndSend(const char* msg, uint32_t holdMs = 100);
    BleCommand parseLine(const char* line);
};
