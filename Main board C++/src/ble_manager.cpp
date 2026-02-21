#include "ble_manager.h"
#include "config.h"
#include "wiring_private.h"

// ── SERCOM1 UART on D5 (PA16, TX, PAD[0]) / D6 (PA18, RX, PAD[2]) ─
// SERCOM1 is nominally allocated to the SPI header but this project
// doesn't use SPI (sensors use I2C, flash uses QSPI), so we repurpose
// the SERCOM for a second hardware UART.
static Uart bleSerial(&sercom1, PIN_BLE_RX, PIN_BLE_TX,
                       SERCOM_RX_PAD_2, UART_TX_PAD_0);

// SAMD51 has 4 interrupt vectors per SERCOM
void SERCOM1_0_Handler() { bleSerial.IrqHandler(); }
void SERCOM1_1_Handler() { bleSerial.IrqHandler(); }
void SERCOM1_2_Handler() { bleSerial.IrqHandler(); }
void SERCOM1_3_Handler() { bleSerial.IrqHandler(); }

// ── begin ────────────────────────────────────────────────────────────
bool BleManager::begin() {
    // Start UART
    bleSerial.begin(BLE_UART_BAUD);

    // Remap PA16/PA18 from their default (timer) to SERCOM1 (MUX C)
    pinPeripheral(PIN_BLE_TX, PIO_SERCOM);
    pinPeripheral(PIN_BLE_RX, PIO_SERCOM);

    // DRDY pin already configured in initPins(), but ensure it's LOW
    digitalWrite(PIN_BLE_DRDY, LOW);

    _rxLen      = 0;
    _pendingCmd = BleCommand::NONE;
    return true;
}

// ── update — non-blocking RX poll ────────────────────────────────────
void BleManager::update() {
    while (bleSerial.available()) {
        char c = (char)bleSerial.read();

        if (c == '\n') {
            // Null-terminate and process
            _rxBuf[_rxLen] = '\0';

            // Trim trailing \r if present
            if (_rxLen > 0 && _rxBuf[_rxLen - 1] == '\r') {
                _rxBuf[_rxLen - 1] = '\0';
            }

            if (_rxLen > 0) {
                BleCommand cmd = parseLine(_rxBuf);
                if (cmd != BleCommand::NONE) {
                    _pendingCmd = cmd;
                }
            }
            _rxLen = 0;
        } else if (c != '\r' && _rxLen < RX_BUF_SIZE - 1) {
            _rxBuf[_rxLen++] = c;
        }
        // Overflow: silently drop characters until next \n
    }
}

// ── Outbound: survey data ────────────────────────────────────────────
void BleManager::sendSurveyData(float compass, float clino, float distance) {
    char buf[64];
    snprintf(buf, sizeof(buf), "COMPASS:%.1f,CLINO:%.1f,DIST:%.2f\n",
             (double)compass, (double)clino, (double)distance);
    Serial.print(F("BLE TX: ")); Serial.print(buf);
    pulseDrdyAndSend(buf, 100);
}

// ── Outbound: keep-alive ─────────────────────────────────────────────
void BleManager::sendKeepAlive() {
    pulseDrdyAndSend("ALIVE\n", 50);
}

// ── Outbound: name change ────────────────────────────────────────────
void BleManager::setName(const char* name) {
    char buf[64];
    snprintf(buf, sizeof(buf), "NAME:%s\n", name);
    pulseDrdyAndSend(buf, 100);
}

// ── Inbound command access ───────────────────────────────────────────
bool BleManager::hasCommand() const {
    return _pendingCmd != BleCommand::NONE;
}

BleCommand BleManager::readCommand() {
    BleCommand cmd = _pendingCmd;
    _pendingCmd = BleCommand::NONE;
    return cmd;
}

// ── BLE connection status ────────────────────────────────────────────
bool BleManager::isConnected() const {
    return digitalRead(PIN_BLE_STATUS) == HIGH;
}

// ── Command name for debug printing ──────────────────────────────────
const char* BleManager::commandName(BleCommand cmd) {
    switch (cmd) {
        case BleCommand::NONE:         return "NONE";
        case BleCommand::ACK_RECEIVED: return "ACK_RECEIVED";
        case BleCommand::STOP_CAL:     return "STOP_CAL";
        case BleCommand::START_CAL:    return "START_CAL";
        case BleCommand::DEVICE_OFF:   return "DEVICE_OFF";
        case BleCommand::LASER_ON:     return "LASER_ON";
        case BleCommand::LASER_OFF:    return "LASER_OFF";
        case BleCommand::TAKE_SHOT:    return "TAKE_SHOT";
        case BleCommand::UNKNOWN:      return "UNKNOWN";
        default:                       return "???";
    }
}

// ── Internal: pulse DRDY and send message ────────────────────────────
void BleManager::pulseDrdyAndSend(const char* msg, uint32_t holdMs) {
    digitalWrite(PIN_BLE_DRDY, HIGH);
    bleSerial.print(msg);
    bleSerial.flush();       // Wait for TX to complete
    delay(holdMs);           // Hold DRDY for DiscoX to drain
    digitalWrite(PIN_BLE_DRDY, LOW);
}

// ── Internal: parse a complete line from DiscoX ──────────────────────
BleCommand BleManager::parseLine(const char* line) {
    // ACK acknowledgement (sent as literal string by DiscoX C++)
    if (strcmp(line, "ACK_RECEIVED") == 0) {
        return BleCommand::ACK_RECEIVED;
    }

    // Decimal command codes (e.g. "48", "54")
    char* end;
    long val = strtol(line, &end, 10);
    if (*end != '\0') {
        // Not a pure integer — unknown format
        return BleCommand::UNKNOWN;
    }

    switch (val) {
        case 0x30: return BleCommand::STOP_CAL;     // 48
        case 0x31: return BleCommand::START_CAL;     // 49
        case 0x34: return BleCommand::DEVICE_OFF;    // 52
        case 0x36: return BleCommand::LASER_ON;      // 54
        case 0x37: return BleCommand::LASER_OFF;     // 55
        case 0x38: return BleCommand::TAKE_SHOT;     // 56
        default:   return BleCommand::UNKNOWN;
    }
}
