// sap6_protocol.cpp — SAP6 BLE GATT service + reliable-delivery state machine
//
// Faithful port of F:\lib\caveble.py  SurveyProtocolService

#include "sap6_protocol.h"
#include "config.h"

// ---------------------------------------------------------------------------
// UUID byte arrays — little-endian for Bluefruit
//
//   String form                                  → reversed bytes
//   137c4435-8a64-4bcb-93f1-3792c6bdc965 (service)
//   137c4435-8a64-4bcb-93f1-3792c6bdc966 (proto name)
//   137c4435-8a64-4bcb-93f1-3792c6bdc967 (command)
//   137c4435-8a64-4bcb-93f1-3792c6bdc968 (leg data)
// ---------------------------------------------------------------------------
const uint8_t SAP6_SERVICE_UUID[16] = {
    0x65, 0xC9, 0xBD, 0xC6, 0x92, 0x37, 0xF1, 0x93,
    0xCB, 0x4B, 0x64, 0x8A, 0x35, 0x44, 0x7C, 0x13
};
const uint8_t SAP6_PROTO_NAME_UUID[16] = {
    0x66, 0xC9, 0xBD, 0xC6, 0x92, 0x37, 0xF1, 0x93,
    0xCB, 0x4B, 0x64, 0x8A, 0x35, 0x44, 0x7C, 0x13
};
const uint8_t SAP6_COMMAND_UUID[16] = {
    0x67, 0xC9, 0xBD, 0xC6, 0x92, 0x37, 0xF1, 0x93,
    0xCB, 0x4B, 0x64, 0x8A, 0x35, 0x44, 0x7C, 0x13
};
const uint8_t SAP6_LEG_DATA_UUID[16] = {
    0x68, 0xC9, 0xBD, 0xC6, 0x92, 0x37, 0xF1, 0x93,
    0xCB, 0x4B, 0x64, 0x8A, 0x35, 0x44, 0x7C, 0x13
};

// ---------------------------------------------------------------------------
// Global instance
// ---------------------------------------------------------------------------
SAP6Protocol sap6;

// ---------------------------------------------------------------------------
// Static BLE write callback trampoline
// ---------------------------------------------------------------------------
static void commandWriteCb(uint16_t conn_hdl, BLECharacteristic* chr,
                           uint8_t* data, uint16_t len) {
    (void)conn_hdl;
    (void)chr;
    if (len >= 1) {
        sap6.onCommandWrite(conn_hdl, data, len);
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void SAP6Protocol::begin() {
    // --- Service ---
    _service.begin();

    // --- Protocol Name characteristic (read-only, fixed "SAP6") ---
    _protoNameChar.setProperties(CHR_PROPS_READ);
    _protoNameChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    _protoNameChar.setFixedLen(4);
    _protoNameChar.begin();
    _protoNameChar.write("SAP6", 4);

    // --- Command characteristic (write-only, uint8) ---
    _commandChar.setProperties(CHR_PROPS_WRITE);
    _commandChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
    _commandChar.setFixedLen(1);
    _commandChar.setWriteCallback(commandWriteCb);
    _commandChar.begin();

    // --- Leg Data characteristic (read + notify, 17 bytes) ---
    //     Struct: <Bffff  =  1 byte sequence-bit + 4 × float32
    _legDataChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    _legDataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    _legDataChar.setFixedLen(17);
    _legDataChar.begin();
}

void SAP6Protocol::sendData(float azimuth, float inclination,
                             float distance, float roll) {
    // Queue the reading (drop if full — same as Python's deque((), 20) behaviour)
    if (_queueCount < QUEUE_CAP) {
        LegReading& r = _queue[_queueTail];
        r.azimuth     = azimuth;
        r.inclination = inclination;
        r.roll        = roll;
        r.distance    = distance;
        _queueTail    = (_queueTail + 1) % QUEUE_CAP;
        _queueCount++;
    }
    pollOut();  // try to send immediately  (matches caveble.py:93)
}

int SAP6Protocol::poll() {
    int result = pollIn();
    pollOut();
    return result;
}

int SAP6Protocol::pending() {
    return _queueCount + (_waitingForAck ? 1 : 0);
}

void SAP6Protocol::onCommandWrite(uint16_t /*conn_hdl*/,
                                   uint8_t* data, uint16_t /*len*/) {
    _pendingCmd  = data[0];
    _cmdReceived = true;
}

// ---------------------------------------------------------------------------
// Internal state machine
// ---------------------------------------------------------------------------

// pollIn — exact port of caveble.py _poll_in() (lines 120-137)
int SAP6Protocol::pollIn() {
    if (!_cmdReceived) return -1;

    uint8_t cmd  = _pendingCmd;
    _cmdReceived = false;          // "self.command = 0"

    Serial.printf("got cmd: %u\n", cmd);

    // After sending with bit X we toggled _lastSentBit to X^1.
    // ACK_TABLE is [0x56, 0x55] (mirrors Python ACK = [0x56, 0x55]).
    uint8_t expectedAck   = ACK_TABLE[_lastSentBit];
    uint8_t unexpectedAck = ACK_TABLE[_lastSentBit ^ 1];

    if (cmd == expectedAck) {
        Serial.println("ACK received");
        _waitingForAck = false;
        return cmd;                // correct ACK → return to caller
    } else if (cmd == unexpectedAck) {
        Serial.printf("Wrong ack received: expecting %u\n", expectedAck);
        return -1;                 // wrong-sequence ACK → swallow silently
    } else {
        return cmd;                // non-ACK command → forward to caller
    }
}

// pollOut — exact port of caveble.py _poll_out() (lines 106-118)
void SAP6Protocol::pollOut() {
    if (_waitingForAck) {
        // Resend if no ACK within timeout  (self.leg = self.leg in Python)
        if (millis() - _lastSendTime > ACK_TIMEOUT_MS) {
            _legDataChar.notify(_currentPacket, sizeof(_currentPacket));
            _lastSendTime = millis();
        }
    } else if (_queueCount > 0) {
        // Dequeue next reading
        LegReading& r = _queue[_queueHead];
        _queueHead = (_queueHead + 1) % QUEUE_CAP;
        _queueCount--;

        // Pack:  <Bffff  — little-endian on ARM Cortex-M4 = native byte order
        _currentPacket[0] = _lastSentBit;
        memcpy(&_currentPacket[1],  &r.azimuth,     4);
        memcpy(&_currentPacket[5],  &r.inclination,  4);
        memcpy(&_currentPacket[9],  &r.roll,         4);
        memcpy(&_currentPacket[13], &r.distance,     4);

        _legDataChar.write(_currentPacket, sizeof(_currentPacket));
        _legDataChar.notify(_currentPacket, sizeof(_currentPacket));

        _lastSentBit ^= 1;          // toggle AFTER sending (caveble.py:116)
        _lastSendTime = millis();
        _waitingForAck = true;
    }
}
