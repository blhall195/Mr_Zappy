#pragma once
#include <Arduino.h>
#include <bluefruit.h>
#include "config.h"

// One queued survey measurement
struct LegReading {
    float azimuth;
    float inclination;
    float roll;       // always 0.0 in current usage
    float distance;
};

// ---------------------------------------------------------------------------
// SAP6Protocol — BLE GATT service + reliable-delivery state machine
//
// Faithful port of caveble.py SurveyProtocolService.
// ---------------------------------------------------------------------------
class SAP6Protocol {
public:
    // Create the GATT service and characteristics, register with Bluefruit.
    // Call AFTER Bluefruit.begin().
    void begin();

    // Queue a measurement for BLE transmission (roll defaults to 0).
    void sendData(float azimuth, float inclination, float distance,
                  float roll = 0.0f);

    // Drive the state machine: check for inbound commands, handle ACK/retry.
    // Returns the command byte (>= 0) if a command should be forwarded to
    // UART, or -1 if nothing actionable.
    int poll();

    // How many readings are waiting (queue + any in-flight packet).
    int pending();

    // Called from the static BLE write callback trampoline.
    void onCommandWrite(uint16_t conn_hdl, uint8_t* data, uint16_t len);

    // Expose service for advertising setup — read-only accessor.
    BLEService& service() { return _service; }

private:
    void pollOut();
    int  pollIn();

    // GATT objects — initialised in begin()
    BLEService        _service{SAP6_SERVICE_UUID};
    BLECharacteristic _protoNameChar{SAP6_PROTO_NAME_UUID};
    BLECharacteristic _commandChar{SAP6_COMMAND_UUID};
    BLECharacteristic _legDataChar{SAP6_LEG_DATA_UUID};

    // Send queue (max SEND_QUEUE_MAX items)
    static constexpr int QUEUE_CAP = SEND_QUEUE_MAX;
    LegReading _queue[SEND_QUEUE_MAX];
    int        _queueHead = 0;
    int        _queueTail = 0;
    int        _queueCount = 0;

    // Sequence-bit ACK/retry state
    uint8_t  _lastSentBit   = 0;
    bool     _waitingForAck = false;
    uint32_t _lastSendTime  = 0;
    uint8_t  _currentPacket[17] = {};  // cached for resend

    // Inbound command from BLE write callback (volatile — set from BLE context)
    volatile uint8_t _pendingCmd  = 0;
    volatile bool    _cmdReceived = false;
};

// Single global instance
extern SAP6Protocol sap6;
