#include "laser_egismos.h"
#include <string.h>  // memcmp

// ── Public API ──────────────────────────────────────────────────────

bool LaserEgismos::begin(HardwareSerial& serial, uint8_t address,
                         float timeoutSec) {
    _serial    = &serial;
    _address   = address;
    _timeoutMs = static_cast<uint32_t>(timeoutSec * 1000.0f);
    _lastError = LaserError::OK;
    return true;
}

LaserError LaserEgismos::setLaser(bool on) {
    _lastError = sendCommandExpectAck(on ? CMD_LASER_ON : CMD_LASER_OFF);
    return _lastError;
}

LaserError LaserEgismos::setBuzzer(bool on) {
    uint8_t data = on ? 0x01 : 0x00;
    _lastError = sendCommandExpectAck(CMD_BUZZER_CONTROL, &data, 1);
    return _lastError;
}

LaserError LaserEgismos::stopMeasuring() {
    _lastError = sendCommandExpectAck(CMD_STOP_CONTINUOUS);
    return _lastError;
}

LaserError LaserEgismos::measure(int32_t& distanceMm) {
    const uint8_t* respData;
    size_t respLen;
    _lastError = sendAndReceive(CMD_SINGLE_MEASURE, respData, respLen);
    if (_lastError != LaserError::OK) return _lastError;

    _lastError = checkMeasurementResult(respData, respLen, distanceMm);
    return _lastError;
}

float LaserEgismos::distanceMeters() {
    int32_t mm;
    if (measure(mm) != LaserError::OK) return -1.0f;
    return mm / 1000.0f;
}

const char* LaserEgismos::errorString(LaserError err) {
    switch (err) {
        case LaserError::OK:             return "OK";
        case LaserError::TIMEOUT:        return "TIMEOUT";
        case LaserError::COMMAND_FAILED: return "COMMAND_FAILED";
        case LaserError::TOO_DIM:        return "TOO_DIM";
        case LaserError::TOO_BRIGHT:     return "TOO_BRIGHT";
        case LaserError::BAD_READING:    return "BAD_READING";
        default:                         return "UNKNOWN";
    }
}

// ── Frame building ──────────────────────────────────────────────────

size_t LaserEgismos::buildFrame(uint8_t* buf, uint8_t command,
                                const uint8_t* data, size_t dataLen) {
    size_t idx = 0;
    buf[idx++] = FRAME_START;
    buf[idx++] = _address;
    buf[idx++] = command;

    uint8_t checksum = _address + command;
    for (size_t i = 0; i < dataLen; i++) {
        buf[idx++] = data[i];
        checksum += data[i];
    }
    buf[idx++] = checksum & 0x7F;
    buf[idx++] = FRAME_END;
    return idx;
}

// ── Frame reading ───────────────────────────────────────────────────

size_t LaserEgismos::readFrame(uint8_t* buf, size_t bufSize) {
    uint32_t deadline = millis() + _timeoutMs;

    // Wait for FRAME_START
    while (true) {
        if (_serial->available()) {
            uint8_t b = _serial->read();
            if (b == FRAME_START) {
                buf[0] = b;
                break;
            }
        }
        if (millis() > deadline) return 0;
    }

    // Read until FRAME_END
    size_t len = 1;
    while (len < bufSize) {
        if (_serial->available()) {
            uint8_t b = _serial->read();
            buf[len++] = b;
            if (b == FRAME_END) return len;
        }
        if (millis() > deadline) return 0;
    }
    return 0;  // buffer overflow
}

// ── Frame parsing ───────────────────────────────────────────────────

bool LaserEgismos::parseFrame(const uint8_t* buf, size_t len,
                              uint8_t& command, uint8_t& address,
                              const uint8_t*& dataStart, size_t& dataLen) {
    if (len < 5) return false;  // minimum: START + addr + cmd + chk + END
    if (buf[0] != FRAME_START) return false;
    if (buf[len - 1] != FRAME_END) return false;

    // Checksum covers bytes between START and checksum byte (indices 1..len-3)
    uint8_t checksum = 0;
    for (size_t i = 1; i <= len - 3; i++) {
        checksum += buf[i];
    }
    checksum &= 0x7F;
    if (buf[len - 2] != checksum) return false;

    address   = buf[1];
    command   = buf[2];
    dataStart = buf + 3;
    dataLen   = len - 5;  // exclude START, addr, cmd, checksum, END
    return true;
}

// ── Send and receive ────────────────────────────────────────────────

LaserError LaserEgismos::sendAndReceive(uint8_t command,
                                        const uint8_t*& responseData,
                                        size_t& responseLen,
                                        const uint8_t* sendData,
                                        size_t sendLen) {
    uint8_t txBuf[MAX_FRAME];
    size_t txLen = buildFrame(txBuf, command, sendData, sendLen);

    // Clear RX buffer before sending
    while (_serial->available()) _serial->read();

    _serial->write(txBuf, txLen);
    _serial->flush();  // wait for TX complete

    size_t rxLen = readFrame(_rxBuf, MAX_FRAME);
    if (rxLen == 0) return LaserError::TIMEOUT;

    uint8_t rxCmd, rxAddr;
    const uint8_t* rxData;
    size_t rxDataLen;
    if (!parseFrame(_rxBuf, rxLen, rxCmd, rxAddr, rxData, rxDataLen)) {
        return LaserError::COMMAND_FAILED;
    }

    if (rxCmd != command) return LaserError::COMMAND_FAILED;
    if (rxAddr != _address) return LaserError::COMMAND_FAILED;

    responseData = rxData;
    responseLen  = rxDataLen;
    return LaserError::OK;
}

LaserError LaserEgismos::sendCommandExpectAck(uint8_t command,
                                              const uint8_t* data,
                                              size_t dataLen) {
    const uint8_t* respData;
    size_t respLen;
    LaserError err = sendAndReceive(command, respData, respLen, data, dataLen);
    if (err != LaserError::OK) return err;

    if (respLen == 0 || respData[0] != 0x01) {
        return LaserError::COMMAND_FAILED;
    }
    return LaserError::OK;
}

// ── Measurement result parsing ──────────────────────────────────────

LaserError LaserEgismos::checkMeasurementResult(const uint8_t* data,
                                                size_t len,
                                                int32_t& distanceMm) {
    // Check for known error strings
    if (len == 6 && memcmp(data, "ERR256", 6) == 0) return LaserError::TOO_BRIGHT;
    if (len == 6 && memcmp(data, "ERR255", 6) == 0) return LaserError::TOO_DIM;
    if (len == 6 && memcmp(data, "ERR204", 6) == 0) return LaserError::BAD_READING;

    // Parse ASCII integer (distance in mm)
    int32_t value = 0;
    for (size_t i = 0; i < len; i++) {
        if (data[i] < '0' || data[i] > '9') return LaserError::COMMAND_FAILED;
        value = value * 10 + (data[i] - '0');
    }
    distanceMm = value;
    return LaserError::OK;
}
