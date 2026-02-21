#include "rm3100.h"
#include <math.h>

// Register addresses
static constexpr uint8_t REG_POLL   = 0x00;
static constexpr uint8_t REG_CMM    = 0x01;
static constexpr uint8_t REG_CCX    = 0x04;
static constexpr uint8_t REG_TMRC   = 0x0B;
static constexpr uint8_t REG_MX     = 0x24;
static constexpr uint8_t REG_STATUS = 0x34;

static constexpr float LN2 = 0.693147f;

bool RM3100::begin(TwoWire& wire, uint8_t addr, uint16_t cycleCount, int8_t drdyPin) {
    wire_       = &wire;
    addr_       = addr;
    cycleCount_ = cycleCount;
    drdyPin_    = drdyPin;
    continuous_ = false;

    if (drdyPin_ >= 0) {
        pinMode(drdyPin_, INPUT);
    }

    // Verify device responds
    wire_->beginTransmission(addr_);
    if (wire_->endTransmission() != 0) return false;

    // Set cycle counts for X, Y, Z (big-endian uint16 each)
    uint8_t cc[6] = {
        (uint8_t)(cycleCount_ >> 8), (uint8_t)(cycleCount_ & 0xFF),
        (uint8_t)(cycleCount_ >> 8), (uint8_t)(cycleCount_ & 0xFF),
        (uint8_t)(cycleCount_ >> 8), (uint8_t)(cycleCount_ & 0xFF),
    };
    writeReg(REG_CCX, cc, 6);
    return true;
}

void RM3100::startSingleReading() {
    uint8_t cmd = 0x70; // poll all 3 axes
    writeReg(REG_POLL, &cmd, 1);
}

void RM3100::startContinuousReading(float frequency) {
    // Compute TMRC exponent: freq = 600 / 2^exp
    int exp = (int)roundf(logf(600.0f / frequency) / LN2);
    if (exp < 0)  exp = 0;
    if (exp > 13) exp = 13;
    uint8_t tmrc = 0x92 + exp;
    writeReg(REG_TMRC, &tmrc, 1);

    // Start continuous mode, all 3 axes
    uint8_t cmm = 0x79;
    writeReg(REG_CMM, &cmm, 1);
    continuous_ = true;
}

void RM3100::stop() {
    uint8_t cmm = 0x70;
    writeReg(REG_CMM, &cmm, 1);
    continuous_ = false;
}

bool RM3100::measurementComplete() const {
    if (drdyPin_ >= 0) {
        return digitalRead(drdyPin_) == HIGH;
    }
    // Fall back to status register polling
    uint8_t status;
    const_cast<RM3100*>(this)->readReg(REG_STATUS, &status, 1);
    return (status & 0x80) != 0;
}

RM3100::Reading RM3100::getLastReading() {
    uint8_t buf[9];
    readReg(REG_MX, buf, 9);

    Reading r;
    // 24-bit big-endian two's complement for each axis
    int32_t vals[3];
    for (uint8_t i = 0; i < 3; i++) {
        uint32_t raw = ((uint32_t)buf[i * 3] << 16)
                     | ((uint32_t)buf[i * 3 + 1] << 8)
                     |  (uint32_t)buf[i * 3 + 2];
        // Sign-extend 24-bit to 32-bit
        if (raw & 0x800000) raw |= 0xFF000000;
        vals[i] = (int32_t)raw;
    }
    r.x = vals[0];
    r.y = vals[1];
    r.z = vals[2];
    return r;
}

float RM3100::measurementTime() const {
    return CYCLE_DURATION * cycleCount_;
}

RM3100::Reading RM3100::readSingle() {
    startSingleReading();
    // Wait for measurement to complete
    uint32_t timeout = millis() + 100; // 100ms timeout
    while (!measurementComplete()) {
        if (millis() > timeout) break;
        delayMicroseconds(100);
    }
    return getLastReading();
}

void RM3100::toMicroTesla(const Reading& raw, float& ux, float& uy, float& uz) const {
    float factor = UT_PER_CYCLE / cycleCount_;
    ux = raw.x * factor;
    uy = raw.y * factor;
    uz = raw.z * factor;
}

// ── I2C helpers ─────────────────────────────────────────────────────

void RM3100::writeReg(uint8_t reg, const uint8_t* data, uint8_t len) {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    wire_->write(data, len);
    wire_->endTransmission();
}

void RM3100::readReg(uint8_t reg, uint8_t* buf, uint8_t len) {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    wire_->endTransmission(false); // repeated start
    wire_->requestFrom(addr_, len);
    for (uint8_t i = 0; i < len && wire_->available(); i++) {
        buf[i] = wire_->read();
    }
}

uint8_t RM3100::readReg8(uint8_t reg) {
    uint8_t val;
    readReg(reg, &val, 1);
    return val;
}
