#pragma once

#include <Arduino.h>
#include <Wire.h>

class RM3100 {
public:
    // Raw reading as signed 24-bit counts
    struct Reading { int32_t x, y, z; };

    bool begin(TwoWire& wire, uint8_t addr, uint16_t cycleCount, int8_t drdyPin = -1);

    void startSingleReading();
    void startContinuousReading(float frequency = 300.0f);
    void stop();

    bool measurementComplete() const;
    Reading getLastReading();
    float   measurementTime() const;

    // Convenience: blocking single shot
    Reading readSingle();

    // Convert raw counts to microtesla
    void toMicroTesla(const Reading& raw, float& ux, float& uy, float& uz) const;

private:
    void writeReg(uint8_t reg, const uint8_t* data, uint8_t len);
    void readReg(uint8_t reg, uint8_t* buf, uint8_t len);
    uint8_t readReg8(uint8_t reg);

    TwoWire* wire_      = nullptr;
    uint8_t  addr_      = 0x20;
    uint16_t cycleCount_ = 200;
    int8_t   drdyPin_   = -1;
    bool     continuous_ = false;

    static constexpr float CYCLE_DURATION = 0.000036f; // seconds per cycle
    static constexpr float UT_PER_CYCLE   = 2.5f;      // µT per LSB per cycle
};
