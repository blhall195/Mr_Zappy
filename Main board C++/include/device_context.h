#pragma once

#include <Arduino.h>
#include "config.h"

// ── System states ───────────────────────────────────────────────────
enum class SystemState : uint8_t {
    IDLE,
    TAKING_MEASUREMENT,
    MENU
};

// ── Live sensor readings ────────────────────────────────────────────
struct Readings {
    float azimuth       = 0.0f;  // degrees, 0-360
    float inclination   = 0.0f;  // degrees, -90 to +90
    float roll          = 0.0f;  // degrees
    float distance      = 0.0f;  // meters
    float batteryLevel  = 0.0f;  // percentage, 0-100
};

// ── Runtime configuration (loaded from flash, editable via menu) ────
struct Config {
    float    magTolerance          = Defaults::magTolerance;
    float    gravTolerance         = Defaults::gravTolerance;
    float    dipTolerance          = Defaults::dipTolerance;
    bool     anomalyDetection      = Defaults::anomalyDetection;
    float    stabilityTolerance    = Defaults::stabilityTolerance;
    uint8_t  stabilityBufferLength = Defaults::stabilityBufferLength;
    float    emaAlpha              = Defaults::emaAlpha;
    float    legAngleTolerance     = Defaults::legAngleTolerance;
    float    legDistanceTolerance  = Defaults::legDistanceTolerance;
    float    laserDistanceOffset   = Defaults::laserDistanceOffset;
    uint32_t autoShutdownTimeout   = Defaults::autoShutdownTimeout;
    uint32_t laserTimeout          = Defaults::laserTimeout;
};

// ── Central device state ────────────────────────────────────────────
struct DeviceContext {
    // State machine
    SystemState currentState = SystemState::IDLE;
    Readings    readings;
    Config      config;
    bool        measurementTaken = false;

    // Peripheral control
    bool laserEnabled  = true;
    bool buzzerEnabled = false;
    bool discoOn       = false;
    bool laserOnFlag   = true;

    // BLE / connectivity
    bool     bleConnected              = false;
    uint16_t bleDisconnectionCounter   = 0;
    bool     bleReadingsTransferredFlag = false;

    // Activity tracking
    uint32_t lastActivityTime    = 0;  // millis()
    uint32_t lastMeasurementTime = 0;  // millis()
    bool     purpleLatched       = false;

    // Stability buffers (fixed-size, index-managed)
    float stableAzimuthBuf[3]      = {};
    float stableInclinationBuf[3]  = {};
    float stableDistanceBuf[3]     = {};
    uint8_t stableBufCount         = 0;

    // (EMA state lives in SensorManager — complementary gravity filter + EMA smoothing)
};
