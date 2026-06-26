#pragma once

#include "config.h"
#include "leg_checker.h"
#include "shot_buffer.h"
#include <Arduino.h>

// ── System states ───────────────────────────────────────────────────
enum class SystemState : uint8_t { IDLE, TAKING_MEASUREMENT, MENU };

// ── Live sensor readings ────────────────────────────────────────────
struct Readings {
    float azimuth = 0.0f;      // degrees, 0-360
    float inclination = 0.0f;  // degrees, -90 to +90
    float roll = 0.0f;         // degrees
    float distance = 0.0f;     // meters
    float batteryLevel = 0.0f; // percentage, 0-100
};

// ── Runtime configuration (loaded from flash, editable via menu) ────
struct Config {
    float magTolerance = Defaults::magTolerance;
    float gravTolerance = Defaults::gravTolerance;
    float dipTolerance = Defaults::dipTolerance;
    bool anomalyDetection = Defaults::anomalyDetection;
    float stabilityTolerance = Defaults::stabilityTolerance;
    uint8_t stabilityBufferLength = Defaults::stabilityBufferLength;
    float emaAlphaStable = Defaults::emaAlphaStable;
    float emaAlphaMoving = Defaults::emaAlphaMoving;
    float legAngleTolerance = Defaults::legAngleTolerance;
    float cartesianTolerance = Defaults::cartesianTolerance;
    float laserDistanceOffset = Defaults::laserDistanceOffset;
    float calMagConsistency = Defaults::calMagConsistency;
    float calGravConsistency = Defaults::calGravConsistency;
    uint8_t calBufferLength = Defaults::calBufferLength;
    uint16_t calSettleMs = Defaults::calSettleMs;
    float calEmaAlpha = Defaults::calEmaAlpha;
    uint16_t calTimeoutMs = Defaults::calTimeoutMs;
    uint32_t autoShutdownTimeout = Defaults::autoShutdownTimeout;
    uint32_t laserTimeout = Defaults::laserTimeout;
    bool laserWibble = Defaults::laserWibble;
    bool measureFromFront = Defaults::measureFromFront;
    bool splaysEnabled = Defaults::splaysEnabled;
    uint8_t screenBrightness = Defaults::screenBrightness;
    char bleName[Defaults::bleNameMaxLen + 1] = {}; // initialized in constructor

    Config() {
        strncpy(bleName, Defaults::bleName, Defaults::bleNameMaxLen);
        bleName[Defaults::bleNameMaxLen] = '\0';
    }
};

// ── Central device state ────────────────────────────────────────────
struct DeviceContext {
    // State machine
    SystemState currentState = SystemState::IDLE;
    Readings readings;
    Config config;
    bool measurementTaken = false;

    // Peripheral control
    bool laserEnabled = true;
    bool buzzerEnabled = false;
    bool discoOn = false;
    bool laserOnFlag = true;
    bool quickShot = false; // true = use wider stability tolerance (button 2)

    // BLE / connectivity
    bool bleConnected = false;
    uint16_t bleDisconnectionCounter = 0;
    bool bleReadingsTransferredFlag = false;

    // Activity tracking
    uint32_t lastActivityTime = 0;    // millis()
    uint32_t lastMeasurementTime = 0; // millis()
    bool purpleLatched = false;
    bool displayFrozen = false; // true = show frozen shot readings, not live

    // Leg-consistency buffer
    CartesianLegChecker legChecker{Defaults::cartesianTolerance};
    ShotBuffer shotBuf = ShotBuffer(legChecker);

    // Stability checker (angular, tolerance in degrees)
    AngularLegChecker stabilityChecker{Defaults::stabilityTolerance};
    AngularLegChecker quickShotStabilityChecker{Defaults::quickShotStabilityTol};

    // (EMA state lives in SensorManager — complementary gravity filter + EMA smoothing)
};
