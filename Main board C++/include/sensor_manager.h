#pragma once
// Sensor fusion: complementary gravity filter + direct angle calculation + EMA
// Session 7 revised: replaces Madgwick AHRS with approach matching Python code.
// Device is held steady — trust mag/accel, use gyros only for smoothing gravity.

#include <ArduinoEigenDense.h>
#include "mag_cal/calibration.h"

class SensorManager {
public:
    /// Initialise with calibration reference and tuning parameters
    void init(const MagCal::Calibration* cal,
              float emaAlpha, uint8_t stabilityLen);

    /// Feed raw sensor readings + time step; updates filtered gravity, angles, EMA
    void update(const Eigen::Vector3f& rawMag,
                const Eigen::Vector3f& rawAccel,
                const Eigen::Vector3f& rawGyro,
                float dt);

    // ── Filtered angles (from calibration + EMA) ─────────────────────
    float getAzimuth()     const { return emaAz_; }      // [0, 360)
    float getInclination() const { return emaInc_; }      // [-90, +90]
    float getRoll()        const { return roll_; }

    // ── Stability detection ──────────────────────────────────────────
    bool isStable(float tolerance) const;
    void resetStability();

    // ── Debug accessors ──────────────────────────────────────────────
    const Eigen::Vector3f& getFilteredGrav() const { return filtGrav_; }
    const Eigen::Vector3f& getGyroBias()     const { return gyroBias_; }

    /// Wraparound-safe angular distance (0–180)
    static float circularDiff(float a, float b);

private:
    // Complementary gravity filter state
    Eigen::Vector3f filtGrav_  = Eigen::Vector3f(0.0f, 0.0f, 9.81f);
    Eigen::Vector3f gyroBias_  = Eigen::Vector3f::Zero();
    bool seeded_ = false;

    // Dynamic alpha thresholds (rad/s) for complementary filter
    static constexpr float GYRO_THRESH_LOW  = 0.05f;  // below = stationary
    static constexpr float GYRO_THRESH_HIGH = 0.5f;   // above = full motion
    static constexpr float ALPHA_LOW        = 0.03f;   // stationary: trust gyro prediction
    static constexpr float ALPHA_HIGH       = 0.8f;    // moving: trust raw accel
    static constexpr float BIAS_LEARN_RATE  = 0.001f;  // slow gyro bias EMA

    // EMA-smoothed output angles
    float emaAz_  = 0.0f;
    float emaInc_ = 0.0f;
    float roll_   = 0.0f;
    float emaAlpha_ = 0.5f;
    bool emaSeeded_ = false;

    // Stability ring buffer
    static constexpr uint8_t MAX_STAB_BUF = 8;
    float azBuf_[MAX_STAB_BUF]  = {};
    float incBuf_[MAX_STAB_BUF] = {};
    uint8_t stabHead_  = 0;
    uint8_t stabCount_ = 0;
    uint8_t stabLen_   = 3;

    // Calibration (not owned)
    const MagCal::Calibration* cal_ = nullptr;

    // ── Internal helpers ─────────────────────────────────────────────
    void complementaryGravFilter(const Eigen::Vector3f& rawAccel,
                                  const Eigen::Vector3f& rawGyro,
                                  float dt);
    void pushStability(float az, float inc);

    /// Circular EMA for azimuth (handles 0/360 wraparound)
    static float circularEma(float prev, float next, float alpha);
};
