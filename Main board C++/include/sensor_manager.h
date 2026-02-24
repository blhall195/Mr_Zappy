#pragma once
// Sensor fusion: direct angle calculation from mag + accel, EMA output smoothing.

#include <ArduinoEigenDense.h>
#include "mag_cal/calibration.h"

class SensorManager {
public:
    /// Initialise with calibration reference and tuning parameters
    void init(const MagCal::Calibration* cal,
              float emaAlpha, uint8_t stabilityLen);

    /// Feed raw sensor readings; updates angles and EMA
    void update(const Eigen::Vector3f& rawMag,
                const Eigen::Vector3f& rawAccel);

    // ── Filtered angles (from calibration + EMA) ─────────────────────
    float getAzimuth()     const { return emaAz_; }      // [0, 360)
    float getInclination() const { return emaInc_; }      // [-90, +90]
    float getRoll()        const { return roll_; }

    // ── Stability detection ──────────────────────────────────────────
    bool isStable(float tolerance) const;
    void resetStability();

    /// Wraparound-safe angular distance (0–180)
    static float circularDiff(float a, float b);

private:
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
    void pushStability(float az, float inc);

    /// Circular EMA for azimuth (handles 0/360 wraparound)
    static float circularEma(float prev, float next, float alpha);
};
