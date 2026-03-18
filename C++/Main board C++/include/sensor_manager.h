#pragma once
// Sensor fusion: direct angle calculation from mag + accel, EMA output smoothing.

#include <ArduinoEigenDense.h>
#include "mag_cal/calibration.h"

class SensorManager {
public:
    /// Initialise with calibration reference and tuning parameters
    void init(const MagCal::Calibration* cal,
              float emaAlphaStable, float emaAlphaMoving,
              uint8_t stabilityLen);

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
    float emaAlphaStable_ = 0.05f;
    float emaAlphaMoving_ = 0.3f;
    bool emaSeeded_ = false;

    // Stability ring buffer
    static constexpr uint8_t MAX_STAB_BUF = 8;
    float azBuf_[MAX_STAB_BUF]  = {};
    float incBuf_[MAX_STAB_BUF] = {};
    uint8_t stabHead_  = 0;
    uint8_t stabCount_ = 0;
    uint8_t stabLen_   = 3;

    // Median pre-filter (5-sample, kills up to 2 consecutive spikes)
    static constexpr uint8_t MEDIAN_LEN = 5;
    float medAzBuf_[MEDIAN_LEN]  = {};
    float medIncBuf_[MEDIAN_LEN] = {};
    uint8_t medHead_  = 0;
    uint8_t medCount_ = 0;

    // Calibration (not owned)
    const MagCal::Calibration* cal_ = nullptr;

    // ── Internal helpers ─────────────────────────────────────────────
    void pushStability(float az, float inc);

    /// Push raw angles into median buffer; returns true when buffer is full
    bool pushMedian(float az, float inc);

    /// Get median azimuth from buffer (circular-aware)
    float medianAzimuth() const;

    /// Get median inclination from buffer
    float medianInclination() const;

    /// Median of a small array (sorts in place)
    static float medianN(float* vals, uint8_t n);

    /// Circular EMA for azimuth (handles 0/360 wraparound)
    static float circularEma(float prev, float next, float alpha);
};
