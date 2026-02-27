#include "sensor_manager.h"
#include <cmath>

// ── Initialisation ──────────────────────────────────────────────────

void SensorManager::init(const MagCal::Calibration* cal,
                         float emaAlpha, uint8_t stabilityLen) {
    cal_       = cal;
    emaAlpha_  = emaAlpha;
    stabLen_   = (stabilityLen > MAX_STAB_BUF) ? MAX_STAB_BUF : stabilityLen;
    resetStability();

    emaSeeded_  = false;
    emaAz_      = 0.0f;
    emaInc_     = 0.0f;
    roll_       = 0.0f;
}

// ── Main update ─────────────────────────────────────────────────────

void SensorManager::update(const Eigen::Vector3f& rawMag,
                           const Eigen::Vector3f& rawAccel) {
    if (!cal_) return;

    // 1) Compute angles directly from calibrated mag + raw accel
    //    calibration.getAngles() does: axis remap, ellipsoid correction,
    //    orientation matrix, ZXY Euler extraction
    MagCal::Angles a = cal_->getAngles(rawMag, rawAccel);

    // 2) EMA smooth the output angles
    if (!emaSeeded_) {
        emaAz_  = a.azimuth;
        emaInc_ = a.inclination;
        roll_   = a.roll;
        emaSeeded_ = true;
    } else {
        emaAz_  = circularEma(emaAz_, a.azimuth, emaAlpha_);
        emaInc_ = emaAlpha_ * a.inclination + (1.0f - emaAlpha_) * emaInc_;
        roll_   = a.roll;  // roll not smoothed (matches Python)
    }

    // 3) Push into stability ring buffer
    pushStability(emaAz_, emaInc_);
}

// ── Circular EMA for azimuth ────────────────────────────────────────
// Handles 0/360 wraparound by working in the shortest-arc direction.

float SensorManager::circularEma(float prev, float next, float alpha) {
    float diff = next - prev;
    // Wrap to [-180, 180]
    if (diff > 180.0f)  diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    float result = prev + alpha * diff;
    if (result < 0.0f)   result += 360.0f;
    if (result >= 360.0f) result -= 360.0f;
    return result;
}

// ── Stability ring buffer ───────────────────────────────────────────

void SensorManager::pushStability(float az, float inc) {
    azBuf_[stabHead_]  = az;
    incBuf_[stabHead_] = inc;
    stabHead_ = (stabHead_ + 1) % stabLen_;
    if (stabCount_ < stabLen_) stabCount_++;
}

bool SensorManager::isStable(float tolerance) const {
    if (stabCount_ < stabLen_) return false;

    // Check all pairwise circular azimuth differences
    for (uint8_t i = 0; i < stabCount_; i++) {
        for (uint8_t j = i + 1; j < stabCount_; j++) {
            if (circularDiff(azBuf_[i], azBuf_[j]) > tolerance)
                return false;
        }
    }

    // Inclination is linear — simple range check
    float incMin = incBuf_[0], incMax = incBuf_[0];
    for (uint8_t i = 1; i < stabCount_; i++) {
        if (incBuf_[i] < incMin) incMin = incBuf_[i];
        if (incBuf_[i] > incMax) incMax = incBuf_[i];
    }
    if ((incMax - incMin) > tolerance) return false;

    return true;
}

void SensorManager::resetStability() {
    stabHead_  = 0;
    stabCount_ = 0;
}

// ── Helpers ─────────────────────────────────────────────────────────

float SensorManager::circularDiff(float a, float b) {
    float d = fmodf(a - b + 180.0f, 360.0f) - 180.0f;
    return fabsf(d);
}
