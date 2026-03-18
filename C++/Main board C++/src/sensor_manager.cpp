#include "sensor_manager.h"
#include <cmath>

// ── Initialisation ──────────────────────────────────────────────────

void SensorManager::init(const MagCal::Calibration* cal,
                         float emaAlphaStable, float emaAlphaMoving,
                         uint8_t stabilityLen) {
    cal_             = cal;
    emaAlphaStable_  = emaAlphaStable;
    emaAlphaMoving_  = emaAlphaMoving;
    stabLen_         = (stabilityLen > MAX_STAB_BUF) ? MAX_STAB_BUF : stabilityLen;
    resetStability();

    emaSeeded_  = false;
    emaAz_      = 0.0f;
    emaInc_     = 0.0f;
    roll_       = 0.0f;
    medHead_    = 0;
    medCount_   = 0;
}

// ── Main update ─────────────────────────────────────────────────────

void SensorManager::update(const Eigen::Vector3f& rawMag,
                           const Eigen::Vector3f& rawAccel) {
    if (!cal_) return;

    // 1) Compute angles directly from calibrated mag + raw accel
    //    calibration.getAngles() does: axis remap, ellipsoid correction,
    //    orientation matrix, ZXY Euler extraction
    MagCal::Angles a = cal_->getAngles(rawMag, rawAccel);
    roll_ = a.roll;  // roll not smoothed (matches Python)

    // 2) Median pre-filter — kills single-sample spikes
    if (!pushMedian(a.azimuth, a.inclination)) return;  // buffer not full yet
    float filtAz  = medianAzimuth();
    float filtInc = medianInclination();

    // 3) EMA smooth — adaptive alpha: low when stable, high when moving
    float alpha = isStable(2.0f) ? emaAlphaStable_ : emaAlphaMoving_;
    if (!emaSeeded_) {
        emaAz_  = filtAz;
        emaInc_ = filtInc;
        emaSeeded_ = true;
    } else {
        emaAz_  = circularEma(emaAz_, filtAz, alpha);
        emaInc_ = alpha * filtInc + (1.0f - alpha) * emaInc_;
    }

    // 4) Push into stability ring buffer
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

// ── Median pre-filter ──────────────────────────────────────────────

bool SensorManager::pushMedian(float az, float inc) {
    medAzBuf_[medHead_]  = az;
    medIncBuf_[medHead_] = inc;
    medHead_ = (medHead_ + 1) % MEDIAN_LEN;
    if (medCount_ < MEDIAN_LEN) medCount_++;
    return medCount_ >= MEDIAN_LEN;
}

float SensorManager::medianInclination() const {
    float vals[MEDIAN_LEN];
    for (uint8_t i = 0; i < MEDIAN_LEN; i++) vals[i] = medIncBuf_[i];
    return medianN(vals, MEDIAN_LEN);
}

float SensorManager::medianAzimuth() const {
    // Unwrap relative to first sample to handle 0/360 boundary
    float ref = medAzBuf_[0];
    float vals[MEDIAN_LEN];
    for (uint8_t i = 0; i < MEDIAN_LEN; i++) {
        float d = medAzBuf_[i] - ref;
        if (d > 180.0f)  d -= 360.0f;
        if (d < -180.0f) d += 360.0f;
        vals[i] = ref + d;
    }
    float med = medianN(vals, MEDIAN_LEN);
    if (med < 0.0f)    med += 360.0f;
    if (med >= 360.0f) med -= 360.0f;
    return med;
}

float SensorManager::medianN(float* vals, uint8_t n) {
    // Insertion sort (tiny array)
    for (uint8_t i = 1; i < n; i++) {
        float key = vals[i];
        int8_t j = i - 1;
        while (j >= 0 && vals[j] > key) {
            vals[j + 1] = vals[j];
            j--;
        }
        vals[j + 1] = key;
    }
    return vals[n / 2];
}

// ── Helpers ─────────────────────────────────────────────────────────

float SensorManager::circularDiff(float a, float b) {
    float d = fmodf(a - b + 180.0f, 360.0f) - 180.0f;
    return fabsf(d);
}
