#include "sensor_manager.h"
#include <cmath>

// ── Initialisation ──────────────────────────────────────────────────

void SensorManager::init(const MagCal::Calibration* cal,
                         float emaAlpha, uint8_t stabilityLen) {
    cal_       = cal;
    emaAlpha_  = emaAlpha;
    stabLen_   = (stabilityLen > MAX_STAB_BUF) ? MAX_STAB_BUF : stabilityLen;
    resetStability();

    filtGrav_   = Eigen::Vector3f(0.0f, 0.0f, 9.81f);
    gyroBias_   = Eigen::Vector3f::Zero();
    seeded_     = false;
    emaSeeded_  = false;
    emaAz_      = 0.0f;
    emaInc_     = 0.0f;
    roll_       = 0.0f;
}

// ── Main update ─────────────────────────────────────────────────────

void SensorManager::update(const Eigen::Vector3f& rawMag,
                           const Eigen::Vector3f& rawAccel,
                           const Eigen::Vector3f& rawGyro,
                           float dt) {
    if (!cal_ || dt <= 0.0f) return;

    // 1) Complementary filter: smooth the gravity vector using gyro
    complementaryGravFilter(rawAccel, rawGyro, dt);

    // 2) Compute angles directly from calibrated mag + filtered gravity
    //    calibration.getAngles() does: axis remap, ellipsoid correction,
    //    orientation matrix, ZXY Euler extraction
    MagCal::Angles a = cal_->getAngles(rawMag, filtGrav_);

    // 3) EMA smooth the output angles
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

    // 4) Push into stability ring buffer
    pushStability(emaAz_, emaInc_);
}

// ── Complementary gravity filter ────────────────────────────────────
// Gyro predicts gravity rotation, accelerometer corrects.
// Dynamic alpha: when still, trust gyro prediction; when moving, trust accel.
// Matches Python sensor_manager.py get_grav().

void SensorManager::complementaryGravFilter(const Eigen::Vector3f& rawAccel,
                                             const Eigen::Vector3f& rawGyro,
                                             float dt) {
    // Seed on first call
    if (!seeded_) {
        filtGrav_ = rawAccel;
        seeded_ = true;
        return;
    }

    // Subtract estimated gyro bias
    float wx = rawGyro.x() - gyroBias_.x();
    float wy = rawGyro.y() - gyroBias_.y();
    float wz = rawGyro.z() - gyroBias_.z();

    // Gyro magnitude (for dynamic alpha + bias learning)
    float gyroMag = sqrtf(wx * wx + wy * wy + wz * wz);

    // Gyro-predict: rotate previous gravity estimate by angular velocity
    // v_new ≈ v + (omega × v) * dt
    float gx = filtGrav_.x();
    float gy = filtGrav_.y();
    float gz = filtGrav_.z();
    float predX = gx + (wy * gz - wz * gy) * dt;
    float predY = gy + (wz * gx - wx * gz) * dt;
    float predZ = gz + (wx * gy - wy * gx) * dt;

    // Dynamic alpha based on gyro magnitude
    float alpha;
    if (gyroMag <= GYRO_THRESH_LOW) {
        alpha = ALPHA_LOW;
    } else if (gyroMag >= GYRO_THRESH_HIGH) {
        alpha = ALPHA_HIGH;
    } else {
        float t = (gyroMag - GYRO_THRESH_LOW) / (GYRO_THRESH_HIGH - GYRO_THRESH_LOW);
        alpha = ALPHA_LOW + t * (ALPHA_HIGH - ALPHA_LOW);
    }

    // Blend: alpha * raw_accel + (1 - alpha) * gyro_prediction
    filtGrav_.x() = alpha * rawAccel.x() + (1.0f - alpha) * predX;
    filtGrav_.y() = alpha * rawAccel.y() + (1.0f - alpha) * predY;
    filtGrav_.z() = alpha * rawAccel.z() + (1.0f - alpha) * predZ;

    // Online gyro bias learning when stationary
    if (gyroMag <= GYRO_THRESH_LOW) {
        gyroBias_.x() += BIAS_LEARN_RATE * (rawGyro.x() - gyroBias_.x());
        gyroBias_.y() += BIAS_LEARN_RATE * (rawGyro.y() - gyroBias_.y());
        gyroBias_.z() += BIAS_LEARN_RATE * (rawGyro.z() - gyroBias_.z());
    }
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
