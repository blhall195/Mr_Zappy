#pragma once
// Calibration math utilities
// Session 6: normalize helper for Eigen vectors

#include <ArduinoEigenDense.h>

namespace MagCal {

/// Normalize a 3D vector to unit length. Returns zero vector if input is near-zero.
inline Eigen::Vector3f normalize(const Eigen::Vector3f& v) {
    float n = v.norm();
    if (n < 1e-10f) return Eigen::Vector3f::Zero();
    return v / n;
}

} // namespace MagCal
