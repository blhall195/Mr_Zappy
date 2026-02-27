#pragma once
// Single-sensor calibration (magnetometer or accelerometer)
// Session 6: port of Python mag_cal/sensor.py
// Session 11: added fitting methods (fitEllipsoid, alignAlongAxis, etc.)

#include <ArduinoEigenDense.h>
#include <ArduinoJson.h>
#include <vector>
#include "mag_cal/axes.h"
#include "mag_cal/rbf.h"

namespace MagCal {

#pragma pack(push, 1)
struct SensorBinary {
    char axes[7];                          // 6 chars + null
    float transform[9];                    // row-major 3x3
    float centre[3];
    uint8_t rbfParamCount[3];             // one per axis
    float rbfParams[3][RBF::MAX_PARAMS];  // 3 axes × 7 floats
    float fieldAvg;
    float fieldStd;
};
#pragma pack(pop)

class Sensor {
public:
    explicit Sensor(const char* axesStr = "+X+Y+Z");

    // ── Real-time path (no allocation) ──

    /// Apply full calibration pipeline to a single raw reading
    Eigen::Vector3f apply(const Eigen::Vector3f& raw) const;

    /// Get field strength in original sensor units
    float getFieldStrength(const Eigen::Vector3f& raw) const;

    /// Check if reading is within expected field strength range
    /// Returns true if anomaly detected
    bool checkAnomaly(const Eigen::Vector3f& raw, float tolerance = 0.05f) const;

    // ── Fitting methods (Session 11 — use dynamic allocation) ──

    /// Fit an ellipsoid to raw sensor readings. Sets transform_ and centre_.
    /// Returns uniformity metric (lower is better).
    float fitEllipsoid(const std::vector<Eigen::Vector3f>& data);

    /// Align sensor axis from multiple orientation datasets.
    /// Each dataset = readings taken pointing one direction with varying roll.
    void alignAlongAxis(const std::vector<std::vector<Eigen::Vector3f>>& datasets,
                        char axis = 'Y');

    /// Set RBF params from flat array (3 axes × paramCount params)
    void setNonLinearParams(const float* params, int totalCount);

    /// Clear non-linear RBFs
    void setLinear();

    /// Measure how well calibrated data fits on unit sphere
    float uniformity(const std::vector<Eigen::Vector3f>& data) const;

    /// Compute and store expected field strength statistics
    void setExpectedFieldStrengths(const std::vector<Eigen::Vector3f>& data);

    // ── Serialization ──

    /// Load calibration from JSON object
    bool fromJson(JsonObjectConst dict);

    /// Save calibration to JSON object
    void toJson(JsonObject dict) const;

    /// Load calibration from binary struct
    bool fromBinary(const SensorBinary& bin);

    /// Save calibration to binary struct
    void toBinary(SensorBinary& bin) const;

    // ── Accessors ──

    bool isCalibrated() const { return calibrated_; }
    const Axes& axes() const { return axes_; }
    const Eigen::Matrix3f& transform() const { return transform_; }
    Eigen::Matrix3f& transformRef() { return transform_; }
    float fieldAvg() const { return fieldAvg_; }
    float fieldStd() const { return fieldStd_; }

private:
    Axes axes_;
    Eigen::Matrix3f transform_;
    Eigen::Vector3f centre_;
    RBF rbfs_[3];          // one per axis (X, Y, Z)
    bool hasRbfs_;
    float fieldAvg_;
    float fieldStd_;
    bool calibrated_;

    Eigen::Vector3f applyNonLinear(const Eigen::Vector3f& vec) const;

    // ── Fitting helpers ──

    /// Fit a plane to calibrated data points. Returns plane normal (unit vector).
    /// Session 17: returns double for fitting precision
    Eigen::Vector3d findPlane(const std::vector<Eigen::Vector3f>& data) const;

    /// Rotate transform so that points along vector land on the given axis.
    /// Session 17: takes double vector for fitting precision
    void alignToVector(const Eigen::Vector3d& vector, char axis);
};

} // namespace MagCal
