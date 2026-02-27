#pragma once
// Combined magnetometer + accelerometer calibration
// Session 6: port of Python mag_cal/calibration.py + calibrate_roll.py
// Session 11: added fitting methods, findSimilarShots, alignSensorRoll

#include <ArduinoEigenDense.h>
#include <ArduinoJson.h>
#include <vector>
#include <utility>
#include "mag_cal/sensor.h"

namespace MagCal {

// Convenience type aliases for paired calibration data
using DataPair = std::pair<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>>;
using PairedData = std::vector<DataPair>;

struct Angles {
    float azimuth;      // degrees, [0, 360)
    float inclination;  // degrees, [-90, 90)
    float roll;         // degrees
};

struct Strictness {
    float mag;   // percentage tolerance for magnetic field strength
    float grav;  // percentage tolerance for gravity field strength
    float dip;   // degrees tolerance for dip angle
};

enum class AnomalyType : uint8_t {
    NONE,
    MAGNETIC,
    GRAVITY,
    DIP
};

// ── Binary calibration format (fast boot) ───────────────────────────
static constexpr uint32_t CAL_BINARY_MAGIC   = 0x43414C00; // "CAL\0"
static constexpr uint8_t  CAL_BINARY_VERSION = 1;

#pragma pack(push, 1)
struct CalibrationBinary {
    uint32_t magic;
    uint8_t  version;
    uint8_t  reserved;
    SensorBinary mag;
    SensorBinary grav;
    float    dipAvg;
    uint16_t crc16;
};
#pragma pack(pop)

class Calibration {
public:
    static constexpr Strictness DEFAULT_STRICTNESS = {2.0f, 2.0f, 3.0f};

    Calibration(const char* magAxes = "+X+Y+Z", const char* gravAxes = nullptr);

    // ── Real-time angle calculation (no allocation) ──

    /// Get azimuth, inclination, roll from raw sensor readings
    Angles getAngles(const Eigen::Vector3f& mag, const Eigen::Vector3f& grav) const;

    /// Get 3x3 orientation matrix (rows = east, north, upward)
    Eigen::Matrix3f getOrientationMatrix(const Eigen::Vector3f& mag,
                                         const Eigen::Vector3f& grav) const;

    /// Extract ZXY rotation angles from orientation matrix
    static Angles matrixToAngles(const Eigen::Matrix3f& matrix);

    /// Build rotation matrix from angles (inverse of matrixToAngles)
    static Eigen::Matrix3f anglesToMatrix(float azimuth, float inclination, float roll);

    // ── Anomaly detection ──

    /// Get magnetic dip angle in degrees
    float getDip(const Eigen::Vector3f& mag, const Eigen::Vector3f& grav) const;

    /// Check all anomaly conditions. Returns NONE if everything is OK.
    AnomalyType checkAnomaly(const Eigen::Vector3f& mag, const Eigen::Vector3f& grav,
                             const Strictness& s = DEFAULT_STRICTNESS) const;

    // ── Fitting methods (Session 11 — use dynamic allocation) ──

    /// Fit ellipsoid to both sensors. Returns (mag_uniformity, grav_uniformity).
    std::pair<float, float> fitEllipsoid(
        const std::vector<Eigen::Vector3f>& magData,
        const std::vector<Eigen::Vector3f>& gravData);

    /// Align both sensors from paired orientation datasets.
    /// Returns accuracy (degrees) after alignment.
    float fitToAxis(const PairedData& pairedData, char axis = 'Y');

    /// Fast non-linear RBF correction using least-squares (magnetometer only).
    /// Returns accuracy (degrees).
    float fitNonLinearQuick(const PairedData& pairedData, int paramCount = 3);

    /// Measure calibration accuracy: avg std dev of orientation in degrees.
    float accuracy(const PairedData& pairedData) const;

    /// Measure uniformity of both sensors. Returns (mag, grav).
    std::pair<float, float> uniformity(
        const std::vector<Eigen::Vector3f>& magData,
        const std::vector<Eigen::Vector3f>& gravData) const;

    /// Store field strengths + dip angle statistics for anomaly detection.
    void setFieldCharacteristics(
        const std::vector<Eigen::Vector3f>& magData,
        const std::vector<Eigen::Vector3f>& gravData);

    /// Find runs of shots pointing in similar directions.
    /// Returns vector of (start, end) index pairs.
    std::vector<std::pair<int, int>> findSimilarShots(
        const std::vector<Eigen::Vector3f>& magData,
        const std::vector<Eigen::Vector3f>& gravData,
        float precision = 30.0f, int minRun = 4) const;

    /// Roll alignment: adjust mag transform so magnetic dip is constant.
    /// Port of calibrate_roll.py's align_sensor_roll.
    void alignSensorRoll(const std::vector<Eigen::Vector3f>& magData,
                         const std::vector<Eigen::Vector3f>& gravData);

    // ── Serialization ──

    /// Parse calibration from a JSON string (e.g. calibration_dict.json)
    bool fromJson(const char* json, size_t len);

    /// Parse calibration from an already-parsed JSON object
    bool fromJson(JsonObjectConst dict);

    /// Write calibration to a JSON object
    void toJson(JsonObject dict) const;

    /// Load calibration from binary struct (fast boot path)
    bool fromBinary(const CalibrationBinary& bin);

    /// Save calibration to binary struct
    void toBinary(CalibrationBinary& bin) const;

    // ── Accessors ──

    Sensor& mag() { return mag_; }
    Sensor& grav() { return grav_; }
    const Sensor& mag() const { return mag_; }
    const Sensor& grav() const { return grav_; }
    float dipAvg() const { return dipAvg_; }
    bool isCalibrated() const { return mag_.isCalibrated() && grav_.isCalibrated(); }

private:
    Sensor mag_;
    Sensor grav_;
    float dipAvg_;

    // ── Fitting helpers ──

    /// Get orientation vector(s) for accuracy calculation
    Eigen::Vector3f getOrientationVector(const Eigen::Vector3f& mag,
                                         const Eigen::Vector3f& grav) const;

    /// Get dip for a set of readings
    float getDipBatch(const std::vector<Eigen::Vector3f>& magData,
                      const std::vector<Eigen::Vector3f>& gravData) const;

    /// Check if a sequence of azimuths/inclinations forms a run
    static bool isARun(const std::vector<float>& azimuths,
                       const std::vector<float>& inclinations,
                       float precision);

    /// Get raw and expected mag data for non-linear quick fitting
    void getRawAndExpectedMagData(
        const std::vector<Eigen::Vector3f>& mag,
        const std::vector<Eigen::Vector3f>& grav,
        std::vector<Eigen::Vector3f>& expected,
        std::vector<Eigen::Vector3f>& raw) const;

    /// Solve for non-linear RBF params using least-squares
    std::vector<float> getLstsqNonLinearParams(
        int paramCount,
        const std::vector<Eigen::Vector3f>& expectedMags,
        const std::vector<Eigen::Vector3f>& rawMags) const;
};

} // namespace MagCal
