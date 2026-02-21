#include "mag_cal/calibration.h"
#include "mag_cal/utils.h"
#include "mag_cal/rbf.h"
#include <math.h>
#include <algorithm>

namespace MagCal {

static constexpr float RAD2DEG = 180.0f / M_PI;
static constexpr float DEG2RAD = M_PI / 180.0f;

// C++14 requires out-of-line definition for ODR-used constexpr static members
constexpr Strictness Calibration::DEFAULT_STRICTNESS;

Calibration::Calibration(const char* magAxes, const char* gravAxes)
    : mag_(magAxes)
    , grav_(gravAxes ? gravAxes : magAxes)
    , dipAvg_(0.0f)
{}

// ── Real-time angle calculation ──

Angles Calibration::getAngles(const Eigen::Vector3f& mag, const Eigen::Vector3f& grav) const {
    Eigen::Matrix3f matrix = getOrientationMatrix(mag, grav);
    return matrixToAngles(matrix);
}

Eigen::Matrix3f Calibration::getOrientationMatrix(
    const Eigen::Vector3f& mag, const Eigen::Vector3f& grav) const
{
    // Apply calibration and normalize
    Eigen::Vector3f magCal = normalize(mag_.apply(mag));
    Eigen::Vector3f upward = normalize(grav_.apply(grav)) * -1.0f;

    // Build orthonormal basis
    Eigen::Vector3f east  = normalize(magCal.cross(upward));
    Eigen::Vector3f north = normalize(upward.cross(east));

    // Pack into matrix: rows = [east, north, upward]
    Eigen::Matrix3f orientation;
    orientation.row(0) = east;
    orientation.row(1) = north;
    orientation.row(2) = upward;
    return orientation;
}

Angles Calibration::matrixToAngles(const Eigen::Matrix3f& matrix) {
    // ZXY rotation extraction
    // matrix layout: row 0 = east, row 1 = north, row 2 = upward
    float m01 = matrix(0, 1);
    float m11 = matrix(1, 1);
    float m21 = matrix(2, 1);
    float m20 = matrix(2, 0);
    float m22 = matrix(2, 2);

    float theta1 = atan2f(m01, m11);            // azimuth
    float theta2 = atan2f(m21 * cosf(theta1), m11); // inclination
    float theta3 = atan2f(-m20, m22);            // roll

    Angles a;
    a.azimuth = fmodf(theta1 * RAD2DEG + 360.0f, 360.0f);
    a.inclination = fmodf(theta2 * RAD2DEG + 90.0f + 360.0f, 180.0f) - 90.0f;
    a.roll = theta3 * RAD2DEG;
    return a;
}

Eigen::Matrix3f Calibration::anglesToMatrix(float azimuth, float inclination, float roll) {
    float t1 = -azimuth * DEG2RAD;
    float t2 = inclination * DEG2RAD;
    float t3 = roll * DEG2RAD;
    float c1 = cosf(t1), s1 = sinf(t1);
    float c2 = cosf(t2), s2 = sinf(t2);
    float c3 = cosf(t3), s3 = sinf(t3);

    Eigen::Matrix3f m;
    m(0, 0) = c1*c3 - s1*s2*s3;   m(0, 1) = -c2*s1;   m(0, 2) = c1*s3 + c3*s1*s2;
    m(1, 0) = c3*s1 + c1*s2*s3;   m(1, 1) =  c1*c2;   m(1, 2) = s1*s3 - c1*c3*s2;
    m(2, 0) = -c2*s3;             m(2, 1) =  s2;       m(2, 2) = c2*c3;
    return m;
}

// ── Anomaly detection ──

float Calibration::getDip(const Eigen::Vector3f& mag, const Eigen::Vector3f& grav) const {
    Eigen::Vector3f magNorm  = normalize(mag_.apply(mag));
    Eigen::Vector3f gravNorm = normalize(grav_.apply(grav));
    float dot = magNorm.dot(gravNorm);
    // Clamp to [-1,1] for numerical safety
    dot = fmaxf(-1.0f, fminf(1.0f, dot));
    return 90.0f - acosf(dot) * RAD2DEG;
}

AnomalyType Calibration::checkAnomaly(
    const Eigen::Vector3f& mag, const Eigen::Vector3f& grav,
    const Strictness& s) const
{
    // Check magnetic field strength
    if (mag_.checkAnomaly(mag, s.mag / 100.0f)) {
        return AnomalyType::MAGNETIC;
    }

    // Check gravity field strength
    if (grav_.checkAnomaly(grav, s.grav / 100.0f)) {
        return AnomalyType::GRAVITY;
    }

    // Check dip angle
    if (dipAvg_ != 0.0f) {
        float dip = getDip(mag, grav);
        if (fabsf(dipAvg_ - dip) > s.dip) {
            return AnomalyType::DIP;
        }
    }

    return AnomalyType::NONE;
}

// ── Serialization ──

bool Calibration::fromJson(const char* json, size_t len) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json, len);
    if (err) return false;
    return fromJson(doc.as<JsonObjectConst>());
}

bool Calibration::fromJson(JsonObjectConst dict) {
    JsonObjectConst magDict = dict["mag"];
    JsonObjectConst gravDict = dict["grav"];
    if (!magDict || !gravDict) return false;

    if (!mag_.fromJson(magDict)) return false;
    if (!grav_.fromJson(gravDict)) return false;

    dipAvg_ = dict["dip_avg"] | 0.0f;
    return true;
}

void Calibration::toJson(JsonObject dict) const {
    JsonObject magObj = dict["mag"].to<JsonObject>();
    mag_.toJson(magObj);

    JsonObject gravObj = dict["grav"].to<JsonObject>();
    grav_.toJson(gravObj);

    dict["dip_avg"] = dipAvg_;
}

// ── Fitting methods (Session 11) ────────────────────────────────────

std::pair<float, float> Calibration::fitEllipsoid(
    const std::vector<Eigen::Vector3f>& magData,
    const std::vector<Eigen::Vector3f>& gravData)
{
    float magAcc = mag_.fitEllipsoid(magData);
    float gravAcc = grav_.fitEllipsoid(gravData);
    return {magAcc, gravAcc};
}

float Calibration::fitToAxis(const PairedData& pairedData, char axis) {
    // Port of Python calibration.py fit_to_axis()
    std::vector<std::vector<Eigen::Vector3f>> magSets, gravSets;
    for (const auto& pair : pairedData) {
        magSets.push_back(pair.first);
        gravSets.push_back(pair.second);
    }
    mag_.alignAlongAxis(magSets, axis);
    grav_.alignAlongAxis(gravSets, axis);
    return accuracy(pairedData);
}

float Calibration::fitNonLinearQuick(const PairedData& pairedData, int paramCount) {
    // Port of Python calibration.py fit_non_linear_quick()
    mag_.setLinear();
    grav_.setLinear();

    std::vector<Eigen::Vector3f> expectedMags, rawMags;
    for (const auto& pair : pairedData) {
        std::vector<Eigen::Vector3f> exp, raw;
        getRawAndExpectedMagData(pair.first, pair.second, exp, raw);
        expectedMags.insert(expectedMags.end(), exp.begin(), exp.end());
        rawMags.insert(rawMags.end(), raw.begin(), raw.end());
    }

    std::vector<float> params = getLstsqNonLinearParams(paramCount, expectedMags, rawMags);

    // Build full param array: X params, Y zeros (rotation axis), Z params
    std::vector<float> allParams(paramCount * 3, 0.0f);
    for (int i = 0; i < paramCount; i++) {
        allParams[i] = params[i];                          // X axis
        allParams[2 * paramCount + i] = params[paramCount + i]; // Z axis
    }
    mag_.setNonLinearParams(allParams.data(), paramCount * 3);

    return accuracy(pairedData);
}

float Calibration::accuracy(const PairedData& pairedData) const {
    // Port of Python calibration.py accuracy()
    // Average std dev of orientation vectors in degrees
    if (pairedData.empty()) return 0.0f;

    float results = 0.0f;
    for (const auto& pair : pairedData) {
        const auto& magSet = pair.first;
        const auto& gravSet = pair.second;
        int N = (int)magSet.size();
        if (N == 0) continue;

        // Get orientation vectors for this set
        std::vector<Eigen::Vector3f> orientations(N);
        for (int i = 0; i < N; i++) {
            orientations[i] = getOrientationVector(magSet[i], gravSet[i]);
        }

        // Compute mean
        Eigen::Vector3f mean = Eigen::Vector3f::Zero();
        for (int i = 0; i < N; i++) mean += orientations[i];
        mean /= (float)N;

        // Compute std dev per axis
        Eigen::Vector3f sumSqDiff = Eigen::Vector3f::Zero();
        for (int i = 0; i < N; i++) {
            Eigen::Vector3f diff = orientations[i] - mean;
            sumSqDiff += diff.cwiseProduct(diff);
        }
        Eigen::Vector3f stds;
        for (int j = 0; j < 3; j++) {
            stds[j] = sqrtf(sumSqDiff[j] / (N - 1));  // ddof=1
        }
        results += stds.norm();
    }

    return results / (float)pairedData.size() * RAD2DEG;
}

std::pair<float, float> Calibration::uniformity(
    const std::vector<Eigen::Vector3f>& magData,
    const std::vector<Eigen::Vector3f>& gravData) const
{
    return {mag_.uniformity(magData), grav_.uniformity(gravData)};
}

void Calibration::setFieldCharacteristics(
    const std::vector<Eigen::Vector3f>& magData,
    const std::vector<Eigen::Vector3f>& gravData)
{
    mag_.setExpectedFieldStrengths(magData);
    grav_.setExpectedFieldStrengths(gravData);

    // Set expected mean dip
    int N = (int)magData.size();
    if (N == 0) return;
    float dipSum = 0.0f;
    for (int i = 0; i < N; i++) {
        dipSum += getDip(magData[i], gravData[i]);
    }
    dipAvg_ = dipSum / (float)N;
}

Eigen::Vector3f Calibration::getOrientationVector(
    const Eigen::Vector3f& mag, const Eigen::Vector3f& grav) const
{
    // Port of Python calibration.py get_orientation_vector()
    // Returns column 1 (Y axis = north) of orientation matrix
    Eigen::Matrix3f m = getOrientationMatrix(mag, grav);
    return Eigen::Vector3f(m(0, 1), m(1, 1), m(2, 1));
}

float Calibration::getDipBatch(
    const std::vector<Eigen::Vector3f>& magData,
    const std::vector<Eigen::Vector3f>& gravData) const
{
    int N = (int)magData.size();
    if (N == 0) return 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < N; i++) {
        sum += getDip(magData[i], gravData[i]);
    }
    return sum / (float)N;
}

std::vector<std::pair<int, int>> Calibration::findSimilarShots(
    const std::vector<Eigen::Vector3f>& magData,
    const std::vector<Eigen::Vector3f>& gravData,
    float precision, int minRun) const
{
    // Port of Python calibration.py find_similar_shots()
    int N = (int)magData.size();
    std::vector<float> azimuths(N), inclinations(N);

    for (int i = 0; i < N; i++) {
        Angles a = getAngles(magData[i], gravData[i]);
        azimuths[i] = a.azimuth;
        inclinations[i] = a.inclination;
    }

    std::vector<std::pair<int, int>> groups;
    int i = 0;
    while (i < N - minRun) {
        bool found = false;
        for (int j = N; j >= i + minRun; j--) {
            // Extract sub-ranges
            std::vector<float> azSub(azimuths.begin() + i, azimuths.begin() + j);
            std::vector<float> incSub(inclinations.begin() + i, inclinations.begin() + j);
            if (isARun(azSub, incSub, precision)) {
                groups.push_back({i, j});
                i = j;
                found = true;
                break;
            }
        }
        if (!found) i++;
    }
    return groups;
}

bool Calibration::isARun(const std::vector<float>& azimuths,
                         const std::vector<float>& inclinations,
                         float precision)
{
    // Port of Python calibration.py _is_a_run()
    float maxInc = *std::max_element(inclinations.begin(), inclinations.end());
    float minInc = *std::min_element(inclinations.begin(), inclinations.end());
    if (maxInc - minInc > precision) return false;

    std::vector<float> az = azimuths;  // copy for possible rotation
    float maxAz = *std::max_element(az.begin(), az.end());
    if (maxAz > 360.0f - precision) {
        // Rotate by 180 degrees to handle wraparound
        for (auto& a : az) {
            a = fmodf(a + 180.0f, 360.0f);
        }
    }
    maxAz = *std::max_element(az.begin(), az.end());
    float minAz = *std::min_element(az.begin(), az.end());
    return (maxAz - minAz) <= precision;
}

void Calibration::getRawAndExpectedMagData(
    const std::vector<Eigen::Vector3f>& magSet,
    const std::vector<Eigen::Vector3f>& gravSet,
    std::vector<Eigen::Vector3f>& expected,
    std::vector<Eigen::Vector3f>& raw) const
{
    // Port of Python calibration.py _get_raw_and_expected_mag_data()
    int N = (int)magSet.size();
    std::vector<Eigen::Vector3f> rotatedMags(N);
    std::vector<Eigen::Matrix3f> rotMats(N);
    raw.resize(N);

    for (int i = 0; i < N; i++) {
        Angles angles = getAngles(magSet[i], gravSet[i]);
        float rollRad = angles.roll * DEG2RAD;
        float c = cosf(rollRad), s = sinf(rollRad);

        Eigen::Matrix3f rotMat;
        rotMat << c, 0, s,
                  0, 1, 0,
                 -s, 0, c;
        rotMats[i] = rotMat;

        Eigen::Vector3f rawMag = mag_.apply(magSet[i]);
        raw[i] = rawMag;

        // Python: rotated_mag = np.dot(raw_mag.reshape((1,3)), rot_mat.transpose())
        rotatedMags[i] = rotMat.transpose() * rawMag;
    }

    // Average of rotated vectors
    Eigen::Vector3f avgVec = Eigen::Vector3f::Zero();
    for (int i = 0; i < N; i++) avgVec += rotatedMags[i];
    avgVec /= (float)N;

    // Rotate average back for each measurement
    expected.resize(N);
    for (int i = 0; i < N; i++) {
        // Python: expected = np.dot(average_vector.reshape((1,3)), rot_mat)
        expected[i] = rotMats[i] * avgVec;
    }
}

std::vector<float> Calibration::getLstsqNonLinearParams(
    int paramCount,
    const std::vector<Eigen::Vector3f>& expectedMags,
    const std::vector<Eigen::Vector3f>& rawMags) const
{
    // Port of Python calibration.py _get_lstsq_non_linear_params()
    int N = (int)expectedMags.size();
    int cols = paramCount * 2;  // X and Z axes only

    // Build least-squares matrices
    Eigen::MatrixXf inputData(N * 2, cols);
    Eigen::VectorXf outputData(N * 2);
    inputData.setZero();

    // Create a dummy RBF to get Gaussian basis functions
    std::vector<float> zeroParams(paramCount, 0.0f);
    RBF dummyRbf;
    dummyRbf.init(zeroParams.data(), paramCount);

    float gaussBuf[RBF::MAX_PARAMS];

    for (int i = 0; i < N; i++) {
        Eigen::Vector3f diff = expectedMags[i] - rawMags[i];

        // Get Gaussian basis for each axis of raw reading
        // Python: factors = rbf.get_gaussians(raw).transpose()
        // We need factors for X (index 0) and Z (index 2)
        dummyRbf.getGaussians(rawMags[i][0], gaussBuf);
        for (int j = 0; j < paramCount; j++) {
            inputData(i * 2, j) = gaussBuf[j];                // X axis → first paramCount cols
        }

        dummyRbf.getGaussians(rawMags[i][2], gaussBuf);
        for (int j = 0; j < paramCount; j++) {
            inputData(i * 2 + 1, paramCount + j) = gaussBuf[j]; // Z axis → last paramCount cols
        }

        outputData[i * 2]     = diff[0];  // X component
        outputData[i * 2 + 1] = diff[2];  // Z component
    }

    // Solve least squares
    Eigen::VectorXf params = inputData.colPivHouseholderQr().solve(outputData);

    return std::vector<float>(params.data(), params.data() + cols);
}

// ── Roll alignment (port of calibrate_roll.py) ──────────────────────

/// Build rotation matrix along axis with sin(s) and cos(c)
static Eigen::Matrix3f rot3Dsc(float s, float c, int ax) {
    Eigen::Matrix3f rot = Eigen::Matrix3f::Zero();
    rot(ax, ax) = 1.0f;
    int axp = (ax > 0) ? ax - 1 : 2;
    int axn = (ax < 2) ? ax + 1 : 0;
    rot(axp, axp) = c;
    rot(axn, axn) = c;
    rot(axp, axn) = -s;
    rot(axn, axp) = s;
    return rot;
}

void Calibration::alignSensorRoll(
    const std::vector<Eigen::Vector3f>& magData,
    const std::vector<Eigen::Vector3f>& gravData)
{
    // Port of calibrate_roll.py: calib_fit_rotM_cstdip + align_sensor_roll
    int N = (int)magData.size();

    // Apply calibration to all data
    std::vector<Eigen::Vector3f> mcorr(N), gcorr(N);
    for (int i = 0; i < N; i++) {
        mcorr[i] = mag_.apply(magData[i]);
        gcorr[i] = grav_.apply(gravData[i]);
    }

    // calib_fit_rotM_cstdip with axis=1 (Y axis)
    const int axis = 1;
    int oa0, oa1;  // other axes
    if (axis == 0)      { oa0 = 1; oa1 = 2; }
    else if (axis == 1) { oa0 = 0; oa1 = 2; }
    else                { oa0 = 0; oa1 = 1; }
    const float signs[] = {1.0f, -1.0f, -1.0f};

    Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
    float prevrotsin = -1.0f;

    for (int it = 0; it < 3; it++) {
        // m = rot * mcorr^T → then transpose back
        // For each sample: m[i] = rot * mcorr[i]
        Eigen::VectorXf xb(N);
        Eigen::VectorXf xa(N);

        for (int i = 0; i < N; i++) {
            Eigen::Vector3f m = rot * mcorr[i];
            // xb = sum(m * gcorr, axis=-1) = dot product
            xb[i] = m.dot(gcorr[i]);
            // xa = m[oa0]*g[oa1] - m[oa1]*g[oa0]
            xa[i] = m[oa0] * gcorr[i][oa1] - m[oa1] * gcorr[i][oa0];
        }

        // Build design matrix D = [xa, ones]
        Eigen::MatrixXf D(N, 2);
        for (int i = 0; i < N; i++) {
            D(i, 0) = xa[i];
            D(i, 1) = 1.0f;
        }

        // Solve D * coeffs = xb
        Eigen::Vector2f coeffs = D.colPivHouseholderQr().solve(xb);
        float s = coeffs[0];

        if (prevrotsin >= 0.0f && fabsf(s) > prevrotsin) {
            break;  // rotation should decrease each iteration
        }
        prevrotsin = fabsf(s);

        float c = sqrtf(1.0f - s * s);
        rot = rot * rot3Dsc(signs[axis] * s, c, axis);
    }

    // Apply roll correction: mag.transform = mag.transform * rot^T
    // Python: self.mag.transform = np.dot(self.mag.transform, rot.transpose())
    mag_.transformRef() = mag_.transform() * rot.transpose();
}

} // namespace MagCal
