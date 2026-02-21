#include "mag_cal/sensor.h"
#include "mag_cal/utils.h"
#include <math.h>

namespace MagCal {

Sensor::Sensor(const char* axesStr)
    : axes_(axesStr)
    , transform_(Eigen::Matrix3f::Identity())
    , centre_(Eigen::Vector3f::Zero())
    , hasRbfs_(false)
    , fieldAvg_(0.0f)
    , fieldStd_(0.0f)
    , calibrated_(false)
{}

Eigen::Vector3f Sensor::apply(const Eigen::Vector3f& raw) const {
    // 1. Fix axes: permute and flip from sensor coords to device coords
    Eigen::Vector3f data = axes_.fixAxes(raw);

    // 2. Subtract centre (hard-iron bias)
    data -= centre_;

    // 3. Apply non-linear RBF correction if present
    if (hasRbfs_) {
        data = applyNonLinear(data);
    }

    // 4. Apply linear transform (scale + soft-iron correction)
    // Python: np.dot(data.reshape((1,3)), self.transform)[0]
    // This is row-vector × matrix, equivalent to transform^T * data in column-vector convention
    data = transform_.transpose() * data;

    return data;
}

Eigen::Vector3f Sensor::applyNonLinear(const Eigen::Vector3f& vec) const {
    // Python: scale = self.transform[0,0]
    //         normalised_v = vectors * scale
    //         vectors[i] += self.rbfs[i](normalised_v[i]) / scale
    Eigen::Vector3f result = vec;
    float scale = transform_(0, 0);
    for (int i = 0; i < 3; i++) {
        if (rbfs_[i].isValid()) {
            float normVal = vec[i] * scale;
            result[i] += rbfs_[i].evaluate(normVal) / scale;
        }
    }
    return result;
}

float Sensor::getFieldStrength(const Eigen::Vector3f& raw) const {
    Eigen::Vector3f corrected = apply(raw);
    // Convert back to original units using mean of diagonal
    float meanDiag = transform_.diagonal().mean();
    Eigen::Vector3f scaled = corrected / meanDiag;
    return scaled.norm();
}

bool Sensor::checkAnomaly(const Eigen::Vector3f& raw, float tolerance) const {
    if (fieldAvg_ == 0.0f) return false;  // no reference data
    float strength = getFieldStrength(raw);
    float acceptable = fmaxf(fieldStd_ * 3.0f, fieldAvg_ * tolerance);
    float variation = fabsf(fieldAvg_ - strength);
    return variation > acceptable;
}

bool Sensor::fromJson(JsonObjectConst dict) {
    // axes
    const char* axesStr = dict["axes"] | "+X+Y+Z";
    axes_ = Axes(axesStr);

    // transform: 3x3 matrix
    JsonArrayConst tArr = dict["transform"];
    if (!tArr || tArr.size() != 3) return false;
    for (int r = 0; r < 3; r++) {
        JsonArrayConst row = tArr[r];
        if (!row || row.size() != 3) return false;
        for (int c = 0; c < 3; c++) {
            transform_(r, c) = row[c].as<float>();
        }
    }

    // centre: array of 3
    JsonArrayConst cArr = dict["centre"];
    if (!cArr || cArr.size() != 3) return false;
    for (int i = 0; i < 3; i++) {
        centre_[i] = cArr[i].as<float>();
    }

    // rbfs: array of 3 arrays (one per axis), each containing nested param arrays
    JsonArrayConst rbfArr = dict["rbfs"];
    hasRbfs_ = false;
    if (rbfArr && rbfArr.size() == 3) {
        for (int axis = 0; axis < 3; axis++) {
            JsonArrayConst axisParams = rbfArr[axis];
            if (axisParams && axisParams.size() > 0) {
                // Each param is a nested array like [[0.003298], [-0.000434], ...]
                float params[RBF::MAX_PARAMS];
                int count = 0;
                for (size_t j = 0; j < axisParams.size() && count < RBF::MAX_PARAMS; j++) {
                    JsonArrayConst nested = axisParams[j];
                    if (nested && nested.size() > 0) {
                        params[count++] = nested[0].as<float>();
                    }
                }
                if (count > 0) {
                    rbfs_[axis].init(params, count);
                    // Check if all params are zero (Y-axis in mag has all zeros)
                    bool allZero = true;
                    for (int k = 0; k < count; k++) {
                        if (params[k] != 0.0f) { allZero = false; break; }
                    }
                    if (!allZero) hasRbfs_ = true;
                }
            }
        }
    }

    // field statistics
    fieldAvg_ = dict["field_avg"] | 0.0f;
    fieldStd_ = dict["field_std"] | 0.0f;

    calibrated_ = true;
    return true;
}

void Sensor::toJson(JsonObject dict) const {
    dict["axes"] = axes_.toString();

    // transform
    JsonArray tArr = dict["transform"].to<JsonArray>();
    for (int r = 0; r < 3; r++) {
        JsonArray row = tArr.add<JsonArray>();
        for (int c = 0; c < 3; c++) {
            row.add(transform_(r, c));
        }
    }

    // centre
    JsonArray cArr = dict["centre"].to<JsonArray>();
    for (int i = 0; i < 3; i++) {
        cArr.add(centre_[i]);
    }

    // rbfs: Python format is [[[p0],[p1],...], [[p0],...], [[p0],...]]
    JsonArray rbfArr = dict["rbfs"].to<JsonArray>();
    for (int axis = 0; axis < 3; axis++) {
        JsonArray axisArr = rbfArr.add<JsonArray>();
        if (rbfs_[axis].isValid()) {
            const float* p = rbfs_[axis].params();
            for (int j = 0; j < rbfs_[axis].paramCount(); j++) {
                JsonArray nested = axisArr.add<JsonArray>();
                nested.add(p[j]);
            }
        }
    }

    dict["field_avg"] = fieldAvg_;
    dict["field_std"] = fieldStd_;
}

// ── Fitting methods (Session 11) ────────────────────────────────────

float Sensor::fitEllipsoid(const std::vector<Eigen::Vector3f>& data) {
    // Port of Python sensor.py fit_ellipsoid()
    // Fit ax² + by² + cz² + 2dxy + 2exz + 2fyz + 2gx + 2hy + 2iz = 1
    const int N = (int)data.size();

    // Fix axes on all data
    std::vector<Eigen::Vector3f> fixed(N);
    for (int i = 0; i < N; i++) {
        fixed[i] = axes_.fixAxes(data[i]);
    }

    // Build (N, 9) design matrix and (N, 1) output vector of ones
    Eigen::MatrixXf A(N, 9);
    Eigen::VectorXf b = Eigen::VectorXf::Ones(N);

    for (int i = 0; i < N; i++) {
        float x = fixed[i][0], y = fixed[i][1], z = fixed[i][2];
        A(i, 0) = x * x;
        A(i, 1) = y * y;
        A(i, 2) = z * z;
        A(i, 3) = 2.0f * x * y;
        A(i, 4) = 2.0f * x * z;
        A(i, 5) = 2.0f * y * z;
        A(i, 6) = 2.0f * x;
        A(i, 7) = 2.0f * y;
        A(i, 8) = 2.0f * z;
    }

    // Solve least squares: A * coeff = b
    Eigen::VectorXf coeff = A.colPivHouseholderQr().solve(b);

    float a = coeff[0], bv = coeff[1], c = coeff[2];
    float d = coeff[3], e = coeff[4], f = coeff[5];
    float g = coeff[6], h = coeff[7], iv = coeff[8];

    // Build 4x4 matrix A4
    Eigen::Matrix4f A4;
    A4 << a,  d,  e,  g,
          d,  bv, f,  h,
          e,  f,  c,  iv,
          g,  h,  iv, -1.0f;

    // Extract A3 (3x3 upper-left)
    Eigen::Matrix3f A3 = A4.block<3, 3>(0, 0);

    // Compute centre: solve A3 * centre = [-g, -h, -i]
    Eigen::Vector3f rhs(-g, -h, -iv);
    centre_ = A3.colPivHouseholderQr().solve(rhs);

    // Build T matrix for similarity transform
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T(3, 0) = centre_[0];
    T(3, 1) = centre_[1];
    T(3, 2) = centre_[2];

    // B4 = T * A4 * T^T
    Eigen::Matrix4f B4 = T * A4 * T.transpose();

    // B3 = B4[0:3,0:3] / -B4[3,3]
    Eigen::Matrix3f B3 = B4.block<3, 3>(0, 0) / (-B4(3, 3));

    // Eigendecomposition of B3 (symmetric positive definite)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(B3);
    Eigen::Vector3f eigenValues = solver.eigenvalues();
    Eigen::Matrix3f eigenVectors = solver.eigenvectors();

    // transform = V * sqrt(diag(eigenvalues)) * V^T
    Eigen::Matrix3f sqrtDiag = Eigen::Matrix3f::Zero();
    for (int i = 0; i < 3; i++) {
        sqrtDiag(i, i) = sqrtf(fabsf(eigenValues[i]));
    }
    transform_ = eigenVectors * sqrtDiag * eigenVectors.transpose();

    calibrated_ = true;
    hasRbfs_ = false;

    return uniformity(data);
}

void Sensor::alignAlongAxis(const std::vector<std::vector<Eigen::Vector3f>>& datasets,
                            char axis) {
    // Port of Python sensor.py align_along_axis()
    int axisIdx = -1;
    if (axis == 'X') axisIdx = 0;
    else if (axis == 'Y') axisIdx = 1;
    else if (axis == 'Z') axisIdx = 2;
    if (axisIdx < 0) return;

    Eigen::Vector3f result = Eigen::Vector3f::Zero();

    for (const auto& points : datasets) {
        Eigen::Vector3f vec = findPlane(points);
        if (vec[axisIdx] < 0.0f) {
            vec = -vec;
        }
        result += vec;
    }

    alignToVector(result, axis);
}

Eigen::Vector3f Sensor::findPlane(const std::vector<Eigen::Vector3f>& data) const {
    // Port of Python sensor.py _find_plane()
    // Apply calibration to data, then solve: calibrated * normal = 1
    const int N = (int)data.size();
    Eigen::MatrixXf A(N, 3);
    Eigen::VectorXf b = Eigen::VectorXf::Ones(N);

    for (int i = 0; i < N; i++) {
        Eigen::Vector3f cal = apply(data[i]);
        A(i, 0) = cal[0];
        A(i, 1) = cal[1];
        A(i, 2) = cal[2];
    }

    Eigen::Vector3f normal = A.colPivHouseholderQr().solve(b);
    return normalize(normal);
}

void Sensor::alignToVector(const Eigen::Vector3f& vector, char axis) {
    // Port of Python sensor.py _align_to_vector()
    // Build orthonormal basis with one axis aligned to the given vector
    Eigen::Vector3f vx, vy, vz;

    if (axis == 'X') {
        vx = vector;
        vz = normalize(vx.cross(Eigen::Vector3f(0, 1, 0)));
        vy = normalize(vz.cross(vx));
    } else if (axis == 'Y') {
        vy = vector;
        vx = normalize(vy.cross(Eigen::Vector3f(0, 0, 1)));
        vz = normalize(vx.cross(vy));
    } else { // Z
        vz = vector;
        vy = normalize(vz.cross(Eigen::Vector3f(1, 0, 0)));
        vx = normalize(vy.cross(vz));
    }

    vx = normalize(vx);
    vy = normalize(vy);
    vz = normalize(vz);

    // mat = [vx; vy; vz] as rows → (3,3)
    Eigen::Matrix3f mat;
    mat.row(0) = vx;
    mat.row(1) = vy;
    mat.row(2) = vz;

    // Python: self.transform = np.dot(mat.transpose(), self.transform)
    transform_ = mat.transpose() * transform_;
}

void Sensor::setNonLinearParams(const float* params, int totalCount) {
    // Port of Python sensor.py set_non_linear_params()
    // params has 3 * paramCount elements: [x0,x1,...,y0,y1,...,z0,z1,...]
    int paramCount = totalCount / 3;
    if (paramCount <= 0) { setLinear(); return; }

    hasRbfs_ = false;
    for (int ax = 0; ax < 3; ax++) {
        rbfs_[ax].init(params + ax * paramCount, paramCount);
        // Check if all zero
        for (int j = 0; j < paramCount; j++) {
            if (params[ax * paramCount + j] != 0.0f) {
                hasRbfs_ = true;
                break;
            }
        }
    }
}

void Sensor::setLinear() {
    hasRbfs_ = false;
    for (int i = 0; i < 3; i++) {
        float zero = 0.0f;
        rbfs_[i].init(&zero, 0);
    }
}

float Sensor::uniformity(const std::vector<Eigen::Vector3f>& data) const {
    // Port of Python sensor.py uniformity()
    // Std dev of radii after calibration (mean should be ~1.0)
    const int N = (int)data.size();
    if (N == 0) return 0.0f;

    float sumSqDiff = 0.0f;
    for (int i = 0; i < N; i++) {
        float radius = apply(data[i]).norm();
        float diff = radius - 1.0f;
        sumSqDiff += diff * diff;
    }
    return sqrtf(sumSqDiff / N);
}

void Sensor::setExpectedFieldStrengths(const std::vector<Eigen::Vector3f>& data) {
    // Port of Python sensor.py set_expected_field_strengths()
    const int N = (int)data.size();
    if (N == 0) return;

    float sum = 0.0f;
    std::vector<float> strengths(N);
    for (int i = 0; i < N; i++) {
        strengths[i] = getFieldStrength(data[i]);
        sum += strengths[i];
    }
    fieldAvg_ = sum / N;

    float sumSqDiff = 0.0f;
    for (int i = 0; i < N; i++) {
        float diff = strengths[i] - fieldAvg_;
        sumSqDiff += diff * diff;
    }
    fieldStd_ = sqrtf(sumSqDiff / N);
}

} // namespace MagCal
