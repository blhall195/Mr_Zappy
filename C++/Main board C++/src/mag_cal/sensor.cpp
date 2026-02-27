#include "mag_cal/sensor.h"
#include "mag_cal/utils.h"
#include <math.h>
#include <string.h>

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

// ── Binary serialization ─────────────────────────────────────────────

bool Sensor::fromBinary(const SensorBinary& bin) {
    axes_ = Axes(bin.axes);

    // transform: row-major flat array → Eigen Matrix3f
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            transform_(r, c) = bin.transform[r * 3 + c];

    for (int i = 0; i < 3; i++)
        centre_[i] = bin.centre[i];

    // RBFs
    hasRbfs_ = false;
    for (int axis = 0; axis < 3; axis++) {
        int count = bin.rbfParamCount[axis];
        if (count > 0 && count <= RBF::MAX_PARAMS) {
            rbfs_[axis].init(bin.rbfParams[axis], count);
            for (int k = 0; k < count; k++) {
                if (bin.rbfParams[axis][k] != 0.0f) { hasRbfs_ = true; break; }
            }
        }
    }

    fieldAvg_ = bin.fieldAvg;
    fieldStd_ = bin.fieldStd;
    calibrated_ = true;
    return true;
}

void Sensor::toBinary(SensorBinary& bin) const {
    memset(&bin, 0, sizeof(bin));

    // axes
    const char* s = axes_.toString();
    strncpy(bin.axes, s, 6);
    bin.axes[6] = '\0';

    // transform: Eigen Matrix3f → row-major flat array
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            bin.transform[r * 3 + c] = transform_(r, c);

    for (int i = 0; i < 3; i++)
        bin.centre[i] = centre_[i];

    // RBFs
    for (int axis = 0; axis < 3; axis++) {
        int count = rbfs_[axis].paramCount();
        bin.rbfParamCount[axis] = (uint8_t)count;
        if (count > 0) {
            const float* p = rbfs_[axis].params();
            for (int j = 0; j < count; j++)
                bin.rbfParams[axis][j] = p[j];
        }
    }

    bin.fieldAvg = fieldAvg_;
    bin.fieldStd = fieldStd_;
}

// ── Fitting methods (Session 11) ────────────────────────────────────

float Sensor::fitEllipsoid(const std::vector<Eigen::Vector3f>& data) {
    // Port of Python sensor.py fit_ellipsoid()
    // Session 17: double-precision fitting for numerical stability
    // Fit ax² + by² + cz² + 2dxy + 2exz + 2fyz + 2gx + 2hy + 2iz = 1
    const int N = (int)data.size();

    // Fix axes on all data (promote to double)
    std::vector<Eigen::Vector3d> fixed(N);
    for (int i = 0; i < N; i++) {
        Eigen::Vector3d v = data[i].cast<double>();
        fixed[i] = axes_.fixAxes(v);
    }

    // Build (N, 9) design matrix and (N, 1) output vector of ones
    Eigen::MatrixXd A(N, 9);
    Eigen::VectorXd b = Eigen::VectorXd::Ones(N);

    for (int i = 0; i < N; i++) {
        double x = fixed[i][0], y = fixed[i][1], z = fixed[i][2];
        A(i, 0) = x * x;
        A(i, 1) = y * y;
        A(i, 2) = z * z;
        A(i, 3) = 2.0 * x * y;
        A(i, 4) = 2.0 * x * z;
        A(i, 5) = 2.0 * y * z;
        A(i, 6) = 2.0 * x;
        A(i, 7) = 2.0 * y;
        A(i, 8) = 2.0 * z;
    }

    // Solve least squares: A * coeff = b
    Eigen::VectorXd coeff = A.colPivHouseholderQr().solve(b);

    double a = coeff[0], bv = coeff[1], c = coeff[2];
    double d = coeff[3], e = coeff[4], f = coeff[5];
    double g = coeff[6], h = coeff[7], iv = coeff[8];

    // Build 4x4 matrix A4
    Eigen::Matrix4d A4;
    A4 << a,  d,  e,  g,
          d,  bv, f,  h,
          e,  f,  c,  iv,
          g,  h,  iv, -1.0;

    // Extract A3 (3x3 upper-left)
    Eigen::Matrix3d A3 = A4.block<3, 3>(0, 0);

    // Compute centre: solve A3 * centre = [-g, -h, -i]
    Eigen::Vector3d rhs(-g, -h, -iv);
    Eigen::Vector3d centreD = A3.colPivHouseholderQr().solve(rhs);

    // Build T matrix for similarity transform
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(3, 0) = centreD[0];
    T(3, 1) = centreD[1];
    T(3, 2) = centreD[2];

    // B4 = T * A4 * T^T
    Eigen::Matrix4d B4 = T * A4 * T.transpose();

    // B3 = B4[0:3,0:3] / -B4[3,3]
    Eigen::Matrix3d B3 = B4.block<3, 3>(0, 0) / (-B4(3, 3));

    // Eigendecomposition of B3 (symmetric positive definite)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(B3);
    Eigen::Vector3d eigenValues = solver.eigenvalues();
    Eigen::Matrix3d eigenVectors = solver.eigenvectors();

    // transform = V * sqrt(diag(eigenvalues)) * V^T
    Eigen::Matrix3d sqrtDiag = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; i++) {
        sqrtDiag(i, i) = sqrt(fabs(eigenValues[i]));
    }
    Eigen::Matrix3d transformD = eigenVectors * sqrtDiag * eigenVectors.transpose();

    // Store results back as float for runtime path
    transform_ = transformD.cast<float>();
    centre_ = centreD.cast<float>();

    calibrated_ = true;
    hasRbfs_ = false;

    return uniformity(data);
}

void Sensor::alignAlongAxis(const std::vector<std::vector<Eigen::Vector3f>>& datasets,
                            char axis) {
    // Port of Python sensor.py align_along_axis()
    // Session 17: double-precision fitting
    int axisIdx = -1;
    if (axis == 'X') axisIdx = 0;
    else if (axis == 'Y') axisIdx = 1;
    else if (axis == 'Z') axisIdx = 2;
    if (axisIdx < 0) return;

    Eigen::Vector3d result = Eigen::Vector3d::Zero();

    for (const auto& points : datasets) {
        Eigen::Vector3d vec = findPlane(points);
        if (vec[axisIdx] < 0.0) {
            vec = -vec;
        }
        result += vec;
    }

    alignToVector(result, axis);
}

Eigen::Vector3d Sensor::findPlane(const std::vector<Eigen::Vector3f>& data) const {
    // Port of Python sensor.py _find_plane()
    // Session 17: double-precision fitting
    const int N = (int)data.size();
    Eigen::MatrixXd A(N, 3);
    Eigen::VectorXd b = Eigen::VectorXd::Ones(N);

    for (int i = 0; i < N; i++) {
        Eigen::Vector3f cal = apply(data[i]);
        A(i, 0) = (double)cal[0];
        A(i, 1) = (double)cal[1];
        A(i, 2) = (double)cal[2];
    }

    Eigen::Vector3d normal = A.colPivHouseholderQr().solve(b);
    return normalized(normal);
}

void Sensor::alignToVector(const Eigen::Vector3d& vector, char axis) {
    // Port of Python sensor.py _align_to_vector()
    // Session 17: double-precision fitting
    Eigen::Vector3d vx, vy, vz;

    if (axis == 'X') {
        vx = vector;
        vz = normalized(vx.cross(Eigen::Vector3d(0, 1, 0)));
        vy = normalized(vz.cross(vx));
    } else if (axis == 'Y') {
        vy = vector;
        vx = normalized(vy.cross(Eigen::Vector3d(0, 0, 1)));
        vz = normalized(vx.cross(vy));
    } else { // Z
        vz = vector;
        vy = normalized(vz.cross(Eigen::Vector3d(1, 0, 0)));
        vx = normalized(vy.cross(vz));
    }

    vx = normalized(vx);
    vy = normalized(vy);
    vz = normalized(vz);

    // mat = [vx; vy; vz] as rows → (3,3)
    Eigen::Matrix3d mat;
    mat.row(0) = vx;
    mat.row(1) = vy;
    mat.row(2) = vz;

    // Python: self.transform = np.dot(mat.transpose(), self.transform)
    // Compute in double, store back as float
    Eigen::Matrix3d transformD = transform_.cast<double>();
    transform_ = (mat.transpose() * transformD).cast<float>();
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
    // Session 17: double accumulator for numerical stability
    const int N = (int)data.size();
    if (N == 0) return 0.0f;

    double sumSqDiff = 0.0;
    for (int i = 0; i < N; i++) {
        double radius = (double)apply(data[i]).norm();
        double diff = radius - 1.0;
        sumSqDiff += diff * diff;
    }
    return (float)sqrt(sumSqDiff / N);
}

void Sensor::setExpectedFieldStrengths(const std::vector<Eigen::Vector3f>& data) {
    // Port of Python sensor.py set_expected_field_strengths()
    // Session 17: double accumulator for numerical stability
    const int N = (int)data.size();
    if (N == 0) return;

    double sum = 0.0;
    std::vector<double> strengths(N);
    for (int i = 0; i < N; i++) {
        strengths[i] = (double)getFieldStrength(data[i]);
        sum += strengths[i];
    }
    double avg = sum / N;
    fieldAvg_ = (float)avg;

    double sumSqDiff = 0.0;
    for (int i = 0; i < N; i++) {
        double diff = strengths[i] - avg;
        sumSqDiff += diff * diff;
    }
    fieldStd_ = (float)sqrt(sumSqDiff / N);
}

} // namespace MagCal
