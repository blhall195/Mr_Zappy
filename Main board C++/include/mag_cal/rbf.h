#pragma once
// Gaussian Radial Basis Function for non-linear sensor correction
// Session 6: port of Python mag_cal/rbf.py

#include <math.h>

namespace MagCal {

class RBF {
public:
    static constexpr int MAX_PARAMS = 7;

    RBF() : paramCount_(0), epsilon_(0.5f) {}

    /// Initialize from an array of parameters
    void init(const float* params, int count);

    /// Evaluate the RBF at point x: returns weighted sum of Gaussians
    float evaluate(float x) const;

    /// Get raw Gaussian basis values at point x (without multiplying by params)
    /// out must have room for paramCount_ elements
    void getGaussians(float x, float* out) const;

    int paramCount() const { return paramCount_; }
    bool isValid() const { return paramCount_ > 0; }
    const float* params() const { return params_; }

private:
    float params_[MAX_PARAMS];
    float offsets_[MAX_PARAMS];
    int paramCount_;
    float epsilon_;
};

} // namespace MagCal
