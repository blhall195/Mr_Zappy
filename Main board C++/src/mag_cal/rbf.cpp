#include "mag_cal/rbf.h"

namespace MagCal {

void RBF::init(const float* params, int count) {
    paramCount_ = (count > MAX_PARAMS) ? MAX_PARAMS : count;
    for (int i = 0; i < paramCount_; i++) {
        params_[i] = params[i];
    }

    if (paramCount_ == 1) {
        offsets_[0] = 0.0f;
        epsilon_ = 0.5f;
    } else {
        // linspace from -1 to 1
        for (int i = 0; i < paramCount_; i++) {
            offsets_[i] = -1.0f + 2.0f * i / (paramCount_ - 1);
        }
        epsilon_ = 1.5f / paramCount_;
    }
}

float RBF::evaluate(float x) const {
    float result = 0.0f;
    for (int i = 0; i < paramCount_; i++) {
        float dist = (x - offsets_[i]) / epsilon_;
        result += params_[i] * expf(-dist * dist);
    }
    return result;
}

void RBF::getGaussians(float x, float* out) const {
    for (int i = 0; i < paramCount_; i++) {
        float dist = (x - offsets_[i]) / epsilon_;
        out[i] = expf(-dist * dist);
    }
}

void RBF::getGaussians(double x, double* out) const {
    for (int i = 0; i < paramCount_; i++) {
        double dist = (x - (double)offsets_[i]) / (double)epsilon_;
        out[i] = exp(-dist * dist);
    }
}

} // namespace MagCal
