#include "shot_vector.h"

#include "math_utils.h"
#include <math.h>

float Shot::angleTo(const Shot &other) const {
    float az1 = azimuth * DEG2RAD, inc1 = inclination * DEG2RAD;
    float az2 = other.azimuth * DEG2RAD, inc2 = other.inclination * DEG2RAD;

    float dot = cosf(inc1) * sinf(az1) * cosf(inc2) * sinf(az2) +
                cosf(inc1) * cosf(az1) * cosf(inc2) * cosf(az2) + sinf(inc1) * sinf(inc2);

    if (dot > 1.0f) {
        dot = 1.0f;
    }
    if (dot < -1.0f) {
        dot = -1.0f;
    }
    return acosf(dot);
}
