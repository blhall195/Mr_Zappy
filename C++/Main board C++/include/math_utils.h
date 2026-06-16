#pragma once

#include <math.h>

static constexpr float DEG2RAD = (float)M_PI / 180.0f;
static constexpr float RAD2DEG = 180.0f / (float)M_PI;

inline float degreesToRadians(float deg) { return deg * DEG2RAD; }
inline float radiansToDegrees(float rad) { return rad * RAD2DEG; }

// Wraps a value to [-180, 180).
inline float wrapTo180(float d) {
    float r = fmodf(d + 180.0f, 360.0f);
    if (r < 0.0f) {
        r += 360.0f;
    }
    return r - 180.0f;
}

// Wraps an angle to [0, 360).
inline float wrapTo360(float a) { return fmodf(fmodf(a, 360.0f) + 360.0f, 360.0f); }

// Shortest angular distance between two bearings (unsigned), in [0, 180].
inline float circularDiff(float a, float b) { return fabsf(wrapTo180(a - b)); }
