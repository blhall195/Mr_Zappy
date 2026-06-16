#include "leg_checker.h"

#include "defaults.h"
#include "math_utils.h"
#include <math.h>

CartesianCoordinate::CartesianCoordinate(float x, float y, float z) : x(x), y(y), z(z) {}

float CartesianCoordinate::distanceTo(const CartesianCoordinate &other) const {
    float dx = x - other.x;
    float dy = y - other.y;
    float dz = z - other.z;
    return sqrtf((dx * dx) + (dy * dy) + (dz * dz));
}

CartesianCoordinate CartesianCoordinate::fromShot(const Shot &shot) {
    float azimuth = degreesToRadians(shot.azimuth);
    float inclination = degreesToRadians(shot.inclination);
    float horiz = shot.distance * cosf(inclination);

    float x = horiz * sinf(azimuth);
    float y = horiz * cosf(azimuth);
    float z = shot.distance * sinf(inclination);

    return CartesianCoordinate{x, y, z};
}

// ── AngularLegChecker ───────────────────────────────────────────────

// Constructor only used to set the default on startup
AngularLegChecker::AngularLegChecker(float toleranceDeg) : toleranceRad_(degreesToRadians(toleranceDeg)) {}

void AngularLegChecker::setTolerance(float toleranceDeg) {
    // Keep existing setting if number passed in is invalid.
    if (toleranceDeg > 0.1 && isfinite(toleranceDeg)) {
        toleranceRad_ = degreesToRadians(toleranceDeg);
    }
}

bool AngularLegChecker::hasValidLeg(const Shot *shots, int count) const {
    if (!shots || count <= 0 || count > 8) {
        return false;
    }

    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            if (shots[i].angleTo(shots[j]) > toleranceRad_) {
                return false;
            }
        }
    }
    return true;
}

// ── CartesianLegChecker ─────────────────────────────────────────────

// Constructor only used to set the default on startup
CartesianLegChecker::CartesianLegChecker(float toleranceCm) : toleranceM_(toleranceCm / 100.0f) {}

void CartesianLegChecker::setTolerance(float toleranceCm) {
    if (!isValidTolerance(toleranceCm)) {
        return;
    }
    toleranceM_ = toleranceCm / 100.0f;
}

bool CartesianLegChecker::hasValidLeg(const Shot *shots, int count) const {
    if (!shots || count <= 0 || count > 8) {
        return false;
    }

    CartesianCoordinate endpoints[8];

    for (int i = 0; i < count; i++) {
        endpoints[i] = CartesianCoordinate::fromShot(shots[i]);
    }

    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            CartesianCoordinate a = endpoints[i];
            CartesianCoordinate b = endpoints[j];

            if (a.distanceTo(b) > toleranceM_) {
                return false;
            }
        }
    }
    return true;
}

bool CartesianLegChecker::isValidTolerance(float toleranceCm) {
    return isfinite(toleranceCm) && toleranceCm >= Defaults::cartesianToleranceMin &&
           toleranceCm <= Defaults::cartesianToleranceMax;
}
