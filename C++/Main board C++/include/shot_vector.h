#pragma once

struct Shot {
    float azimuth = 0.0f;     // degrees, 0-360
    float inclination = 0.0f; // degrees, -90 to +90
    float distance = 0.0f;    // meters

    Shot() = default;
    Shot(float az, float inc, float dist) : azimuth(az), inclination(inc), distance(dist) {}

    // Angular separation between two shot directions, in radians. Distance is ignored.
    float angleTo(const Shot &other) const;
};
