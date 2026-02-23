#pragma once
// Axis permutation and sign-flip for sensor coordinate systems
// Session 6: port of Python mag_cal/axes.py

#include <ArduinoEigenDense.h>

namespace MagCal {

class Axes {
public:
    /// Parse axis string like "+X+Y+Z" or "-X-Y-Z". Must be exactly 6 chars.
    explicit Axes(const char* axesStr = "+X+Y+Z");

    /// Transform a single reading from sensor coords to device coords
    Eigen::Vector3f fixAxes(const Eigen::Vector3f& data) const;

    /// Double-precision overload for fitting path
    Eigen::Vector3d fixAxes(const Eigen::Vector3d& data) const;

    /// Return axis string representation (e.g. "-X-Y-Z")
    const char* toString() const { return str_; }

private:
    int8_t indices_[3];    // which sensor axis maps to each device axis (0=X,1=Y,2=Z)
    int8_t polarities_[3]; // +1 or -1 for each device axis
    char str_[7];          // "+X+Y+Z\0"

    static void parseFragment(const char* frag, int8_t& axis, int8_t& polarity);
};

} // namespace MagCal
