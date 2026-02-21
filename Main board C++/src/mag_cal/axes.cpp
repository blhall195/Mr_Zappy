#include "mag_cal/axes.h"
#include <string.h>
#include <ctype.h>

namespace MagCal {

void Axes::parseFragment(const char* frag, int8_t& axis, int8_t& polarity) {
    polarity = (frag[0] == '-') ? -1 : 1;
    char ch = toupper(frag[1]);
    axis = (ch == 'X') ? 0 : (ch == 'Y') ? 1 : 2;
}

Axes::Axes(const char* axesStr) {
    for (int i = 0; i < 3; i++) {
        parseFragment(axesStr + i * 2, indices_[i], polarities_[i]);
    }
    // Store string representation
    const char* letters = "XYZ";
    for (int i = 0; i < 3; i++) {
        str_[i * 2]     = (polarities_[i] > 0) ? '+' : '-';
        str_[i * 2 + 1] = letters[indices_[i]];
    }
    str_[6] = '\0';
}

Eigen::Vector3f Axes::fixAxes(const Eigen::Vector3f& data) const {
    return Eigen::Vector3f(
        data[indices_[0]] * polarities_[0],
        data[indices_[1]] * polarities_[1],
        data[indices_[2]] * polarities_[2]
    );
}

} // namespace MagCal
