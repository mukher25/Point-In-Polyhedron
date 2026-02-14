#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <limits>

namespace constants
{
    constexpr double EPSILON = 1e-9;
    constexpr double SMALL = 1e-12;
    constexpr double BIG = 1e12;

    // for stl check
    constexpr double WELD_TOL = 1e-9;

    // smart stop on split
    constexpr double FLAT_THRESHOLD = 0.98;
}

#endif
