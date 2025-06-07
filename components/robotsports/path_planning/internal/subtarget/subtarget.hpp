#ifndef SUBTARGET_HPP
#define SUBTARGET_HPP

#include <vector>
#include <cmath> // For std::sqrt, std::pow, std::cos, std::deg2rad, M_PI
#include "check_collisionfree.hpp" // Include the header for check_collisionfree
// #include "subtarget/replan/determine_setpoint_limits.hpp" // Assuming these are custom headers
// #include "subtarget/replan/quickstop_desired.hpp"
// #include "subtarget/replan/quickstop.hpp"
// #include "subtarget/replan/to_target.hpp"
// #include "subtarget/replan/new_subtarget.hpp"
// #include "subtarget/replan/new_subtarget_desired.hpp"
// #include "subtarget/angle/set.h" // Assuming this is set.h inside subtarget/angle

// Assuming MainData and SetpointData are defined in check_collisionfree.h or a common header

MainData set(MainData d);

#endif // SUBTARGET_HPP