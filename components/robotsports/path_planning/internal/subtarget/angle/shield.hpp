#ifndef SHIELD_H
#define SHIELD_H

#include <eigen3/Eigen/Dense>
#include <cmath>     // For std::sqrt, std::atan2, std::max, std::min
#include <limits>    // For std::numeric_limits
#include "subtarget.hpp"

double shield(const ShieldData_t& d);

#endif // SHIELD_H
