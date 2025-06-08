#ifndef CHECK_COLLISIONFREE_H
#define CHECK_COLLISIONFREE_H

#include <vector>
#include <map>
#include <string>
#include <cmath> // For std::sqrt, std::abs, std::max, std::sin, std::cos, std::atan2, std::degrees, M_PI
#include "subtarget.hpp"


// Helper function declaration
Violation_t update_violation(Violation_t violation, const std::string& field, double violation_value);

// Main function declaration
std::tuple<SetpointData_t, std::vector<std::vector<double>>, std::vector<std::vector<std::vector<double>>>>
check_collisionfree(MainData_t& d, SetpointData subtarget, double obstacle_margin);

#endif // CHECK_COLLISIONFREE_H
