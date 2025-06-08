#ifndef ANGLE_HPP
#define ANGLE_HPP

#include "dribble_module.hpp" // For dribble function and DribbleData struct
#include "aim_at_target.hpp"  // Although not directly used for skillID, it's a related module
#include "shield.hpp"         // Although not directly used for skillID, it's a related module
#include <cmath>            // For fmod and other math operations

// Define a wrap function similar to setpoint.wrap
// This function needs to be defined or included from a C++ equivalent of setpoint.wrap
// For now, let's define a simple wrap function assuming it wraps angles to -PI to PI
double wrap_angle(double angle, double current_setpoint_angle);


double set_angle(const GlobalData& d);

#endif // ANGLE_HPP
