#ifndef AIM_AT_TARGET_H
#define AIM_AT_TARGET_H

#include <eigen3/Eigen/Dense>
#include <cmath> // For std::atan2

// Structure to mimic the relevant parts of the Python 'd' dictionary
struct AimData {
    struct Target {
        Eigen::Vector3d p; // Position vector (x, y, angle)
    } target;
    struct Setpoint {
        Eigen::Vector3d p; // Position vector (x, y, angle)
    } setpoint;
    struct Subtarget {
        Eigen::Vector3d p; // Position vector (x, y, angle)
    } subtarget;
};

double aim_at_target(const AimData& d);

#endif // AIM_AT_TARGET_H