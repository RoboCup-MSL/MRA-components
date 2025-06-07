#ifndef SHIELD_H
#define SHIELD_H

#include <eigen3/Eigen/Dense>
#include <cmath>     // For std::sqrt, std::atan2, std::max, std::min
#include <limits>    // For std::numeric_limits

// Structure to mimic the relevant parts of the Python 'd' dictionary
struct ShieldData {
    struct Input {
        struct Obstacles {
            Eigen::MatrixX2d p; // N x 2 matrix for obstacle positions (x, y)
            Eigen::VectorXi active; // Boolean array as an integer vector (0 or 1)
        } obstacles;
        struct Robot {
            int skillID; // Assuming skillID is an integer
        } robot;
    } input;
    struct Setpoint {
        Eigen::Vector3d p; // Position vector (x, y, angle)
        Eigen::Vector3d v; // Velocity vector (vx, vy, vz)
    } setpoint;
    struct Subtarget {
        Eigen::Vector3d p; // Position vector (x, y, angle)
    } subtarget;
};

double shield(const ShieldData& d);

#endif // SHIELD_H