#ifndef DRIBBLE_MODULE_H
#define DRIBBLE_MODULE_H

#include <eigen3/Eigen/Dense>
#include <cmath> // For std::atan2

// Structure to mimic the relevant parts of the Python 'd' dictionary
struct DribbleData {
    struct Setpoint {
        Eigen::Vector3d v; // Velocity vector (x, y, z)
        Eigen::Vector3d p; // Position vector (x, y, angle) - in Python, p[2] was the angle
    } setpoint;
};

double dribble(const DribbleData& d);

#endif // DRIBBLE_MODULE_H