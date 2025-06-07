#include "aim_at_target.hpp"

double aim_at_target(const AimData& d) {
    double angle;
    // Equivalent to d['target']['p'][:2] - d['setpoint']['p'][:2]
    Eigen::Vector2d v = d.target.p.head<2>() - d.setpoint.p.head<2>();

    // Equivalent to np.count_nonzero(v)
    if (v.any()) { // Check if any element in v is non-zero
        angle = std::atan2(-v[0], v[1]);
    } else {
        angle = d.subtarget.p[2]; // Equivalent to d['subtarget']['p'][2]
    }
    return angle;
}