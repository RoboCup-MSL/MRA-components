#include "dribble_module.hpp"

double dribble(const DribbleData& d) {
    double angle;
    // Equivalent to np.linalg.norm(d['setpoint']['v'][:2])
    if (d.setpoint.v.head<2>().norm() > 1e-6) {
        Eigen::Vector2d v = d.setpoint.v.head<2>();
        angle = std::atan2(-v[0], v[1]);
    } else {
        angle = d.setpoint.p[2]; // Equivalent to d['setpoint']['p'][2]
    }
    return angle;
}