#include "shield.hpp"

double shield(const ShieldData_t& d) {
    // get the distances to active obstacles
    // Filter active obstacles. Eigen doesn't have direct boolean indexing like numpy.
    // We'll build a new matrix of active obstacles.
    std::vector<Eigen::Vector2d> active_obstacles_vec;
    for (int i = 0; i < d.input.obstacles.active.size(); ++i) {
        if (d.input.obstacles.active[i] == 1) { // Assuming 1 means active, 0 means inactive
            active_obstacles_vec.push_back(d.input.obstacles.p.row(i));
        }
    }

    if (active_obstacles_vec.empty()) {
        // Handle case where no active obstacles are found.
        // This scenario is not explicitly handled in the Python code,
        // but it's good practice for C++. For now, we'll return a default angle
        // or throw an error depending on desired behavior.
        // For simplicity, let's return the subtarget angle if no obstacles.
        return d.subtarget.p[2];
    }

    Eigen::MatrixX2d obstacles(active_obstacles_vec.size(), 2);
    for (size_t i = 0; i < active_obstacles_vec.size(); ++i) {
        obstacles.row(i) = active_obstacles_vec[i];
    }

    // Equivalent to np.sum((obstacles - d['setpoint']['p'][:2])**2, axis=1)
    Eigen::VectorXd obsdist2 = (obstacles.rowwise() - d.setpoint.p.head<2>().transpose()).rowwise().squaredNorm();

    // Equivalent to np.min(obsdist2) and np.argmin(obsdist2)
    int nearest_obstacle_idx;
    double mindist2 = obsdist2.minCoeff(&nearest_obstacle_idx);
    Eigen::Vector2d nearest_obstacle = obstacles.row(nearest_obstacle_idx);

    // point away from nearest obstacle
    // Equivalent to -(obstacles[nearest_obstacle, :2] - d['setpoint']['p'][:2])
    Eigen::Vector2d v = -(nearest_obstacle - d.setpoint.p.head<2>());
    
    // normalize vector
    // Equivalent to v = v/ np.linalg.norm(v)
    if (v.norm() > 1e-9) { // Avoid division by zero
        v = v.normalized();
    } else {
        // If v is a zero vector, it cannot be normalized.
        // In this case, the robot is at the same position as the nearest obstacle.
        // We need to decide a sensible default or error handling.
        // For now, let's just make it point in a default direction (e.g., +x)
        v = Eigen::Vector2d(1.0, 0.0);
    }


    // determine the shielding fraction
    double x_end = 2.0;
    double x_start = 1.0;

    // Equivalent to max(0, min(1, (x_end - np.sqrt(mindist2)) / (x_end - x_start)))
    double frac = std::max(0.0, std::min(1.0, (x_end - std::sqrt(mindist2)) / (x_end - x_start)));

    // if moving, adjust the vector based on the robot velocity and obstacles
    // Equivalent to np.count_nonzero(d['setpoint']['v'][:2])
    if (d.setpoint.v.head<2>().norm() > 1e-6) {
        Eigen::Vector2d v_robot = d.setpoint.v.head<2>();
        // normalize the velocity
        if (v_robot.norm() > 1e-9) {
            v_robot = v_robot.normalized();
        } else {
            v_robot = Eigen::Vector2d(0.0, 0.0); // If velocity is zero, keep it zero
        }
        v = frac * v + (1.0 - frac) * v_robot;
    } else {
        Eigen::Vector2d robot_direction = Eigen::Vector2d(-std::sin(d.subtarget.p[2]), std::cos(d.subtarget.p[2]));
        v = frac * v + (1.0 - frac) * robot_direction;
    }

    // calculate the angle
    double angle = std::atan2(-v[0], v[1]);
    return angle;
}