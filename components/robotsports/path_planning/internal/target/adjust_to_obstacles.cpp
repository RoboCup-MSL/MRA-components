// adjust_to_obstacles.cpp
#include "adjust_to_obstacles.hpp"
#include <limits> // For std::numeric_limits

TargetType_t adjust_to_obstacles(const DType_t& d, TargetType_t target) {
    // Prep
    double extra_margin = 0.05; // [m] move a bit further beyond the collision_distance
    int nAttempts = 10; // [#] maximum iteration count to step away from nearby obstacles

    std::vector<double> obstacles_p_active;
    std::vector<double> r_obstacles_active;
    std::vector<double> collision_distance_active;

    for (size_t i = 0; i < d.input_obstacles.active.size(); ++i) {
        if (d.input_obstacles.active[i]) {
            obstacles_p_active.push_back(d.input_obstacles.p[i * 2]);     // x-coordinate
            obstacles_p_active.push_back(d.input_obstacles.p[i * 2 + 1]); // y-coordinate
            r_obstacles_active.push_back(d.input_obstacles.r[i]);
            collision_distance_active.push_back(d.par.robot_radius + d.input_obstacles.r[i]);
        }
    }

    // Shift target
    if (!obstacles_p_active.empty()) {
        for (int iAttempt = 0; iAttempt < nAttempts; ++iAttempt) {
            // Get distance of target to obstacles
            double norm2min = std::numeric_limits<double>::max();
            int ind = -1;

            for (size_t i = 0; i < obstacles_p_active.size() / 2; ++i) {
                double obs_x = obstacles_p_active[i * 2];
                double obs_y = obstacles_p_active[i * 2 + 1];

                double v_target_obstacles_x = target.p[0] - obs_x;
                double v_target_obstacles_y = target.p[1] - obs_y;

                double v_target_obstacles_norm2 = v_target_obstacles_x * v_target_obstacles_x + v_target_obstacles_y * v_target_obstacles_y;
                double current_norm2_diff = v_target_obstacles_norm2 - collision_distance_active[i] * collision_distance_active[i];

                if (current_norm2_diff < norm2min) {
                    norm2min = current_norm2_diff;
                    ind = i;
                }
            }

            // Project target away from nearest obstacle
            if (norm2min < 0) {
                double obs_x = obstacles_p_active[ind * 2];
                double obs_y = obstacles_p_active[ind * 2 + 1];

                double vec_x = target.p[0] - obs_x;
                double vec_y = target.p[1] - obs_y;
                double vec_norm = std::sqrt(vec_x * vec_x + vec_y * vec_y);

                if (vec_norm > 1e-9) { // Avoid division by zero
                    target.p[0] = (obs_x + (vec_x / vec_norm) * (collision_distance_active[ind] + extra_margin));
                    target.p[1] = (obs_y + (vec_y / vec_norm) * (collision_distance_active[ind] + extra_margin));
                } else {
                    // If target is exactly on obstacle, move it in a default direction (e.g., positive x)
                    target.p[0] = obs_x + (collision_distance_active[ind] + extra_margin);
                }
            } else {
                break;
            }
        }
    }

    return target;
}