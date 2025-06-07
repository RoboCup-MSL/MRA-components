#include "check_collisionfree.hpp"
#include "get_segments.hpp" // Assuming this is a custom header for get_segments
#include "traj_pred.hpp"     // Assuming this is a custom header for traj_pred
#include <iostream>        // For debugging, can be removed

// Helper function definition
Violation update_violation(Violation violation, const std::string& field, double violation_value) {
    if (violation_value > 0) {
        violation.count += 1;
    }
    // Using a map to access violation fields by string name
    // A more robust way would be to use enums or direct member access
    if (field == "obstacle") {
        if (violation_value > violation.obstacle) {
            violation.collisionfree = false;
        } else {
            violation.obstacle = violation_value;
        }
    } else if (field == "field") {
        if (violation_value > violation.field) {
            violation.collisionfree = false;
        } else {
            violation.field = violation_value;
        }
    }
    // Add other fields as needed
    return violation;
}

std::tuple<SetpointData, std::vector<std::vector<double>>, std::vector<std::vector<std::vector<double>>>>
check_collisionfree(MainData& d, SetpointData subtarget, double obstacle_margin) {

    // Determine robot trajectory prediction
    // Assuming get_segments and traj_predict are C++ functions that modify 'subtarget' and 'd' respectively
    subtarget.segment = GetSegments::get_segments(
        subtarget.segment,
        d.setpoint.p, d.setpoint.v,
        subtarget.p, subtarget.v,
        subtarget.vmax, subtarget.amax,
        d.par.dmax_move // Using d['par']['dmax_move'] instead of hardcoded list from original
    );
    // Assuming segment[2]['t'] is accessible and has at least two elements
    // This part of the Python code seems to assume a specific structure for 'segment'
    // For now, let's assume segment is a vector of maps or a custom struct
    // and access its elements accordingly.
    // If segment is a vector of maps, we might need a more complex way to get 't'
    // For simplicity, let's assume subtarget.segment is structured such that
    // subtarget.segment[2] is a map containing 't' as a key
    // For this example, let's simplify and assume `segment` has a direct access to `t`
    // (e.g., `subtarget.segment_times`) or adapt `get_segments` return.
    // Given the Python code's `max(subtarget['segment'][2]['t'][:2])`, it looks like
    // `subtarget['segment'][2]` is a dictionary/map and it contains a list/vector 't'.
    // We'll need to ensure our C++ `segment` structure supports this.
    // For now, let's make an assumption based on the usage:
    // If subtarget.segment is a vector of structs, and each struct has a 't' member which is a vector:
    if (subtarget.segment.size() > 2 && subtarget.segment[2].count("t")) {
        subtarget.eta = std::max(subtarget.segment[2]["t"][0][0], subtarget.segment[2]["t"][0][1]);
    } else {
        subtarget.eta = 0.0; // Default or error handling
    }

    d = TrajPred::traj_predict(d, subtarget.segment);

    // This assignment seems to be using `d['traj']['segment_id'][0, :3]` which implies
    // `segment_id` is a 2D array/vector and we're taking the first row, first three elements.
    // Assuming `d.traj.segment_id` is a `std::vector<int>` and has enough elements.
    if (d.traj.segment_id.size() >= 3) {
        subtarget.segment_id = d.traj.segment_id[0]; // Assuming first element of segment_id is the relevant one
    }

    std::vector<std::vector<double>> p_robot = d.traj.p;
    std::vector<std::vector<double>> v_robot = d.traj.v;

    // Preparations for trajectory check
    std::vector<std::vector<double>> p_obstacles_active;
    std::vector<std::vector<double>> v_obstacles_active;
    std::vector<double> r_obstacles_active;

    for (size_t i = 0; i < d.input.obstacles.active.size(); ++i) {
        if (d.input.obstacles.active[i]) {
            p_obstacles_active.push_back(d.input.obstacles.p[i]);
            v_obstacles_active.push_back(d.input.obstacles.v[i]);
            r_obstacles_active.push_back(d.input.obstacles.r[i]);
        }
    }

    std::vector<double> collision_distance(r_obstacles_active.size());
    for (size_t i = 0; i < r_obstacles_active.size(); ++i) {
        collision_distance[i] = d.par.robot_radius + r_obstacles_active[i];
    }

    Violation violation = {
        true,   // collisionfree
        0,      // count
        1e10,   // SubtargetAvoidPolygon
        1e10,   // obstacle
        1e10    // field
    };

    int npropagate = p_robot.size(); // Number of trajectory samples
    // Initialize p_obstacles_traj with NaNs if possible, or zeros if not critical
    // For C++, we can use std::vector and resize/fill.
    std::vector<std::vector<std::vector<double>>> p_obstacles_traj(
        p_obstacles_active.size(),
        std::vector<std::vector<double>>(p_obstacles_active[0].size(),
                                         std::vector<double>(npropagate, NAN))
    );

    // Propagate to assess trajectory feasibility
    for (int i = 0; i < npropagate; ++i) {
        // Obstacle path (constant/declining velocity model)
        for (size_t j = 0; j < v_obstacles_active.size(); ++j) {
            for (size_t k = 0; k < v_obstacles_active[j].size(); ++k) {
                v_obstacles_active[j][k] *= d.par.obstacle_vel_gain;
                p_obstacles_active[j][k] += v_obstacles_active[j][k] * d.par.Ts_predict;
            }
        }
        for (size_t j = 0; j < p_obstacles_active.size(); ++j) {
            p_obstacles_traj[j][0][i] = p_obstacles_active[j][0];
            p_obstacles_traj[j][1][i] = p_obstacles_active[j][1];
            // Assuming p_obstacles_active has at least 2 columns
        }


        // Check collision
        if (p_obstacles_active.empty()) {
            violation = update_violation(violation, "obstacle", 0);
        } else {
            std::vector<double> p_diff_norm(p_obstacles_active.size());
            std::vector<std::vector<double>> p_diff(p_obstacles_active.size(), std::vector<double>(2));

            for (size_t j = 0; j < p_obstacles_active.size(); ++j) {
                p_diff[j][0] = p_obstacles_active[j][0] - p_robot[i][0];
                p_diff[j][1] = p_obstacles_active[j][1] - p_robot[i][1];
                p_diff_norm[j] = std::sqrt(p_diff[j][0] * p_diff[j][0] + p_diff[j][1] * p_diff[j][1]);
            }

            // Determine collision violation value
            std::vector<double> p_diff_obst2Target_abs_min(2);
            if (!p_obstacles_active.empty()) {
                p_diff_obst2Target_abs_min[0] = std::abs(p_obstacles_active[0][0] - d.target.p[0]);
                p_diff_obst2Target_abs_min[1] = std::abs(p_obstacles_active[0][1] - d.target.p[1]);
                for (size_t j = 1; j < p_obstacles_active.size(); ++j) {
                    p_diff_obst2Target_abs_min[0] = std::min(p_diff_obst2Target_abs_min[0], std::abs(p_obstacles_active[j][0] - d.target.p[0]));
                    p_diff_obst2Target_abs_min[1] = std::min(p_diff_obst2Target_abs_min[1], std::abs(p_obstacles_active[j][1] - d.target.p[1]));
                }
            } else {
                 p_diff_obst2Target_abs_min[0] = 0.0;
                 p_diff_obst2Target_abs_min[1] = 0.0;
            }


            double target2ball_distance = std::sqrt(
                std::pow(d.target.p[0] - d.input.ball.p[0], 2) +
                std::pow(d.target.p[1] - d.input.ball.p[1], 2)
            );

            std::vector<double> p_diff_robot2Target = {
                p_robot[i][0] - d.target.p[0],
                p_robot[i][1] - d.target.p[1]
            };
            double p_diff_robot2Target_norm = std::sqrt(
                std::pow(p_diff_robot2Target[0], 2) +
                std::pow(p_diff_robot2Target[1], 2)
            );

            // Angle check between robot and ball
            std::vector<std::vector<double>> ball2obstacle(p_obstacles_active.size(), std::vector<double>(2));
            for (size_t j = 0; j < p_obstacles_active.size(); ++j) {
                ball2obstacle[j][0] = p_obstacles_active[j][0] - d.input.ball.p[0];
                ball2obstacle[j][1] = p_obstacles_active[j][1] - d.input.ball.p[1];
            }

            std::vector<double> robot2ball = {
                p_robot[i][0] - d.input.ball.p[0],
                p_robot[i][1] - d.input.ball.p[1]
            };

            std::vector<double> angles;
            bool no_obstacle_between_me_and_ball = true;
            for (size_t j = 0; j < ball2obstacle.size(); ++j) {
                double cross_prod = ball2obstacle[j][0] * robot2ball[1] - ball2obstacle[j][1] * robot2ball[0];
                double dot_prod = ball2obstacle[j][0] * robot2ball[0] + ball2obstacle[j][1] * robot2ball[1];
                double angle_rad = std::atan2(std::abs(cross_prod), dot_prod);
                angles.push_back(angle_rad * 180.0 / M_PI); // Convert to degrees
                if (angles.back() < 45.0) { // Check if angle is less than 45 degrees
                    no_obstacle_between_me_and_ball = false;
                    break;
                }
            }


            // Update violation based on obstacle distance and angle checks
            double violation_value;
            if (((d.input.robot.skillID == 5 || d.input.robot.skillID == 0)) &&
                (std::sqrt(std::pow(p_diff_obst2Target_abs_min[0], 2) + std::pow(p_diff_obst2Target_abs_min[1], 2)) + d.input.robot.dist2ball_vs_opp >
                 p_diff_robot2Target_norm) &&
                (target2ball_distance < 0.5) &&
                no_obstacle_between_me_and_ball) {
                violation_value = 0;
            } else {
                double max_obs_violation = 0;
                for (size_t j = 0; j < p_obstacles_active.size(); ++j) {
                    double obs_violation = std::max(0.0, (collision_distance[j] + obstacle_margin * std::sqrt(std::pow(v_robot[i][0], 2) + std::pow(v_robot[i][1], 2))) - p_diff_norm[j]);
                    if (obs_violation > max_obs_violation) {
                        max_obs_violation = obs_violation;
                    }
                }
                violation_value = max_obs_violation;
            }

            // Update the violation status
            violation = update_violation(violation, "obstacle", violation_value);
        }

        // Other checks (e.g., illegal driving zones, field boundaries) would follow here...

        // Stop if close to the subtarget
        if (std::sqrt(std::pow(p_robot[i][0] - subtarget.p[0], 2) + std::pow(p_robot[i][1] - subtarget.p[1], 2)) < 1e-2) {
            break;
        }
    }

    // Update the subtarget's collision-free status
    subtarget.collisionfree = violation.collisionfree;
    subtarget.violation_count = violation.count;

    return std::make_tuple(subtarget, p_robot, p_obstacles_traj);
}