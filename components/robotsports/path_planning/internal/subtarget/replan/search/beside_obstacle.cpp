#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "subtarget.hpp"

// Assuming these are C++ equivalents or that their functionalities are handled within this file.
// If they are separate modules, they would need to be translated as well.
namespace subtarget {
    namespace replan {
        namespace determine_setpoint_limits {
            // Placeholder for the C++ equivalent of determine_setpoint_limits
            // Needs to be defined based on its Python implementation.
            // For now, it returns the candidate unchanged.
            // SubtargetCandidate determine_setpoint_limits(const D_Struct& d, const SubtargetCandidate& candidate) {
            //     // Implement the logic here
            //     return candidate;
            // }
        }

        namespace not_touching_obstacle {
            // Placeholder for the C++ equivalent of not_touching_obstacle
            // Needs to be defined based on its Python implementation.
            // For now, it always returns true.
            // bool not_touching_obstacle(const Eigen::Vector2d& p_candidate,
            //                            const std::vector<Eigen::Vector2d>& obstacle_positions,
            //                            double obstacle_margin) {
            //     // Implement the logic here
            //     return true;
            // }
        }

        namespace update_best {
            // Placeholder for the C++ equivalent of update_best
            // Needs to be defined based on its Python implementation.
            // For now, it returns the new_candidate if it's "better", otherwise best.
            // Best_Struct update_best(const Best_Struct& best, const SubtargetCandidate& new_candidate, const Target_Struct& target) {
            //     // Implement the logic here based on how 'best' is determined in the Python code
            //     // This is a simplified placeholder.
            //     if (new_candidate.score > best.score) { // Assuming 'score' is a metric to compare
            //         return Best_Struct{new_candidate.p, new_candidate.v, new_candidate.score};
            //     }
            //     return best;
            // }
        }
    }

    namespace check_collisionfree {
        // Placeholder for the C++ equivalent of check_collisionfree
        // Needs to be defined based on its Python implementation.
        // For now, it returns the candidate unchanged.
        // SubtargetCandidate check_collisionfree(const D_Struct& d, const SubtargetCandidate& candidate, double margin_replan) {
        //     // Implement the logic here
        //     return candidate;
        // }
    }
}



// Include Eigen library for vector and matrix operations
#include <eigen3/Eigen/Dense>

// Forward declarations for placeholder functions
namespace subtarget {
    namespace replan {
        namespace determine_setpoint_limits {
            SubtargetCandidate_t determine_setpoint_limits(const D_Struct_t& d, const SubtargetCandidate_t& candidate) {
                // Placeholder implementation
                return candidate;
            }
        }

        namespace not_touching_obstacle {
            bool not_touching_obstacle(const Eigen::Vector2d& p_candidate_xy,
                                       const std::vector<Eigen::Vector2d>& obstacle_positions_xy,
                                       double obstacle_margin) {
                // Placeholder implementation
                for (const auto& obs_pos : obstacle_positions_xy) {
                    if ((p_candidate_xy - obs_pos).norm() < obstacle_margin) {
                        return false; // Collision detected
                    }
                }
                return true; // No collision
            }
        }

        namespace update_best {
            Best_Struct update_best(const Best_Struct& best_current, const SubtargetCandidate& new_candidate, const Target& target) {
                // Placeholder: A simple example of how 'best' might be updated.
                // In a real scenario, this would likely involve a more complex scoring function.
                // For demonstration, let's say a candidate is better if it's closer to the target
                // and is considered "collision-free" (which check_collisionfree should have ensured).
                
                // This is a simplified example. The actual scoring logic from Python's update_best
                // needs to be replicated here. Assuming 'score' is already calculated in candidate.
                if (new_candidate.score > best_current.score) {
                    return Best_Struct{new_candidate.p, new_candidate.v, new_candidate.score};
                }
                return best_current;
            }
        }
    }

    namespace check_collisionfree {
        SubtargetCandidate check_collisionfree(const D_Struct& d, const SubtargetCandidate& candidate, double margin_replan) {
            // Placeholder implementation: Assume it performs checks and potentially modifies candidate
            // For example, it might adjust 'p' or 'v' to ensure collision-free movement.
            // If it determines the candidate is not collision-free, it might invalidate it
            // or return a modified version. Here, we just return it as is.
            return candidate;
        }
    }
}


Best_Struct beside_obstacle(const D_Struct& d, Best_Struct best) {
    SubtargetCandidate subtarget_candidate;
    subtarget_candidate.p = best.p;
    subtarget_candidate.v = best.v;
    subtarget_candidate.score = best.score; // Ensure score is copied as well

    double additional_obstacle_margin = 0.2; // [m] - Can be modified based on obstacle state

    for (auto i = 0u; i < d.input.obstacles.active.size(); ++i) {
        if (d.input.obstacles.active[i]) {
            Eigen::Vector3d pos = d.input.obstacles.p[i]; // Position of the obstacle
            double obstacle_radius = d.input.obstacles.r[i]; // Radius of the obstacle

            // Velocity vectors (2D for calculations)
            Eigen::Vector2d v1 = d.target.p.head<2>() - d.setpoint.p.head<2>(); // Target to setpoint vector
            Eigen::Vector2d v2 = pos.head<2>() - d.setpoint.p.head<2>();       // Obstacle to setpoint vector

            // Project v2 onto v1
            Eigen::Vector2d v3 = (v1.dot(v2) / v1.squaredNorm()) * v1; // Projection of v2 onto v1
            Eigen::Vector2d v4 = v2 - v3; // Vector orthogonal to v1

            for (double speed_factor = 0.0; speed_factor <= 0.6; speed_factor += 0.6 / 3.0) { // Equivalent to np.linspace(0, vmax_move * 0.6, 4)
                double speed = d.par.vmax_move * speed_factor;

                if (v4.norm() > 0) {
                    Eigen::Vector2d displacement_dir = v4.normalized();
                    double displacement_magnitude = d.par.robot_radius + obstacle_radius + 0.1 + d.par.margin_replan * speed;
                    Eigen::Vector2d displacement = displacement_dir * displacement_magnitude;

                    double obstacle_margin = d.par.robot_radius + obstacle_radius + additional_obstacle_margin;

                    for (int side_val : {-1, 1}) {
                        // Update the candidate position (2D for calculation, then extend to 3D)
                        Eigen::Vector2d p_candidate_xy = pos.head<2>() + (double)side_val * displacement;
                        subtarget_candidate.p.head<2>() = p_candidate_xy;
                        subtarget_candidate.p[2] = std::atan2(-v1[0], v1[1]); // Set angle

                        // Prepare obstacle positions for not_touching_obstacle
                        std::vector<Eigen::Vector2d> active_obstacle_positions_xy;
                        for (auto k = 0u; k < d.input.obstacles.active.size(); ++k) {
                            if (d.input.obstacles.active[k]) {
                                active_obstacle_positions_xy.push_back(d.input.obstacles.p[k].head<2>());
                            }
                        }

                        // Check for obstacle collision
                        if (subtarget::replan::not_touching_obstacle::not_touching_obstacle(
                            subtarget_candidate.p.head<2>(),
                            active_obstacle_positions_xy,
                            obstacle_margin
                        )) {
                            // Set candidate velocity
                            if (v1.norm() > 0) {
                                subtarget_candidate.v[0] = (v1[0] / v1.norm()) * speed;
                                subtarget_candidate.v[1] = (v1[1] / v1.norm()) * speed;
                            } else {
                                subtarget_candidate.v[0] = 0.0;
                                subtarget_candidate.v[1] = 0.0;
                            }
                            subtarget_candidate.v[2] = 0.0; // Z-component is 0 for velocity

                            // Determine setpoint limits and check for collision
                            subtarget_candidate = subtarget::replan::determine_setpoint_limits::determine_setpoint_limits(d, subtarget_candidate);
                            subtarget_candidate = subtarget::check_collisionfree::check_collisionfree(d, subtarget_candidate, d.par.margin_replan);

                            // Update best candidate
                            best = subtarget::replan::update_best::update_best(best, subtarget_candidate, d.target);
                        }
                    }
                }
            }
        }
    }
    return best;
}
