// adjust_to_3m_rule.cpp
#include "adjust_to_3m_rule.hpp"
#include <algorithm> // For std::any_of, std::max, std::min

TargetType_t adjust_to_3m_rule(const DType_t& d, TargetType_t target) {
    double max_radius = 3.0; // Maximum allowable radius (in meters) from the reference point

    // Calculate the differences in x and y coordinates between the reference point
    // (cpb_poi_xy) and the target position
    double dx = target.p[0] - d.input_robot.cpb_poi_xy[0];
    double dy = target.p[1] - d.input_robot.cpb_poi_xy[1];

    // Calculate the straight-line distance from the reference point to the target
    double distance_between_cpb_poi_xy_and_target = std::sqrt(dx * dx + dy * dy);

    // Calculate the angle (in radians) from the reference point to the target
    double angle_between_cpb_poi_xy_and_target = std::atan2(dy, dx);

    // Check if specific conditions are met:
    // - Any skill ID from 1 to 4 is active
    // - The distance from the reference point to the target exceeds the max radius
    // - The human dribble flag is inactive (set to 0)
    bool skill_id_active = false;
    for (int skill_id : d.input_robot.skillID) {
        if (skill_id >= 1 && skill_id <= 4) {
            skill_id_active = true;
            break;
        }
    }

    if (skill_id_active &&
        distance_between_cpb_poi_xy_and_target > max_radius &&
        d.input_robot.human_dribble_flag == 0) {

        // If conditions are met, clip the target position to the 3-meter radius
        // around the reference point
        double clipped_target_x = max_radius * std::cos(angle_between_cpb_poi_xy_and_target);
        double clipped_target_y = max_radius * std::sin(angle_between_cpb_poi_xy_and_target);

        // Update the target's x and y coordinates to the clipped position
        target.p[0] = clipped_target_x;
        target.p[1] = clipped_target_y;
    }

    return target;
}