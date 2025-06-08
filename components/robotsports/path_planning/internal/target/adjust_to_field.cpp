// adjust_to_field.cpp
#include "adjust_to_field.hpp"
#include <algorithm> // For std::max and std::min

TargetType_t adjust_to_field(const DType_t& d, TargetType_t target) {
    bool skill_id_active = false;
    for (int skill_id : d.input_robot.skillID) {
        if (skill_id >= 1 && skill_id <= 4) {
            skill_id_active = true;
            break;
        }
    }

    if (skill_id_active) {
        // Check ball-field violation at target
        double robot_ball_distance = d.par.robot_radius + d.par.ball_radius; // [m]
        double dball_x = -robot_ball_distance * std::sin(target.p[2]);
        double dball_y = robot_ball_distance * std::cos(target.p[2]);

        double ball_pos_at_target_x = target.p[0] + dball_x;
        double ball_pos_at_target_y = target.p[1] + dball_y;

        ball_pos_at_target_x = std::max(ball_pos_at_target_x, -d.par.field_size[0] * 0.5);
        ball_pos_at_target_y = std::max(ball_pos_at_target_y, -d.par.field_size[1] * 0.5);

        ball_pos_at_target_x = std::min(ball_pos_at_target_x, d.par.field_size[0] * 0.5);
        ball_pos_at_target_y = std::min(ball_pos_at_target_y, d.par.field_size[1] * 0.5);

        target.p[0] = ball_pos_at_target_x - dball_x;
        target.p[1] = ball_pos_at_target_y - dball_y;
    } else {
        // Clip to field size with some additional margin
        if (d.subtarget.automatic_substitution_flag == 1) {
            target.p[0] = std::max(target.p[0], -d.par.field_size[0] * 0.5 - d.par.field_border_margin - d.par.technical_area_width);
            target.p[1] = std::max(target.p[1], -d.par.field_size[1] * 0.5 - d.par.field_border_margin - d.par.technical_area_width);
            target.p[0] = std::min(target.p[0], d.par.field_size[0] * 0.5 + d.par.field_border_margin + d.par.technical_area_width);
            target.p[1] = std::min(target.p[1], d.par.field_size[1] * 0.5 + d.par.field_border_margin + d.par.technical_area_width);
        } else {
            target.p[0] = std::max(target.p[0], -d.par.field_size[0] * 0.5 - d.par.field_border_margin);
            target.p[1] = std::max(target.p[1], -d.par.field_size[1] * 0.5 - d.par.field_border_margin);
            target.p[0] = std::min(target.p[0], d.par.field_size[0] * 0.5 + d.par.field_border_margin);
            target.p[1] = std::min(target.p[1], d.par.field_size[1] * 0.5 + d.par.field_border_margin);
        }
    }

    return target;
}