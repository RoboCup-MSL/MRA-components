#ifndef SET_H
#define SET_H

#include "setpoint.hpp"
#include "get_segments.hpp" // For get_segments
#include "traj1.hpp"        // For traj1


Data set(Data d) {
    bool within_bounds_x = (-d.par.field_size[0] * 0.5 - d.par.field_border_margin < d.input.robot.p[0] &&
                            d.input.robot.p[0] < d.par.field_size[0] * 0.5 + d.par.field_border_margin);
    bool within_bounds_y = (-d.par.field_size[1] * 0.5 - d.par.field_border_margin < d.input.robot.p[1] &&
                            d.input.robot.p[1] < d.par.field_size[1] * 0.5 + d.par.field_border_margin);

    if ((within_bounds_x && within_bounds_y) || d.subtarget.automatic_substitution_flag == true) {

        double tipping_degrees = 5.0;
        double robot_tipping_degrees_sq = (d.input.robot.IMU_orientation[0] * d.input.robot.IMU_orientation[0] +
                                           d.input.robot.IMU_orientation[1] * d.input.robot.IMU_orientation[1]);

        std::array<double, DOF> current_dmax_move;
        if (std::sqrt(robot_tipping_degrees_sq) > tipping_degrees) {
            current_dmax_move[0] = 0.5 * d.par.dmax_move;
            current_dmax_move[1] = 0.5 * d.par.dmax_move;
            current_dmax_move[2] = 0.5 * d.par.dmax_rotate;
        } else {
            current_dmax_move[0] = d.par.dmax_move;
            current_dmax_move[1] = d.par.dmax_move;
            current_dmax_move[2] = d.par.dmax_rotate;
        }
        
        d.aux.segment = get_segments(d.aux.segment, d.setpoint.p, d.setpoint.v, d.subtarget.p, d.subtarget.v, d.subtarget.vmax, d.subtarget.amax, current_dmax_move);

        d.traj = traj1(d, d.aux.segment, d.par.Ts).traj;

        double field_width_half = d.par.field_size[0] * 0.5 + d.par.field_border_margin + (d.subtarget.automatic_substitution_flag ? d.par.technical_area_width : 0.0);
        double field_length_half = d.par.field_size[1] * 0.5 + d.par.field_border_margin;

        d.setpoint.p[0] = std::clamp(d.setpoint.p[0], -field_width_half, field_width_half);
        double dist2sideline = field_width_half - std::abs(d.traj.p[0][0]);
        double vx_max = 2 * d.par.dmax_move * dist2sideline;
        d.setpoint.v[0] = std::clamp(d.setpoint.v[0], -vx_max, vx_max);

        d.setpoint.p[1] = std::clamp(d.setpoint.p[1], -field_length_half, field_length_half);
        double dist2goalline = field_length_half - std::abs(d.traj.p[0][1]);
        double vy_max = 2 * d.par.dmax_move * dist2goalline;
        d.setpoint.v[1] = std::clamp(d.setpoint.v[1], -vy_max, vy_max);

        if (d.setpoint.v[0] > vx_max && d.setpoint.p[0] > 0) {
            d.setpoint.a[0] = -d.par.dmax_move;
        } else if (d.setpoint.v[0] < -vx_max && d.setpoint.p[0] < 0) {
            d.setpoint.a[0] = d.par.dmax_move;
        }

        if (d.setpoint.v[1] > vy_max && d.setpoint.p[1] > 0) {
            d.setpoint.a[1] = -d.par.dmax_move;
        } else if (d.setpoint.v[1] < -vy_max && d.setpoint.p[1] < 0) {
            d.setpoint.a[1] = d.par.dmax_move;
        }

    } else {
        d.setpoint.p = d.input.robot.p;
        d.setpoint.v = {0.0, 0.0, 0.0};
        d.setpoint.a = {0.0, 0.0, 0.0};
    }

    return d;
}

#endif // SET_H