#include "determine_setpoint_limits.hpp"

// determine_setpoint_limits.py
Subtarget_t determine_setpoint_limits(Data_t& d, Subtarget_t subtarget) {
    // Calculate the angle difference within the range [-pi, pi]
    double angle_diff = std::fmod((d.setpoint["p"][2] - subtarget.p[2] + M_PI), (2 * M_PI)) - M_PI;
    double sc = d.par["scale_rotate"][0]; // Assuming scale_rotate is a single value in a vector

    // Check conditions
    bool possess_ball = false;
    for (int skill_id : d.input_robot.skillID) { // Assuming skillID is a vector/array of ints
        if (skill_id >= 1 && skill_id <= 4) {
            possess_ball = true;
            break;
        }
    }

    bool large_angle = std::abs(angle_diff) > d.par["scale_angle"][0]; // Assuming scale_angle is a single value
    bool low_velocity = vector_norm_2d(d.setpoint["v"]) < 1;

    // Determine limits based on conditions
    std::vector<double> vmax;
    std::vector<double> amax;
    std::vector<double> dmax;

    if (possess_ball && large_angle && low_velocity) {
        vmax = {d.par["vmax_move"][0] * sc / std::sqrt(2),
                d.par["vmax_move"][0] * sc / std::sqrt(2),
                d.par["vmax_rotate"][0]};

        amax = {d.par["amax_move"][0] * sc / std::sqrt(2),
                d.par["amax_move"][0] * sc / std::sqrt(2),
                d.par["amax_rotate"][0]};

        dmax = {d.par["dmax_move"][0] * sc / std::sqrt(2),
                d.par["dmax_move"][0] * sc / std::sqrt(2),
                d.par["dmax_rotate"][0]};
    } else {
        vmax = {d.par["vmax_move"][0] / std::sqrt(2),
                d.par["vmax_move"][0] / std::sqrt(2),
                d.par["vmax_rotate"][0]};

        amax = {d.par["amax_move"][0] / std::sqrt(2),
                d.par["amax_move"][0] / std::sqrt(2),
                d.par["amax_rotate"][0]};

        dmax = {d.par["dmax_move"][0] / std::sqrt(2),
                d.par["dmax_move"][0] / std::sqrt(2),
                d.par["dmax_rotate"][0]};
    }

    // Clip desired subtarget velocity to maximum velocity if condition is met
    // Assuming subtarget.v and vmax are at least 2 elements for 0:2 slicing
    if (vector_norm_2d(subtarget.v) > vector_norm_2d(vmax) && d.input_robot.skillID != 5) {
        double norm_subtarget_v = vector_norm_2d(subtarget.v);
        if (norm_subtarget_v > 1e-6) { // Avoid division by zero
            subtarget.v[0] = (subtarget.v[0] / norm_subtarget_v) * vector_norm_2d(vmax);
            subtarget.v[1] = (subtarget.v[1] / norm_subtarget_v) * vector_norm_2d(vmax);
        } else {
            subtarget.v[0] = 0;
            subtarget.v[1] = 0;
        }
    }

    // Update subtarget limits using a balancing function (spg.setpoint.balance_xy in MATLAB)
    // This assumes balance_xy returns a tuple that matches the assignment.
    std::tie(subtarget.segment, subtarget.vmax, subtarget.amax) = balance_xy(
        subtarget.segment,
        d.setpoint["p"],
        d.setpoint["v"],
        subtarget.p,
        subtarget.v,
        vmax,
        amax,
        dmax
    );

    // Compute eta, the maximum value in the time component of subtarget's segment
    // Assuming segment[2]['t'] is a vector with at least 2 elements
    subtarget.eta = std::fmax(subtarget.segment[2]['t'][0], subtarget.segment[2]['t'][1]);

    return subtarget;
}
