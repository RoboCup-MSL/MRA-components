#include "subtarget.hpp"
#include <numeric> // For std::inner_product

MainData_t set(MainData_t d) {
    // Calculate the distance between the robot and the ball
    double robot2ball_dist = std::sqrt(
        std::pow(d.input.ball.p[0] - d.input.robot.p[0], 2) +
        std::pow(d.input.ball.p[1] - d.input.robot.p[1], 2)
    );

    // Check if the robot is moving in the same direction as the ball (within a 10-degree angle)
    bool robotVel_same_direction_as_ballVel = false;
    double robot_vel_norm = std::sqrt(std::pow(d.input.robot.v[0], 2) + std::pow(d.input.robot.v[1], 2));
    double ball_vel_norm = std::sqrt(std::pow(d.input.ball.v[0], 2) + std::pow(d.input.ball.v[1], 2));

    if (robot_vel_norm > 0 && ball_vel_norm > 0) {
        std::vector<double> robot_vel_unit = {d.input.robot.v[0] / robot_vel_norm, d.input.robot.v[1] / robot_vel_norm};
        std::vector<double> ball_vel_unit = {d.input.ball.v[0] / ball_vel_norm, d.input.ball.v[1] / ball_vel_norm};
        double dot_product = robot_vel_unit[0] * ball_vel_unit[0] + robot_vel_unit[1] * ball_vel_unit[1];
        robotVel_same_direction_as_ballVel = dot_product > std::cos(10.0 * M_PI / 180.0);
    }

    // Calculate if the robot is behind the ball within a +-45 degree angle
    bool behind_ball_flag = false;
    std::vector<double> robot2ball = {
        d.input.ball.p[0] - d.input.robot.p[0],
        d.input.ball.p[1] - d.input.robot.p[1]
    };
    double robot2ball_norm = std::sqrt(std::pow(robot2ball[0], 2) + std::pow(robot2ball[1], 2));

    if (robot2ball_norm > 0 && ball_vel_norm > 0) {
        std::vector<double> robot2ball_unit = {robot2ball[0] / robot2ball_norm, robot2ball[1] / robot2ball_norm};
        std::vector<double> ball_vel_unit = {d.input.ball.v[0] / ball_vel_norm, d.input.ball.v[1] / ball_vel_norm};
        double dot_product = robot2ball_unit[0] * ball_vel_unit[0] + robot2ball_unit[1] * ball_vel_unit[1];
        behind_ball_flag = dot_product > std::cos(M_PI / 4.0);
    }

    // Human dribbling logic: if dribbling, within 3.5 meters, behind ball, and moving in the same direction
    if (d.input.robot.human_dribble_flag == 1 && robot2ball_dist < 3.5 &&
        behind_ball_flag && robotVel_same_direction_as_ballVel) {
        // Move directly to target without checking for collisions
        d.subtarget.p[0] = d.target.p[0];
        d.subtarget.p[1] = d.target.p[1];
        // d.subtarget.p[2] retains its original value as per Python code
        d.subtarget.v[0] = d.target.v[0];
        d.subtarget.v[1] = d.target.v[1];
        d.subtarget.v[2] = d.target.v[2];

        d.subtarget = DetermineSetpointLimits::determine_setpoint_limits(d, d.subtarget);
        d.subtarget.target_p = d.target.p; // Assuming target.p is the full target position
        d.subtarget.age = 0;
        d.subtarget.action = 1; // Move to target
        return d;
    }

    // Update the current subtarget's orientation and collision-free status
    d.subtarget.p[2] = SetAngle::set(d); // Assuming set function in angle.set is accessible
    std::tuple<SetpointData, std::vector<std::vector<double>>, std::vector<std::vector<std::vector<double>>>> collision_result =
        check_collisionfree(d, d.subtarget, 0);
    d.subtarget = std::get<0>(collision_result); // Update d.subtarget with the returned SetpointData
    d.subtarget.age += 1;

    // Perform a quickstop if required
    if (QuickstopDesired::quickstop_desired(d)) {
        d.subtarget = Quickstop::quickstop(d, d.subtarget);
        d.subtarget.action = 0; // Quickstop
        return d;
    }

    // Try to move directly to the target if possible
    SetpointData subtarget_target = ToTarget::to_target(d);
    if (subtarget_target.collisionfree) {
        subtarget_target.action = 1; // Move to target
        d.subtarget = subtarget_target;
        return d;
    }

    // Try to replan a new subtarget if desired
    if (NewSubtargetDesired::new_subtarget_desired(d)) {
        SetpointData new_subt = NewSubtarget::new_subtarget(d, d.subtarget);
        if (new_subt.collisionfree) {
            new_subt.action = 2; // Replan subtarget
            d.subtarget = new_subt;
            return d;
        }
    }

    // Keep the current subtarget if it’s collision-free
    if (d.subtarget.collisionfree) {
        d.subtarget.action = 3; // Keep subtarget
        return d;
    }

    // If all else fails, perform a quickstop
    d.subtarget = Quickstop::quickstop(d, d.subtarget);
    d.subtarget.action = 0; // Quickstop

    return d;
}