#include "SetpointCalculation.hpp"
#include "mra_tracing/tracing.hpp"

SetpointCalculation::SetpointCalculation()
{
    TRACE_FUNCTION();
    // Initialize VelocityControl component
    velocity_control_ = std::make_unique<VelocityControl>();

    // TODO: Initialize PathPlanning when component is available
    // path_planning_ = std::make_unique<PathPlanning>();

    // TODO: Initialize ShootPlanning when component is available
    // shoot_planning_ = std::make_unique<ShootPlanning>();
}

SetpointCalculation::~SetpointCalculation()
{
    TRACE_FUNCTION();
}

mra_falcons_msgs::msg::Setpoints SetpointCalculation::process(
    const mra_common_msgs::msg::Targets& targets,
    const mra_common_msgs::msg::WorldState& world_state)
{
    TRACE_FUNCTION_INPUTS(targets, world_state);

    auto setpoints_msg = mra_falcons_msgs::msg::Setpoints();

    // Step 1: PathPlanning - Convert unsafe movement target to safe path
    auto safe_movement_target = callPathPlanning(targets.move, world_state);

    // Step 2: VelocityControl - Convert safe movement target to velocity setpoints
    setpoints_msg.velocity = callVelocityControl(safe_movement_target, world_state);

    // Step 3: ShootPlanning - Determine shooting parameters
    setpoints_msg.kicker = callShootPlanning(targets.shoot, world_state);

    // Step 4: BallHandling - Configure ball handlers
    //setpoints_msg.ballhandlers = callBallHandling(targets.grab, world_state);

    TRACE_FUNCTION_OUTPUTS(setpoints_msg);
    return setpoints_msg;
}

mra_common_msgs::msg::MovementTarget SetpointCalculation::callPathPlanning(
    const mra_common_msgs::msg::MovementTarget& unsafe_target,
    const mra_common_msgs::msg::WorldState& world_state)
{
    TRACE_FUNCTION_INPUTS(unsafe_target, world_state);

    // For now, return the unsafe target as-is (no obstacle avoidance)
    auto safe_target = unsafe_target;

    TRACE_FUNCTION_OUTPUTS(safe_target);
    return safe_target;
}

geometry_msgs::msg::Twist SetpointCalculation::callVelocityControl(
    const mra_common_msgs::msg::MovementTarget& safe_target,
    const mra_common_msgs::msg::WorldState& world_state)
{
    TRACE_FUNCTION_INPUTS(safe_target, world_state);

    // Convert ROS2 messages to internal VelocityControl types
    VelocityControlTypes::VelocityControlInput vc_input;

    // Fill world state
    vc_input.world_state.robot.position = convertPoseToPosition2D(world_state.robot.pose);
    vc_input.world_state.robot.velocity = convertTwistToVelocity2D(world_state.robot.velocity);
    vc_input.world_state.robot.active = world_state.robot.active;
    vc_input.world_state.robot.has_ball = world_state.robot.hasball;
    vc_input.world_state.timestamp = world_state.time.sec + world_state.time.nanosec * 1e-9;

    // Fill setpoint from safe movement target
    vc_input.setpoint.position = convertPoseToPosition2D(safe_target.pose);
    vc_input.setpoint.velocity = convertTwistToVelocity2D(safe_target.velocity);
    vc_input.setpoint.has_position = true;
    vc_input.setpoint.has_velocity = true;

    // TODO: Determine motion profile based on robot state and target
    // Could consider factors like: has_ball, distance to target, urgency, etc.
    vc_input.motion_profile_id = world_state.robot.hasball ? 1 : 0;  // Use withBall profile if robot has ball
    vc_input.timestamp = vc_input.world_state.timestamp;

    // Call VelocityControl
    auto vc_output = velocity_control_->calculate(vc_input);

    geometry_msgs::msg::Twist result_twist;
    if (vc_output.success) {
        result_twist = convertVelocity2DToTwist(vc_output.velocity_rcs);
    } else {
        // Return zero velocities on error
        result_twist.linear.x = 0.0;
        result_twist.linear.y = 0.0;
        result_twist.angular.z = 0.0;
    }

    TRACE_FUNCTION_OUTPUTS(result_twist);
    return result_twist;
}

mra_falcons_msgs::msg::Kicker SetpointCalculation::callShootPlanning(
    const mra_common_msgs::msg::ShootingTarget& shoot_target,
    const mra_common_msgs::msg::WorldState& world_state)
{
    TRACE_FUNCTION_INPUTS(shoot_target, world_state);

    mra_falcons_msgs::msg::Kicker kicker_setpoints;

    // Placeholder implementation - disable kicker by default
    //kicker_setpoints.enabled = false;
    kicker_setpoints.power = 0.0;

    TRACE_FUNCTION_OUTPUTS(kicker_setpoints);
    return kicker_setpoints;
}

// Utility functions for data conversion

VelocityControlTypes::Position2D SetpointCalculation::convertPoseToPosition2D(const geometry_msgs::msg::Pose& pose) {
    // TODO: Proper quaternion to yaw conversion
    // For now, using simplified approach assuming pose.orientation.z represents yaw
    return VelocityControlTypes::Position2D(
        pose.position.x,
        pose.position.y,
        pose.orientation.z  // Simplified - should be proper quaternion conversion
    );
}

VelocityControlTypes::Velocity2D SetpointCalculation::convertTwistToVelocity2D(const geometry_msgs::msg::Twist& twist) {
    return VelocityControlTypes::Velocity2D(
        twist.linear.x,
        twist.linear.y,
        twist.angular.z
    );
}

geometry_msgs::msg::Twist SetpointCalculation::convertVelocity2DToTwist(const VelocityControlTypes::Velocity2D& velocity) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = velocity.x;
    twist.linear.y = velocity.y;
    twist.linear.z = 0.0;  // No z-movement for ground robots
    twist.angular.x = 0.0;  // No roll
    twist.angular.y = 0.0;  // No pitch
    twist.angular.z = velocity.rz;  // Yaw rate
    return twist;
}
