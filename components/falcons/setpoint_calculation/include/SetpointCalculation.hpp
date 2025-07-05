#ifndef SETPOINT_CALCULATION_HPP
#define SETPOINT_CALCULATION_HPP

#include "mra_common_msgs/msg/targets.hpp"
#include "mra_common_msgs/msg/world_state.hpp"
#include "mra_falcons_msgs/msg/setpoints.hpp"
#include "VelocityControl.hpp"
#include <memory>

class SetpointCalculation {
public:
    SetpointCalculation();
    ~SetpointCalculation();

    /**
     * Process targets and world state to generate setpoints
     * @param targets The target positions/velocities for the robot
     * @param world_state Current state of the world (robot position, ball, etc.)
     * @return Generated setpoints for robot control
     */
    mra_falcons_msgs::msg::Setpoints process(
        const mra_common_msgs::msg::Targets& targets,
        const mra_common_msgs::msg::WorldState& world_state
    );

private:
    // Component instances
    std::unique_ptr<VelocityControl> velocity_control_;

    // TODO: Add PathPlanning component when available
    // std::unique_ptr<PathPlanning> path_planning_;

    // TODO: Add ShootPlanning component when available
    // std::unique_ptr<ShootPlanning> shoot_planning_;

    // Helper functions for component integration

    /**
     * Call PathPlanning to convert unsafe movement target to safe path
     * @param unsafe_target The original movement target from ActionPlanning
     * @param world_state Current world state with obstacles
     * @return Safe movement target after obstacle avoidance
     */
    mra_common_msgs::msg::MovementTarget callPathPlanning(
        const mra_common_msgs::msg::MovementTarget& unsafe_target,
        const mra_common_msgs::msg::WorldState& world_state
    );

    /**
     * Call VelocityControl to convert movement target to velocity setpoints
     * @param safe_target Safe movement target from PathPlanning
     * @param world_state Current world state
     * @return Velocity setpoints in RCS
     */
    geometry_msgs::msg::Twist callVelocityControl(
        const mra_common_msgs::msg::MovementTarget& safe_target,
        const mra_common_msgs::msg::WorldState& world_state
    );

    /**
     * Call ShootPlanning to determine shooting parameters
     * @param shoot_target Shooting target from ActionPlanning
     * @param world_state Current world state
     * @return Shooting setpoints (kicker configuration)
     */
    mra_falcons_msgs::msg::Kicker callShootPlanning(
        const mra_common_msgs::msg::ShootingTarget& shoot_target,
        const mra_common_msgs::msg::WorldState& world_state
    );

    // Utility functions for data conversion

    /**
     * Convert ROS2 pose to internal VelocityControl position
     */
    VelocityControlTypes::Position2D convertPoseToPosition2D(const geometry_msgs::msg::Pose& pose);

    /**
     * Convert ROS2 twist to internal VelocityControl velocity
     */
    VelocityControlTypes::Velocity2D convertTwistToVelocity2D(const geometry_msgs::msg::Twist& twist);

    /**
     * Convert internal VelocityControl velocity to ROS2 twist
     */
    geometry_msgs::msg::Twist convertVelocity2DToTwist(const VelocityControlTypes::Velocity2D& velocity);
};

#endif // SETPOINT_CALCULATION_HPP
