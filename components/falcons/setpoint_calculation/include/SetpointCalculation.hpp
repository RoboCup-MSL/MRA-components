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
    // VelocityControl component for velocity processing
    std::unique_ptr<VelocityControl> velocity_control_;

    // TODO: Add any other internal state variables here
};

#endif // SETPOINT_CALCULATION_HPP
