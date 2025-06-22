#pragma once

#include <rclcpp/rclcpp.hpp>
#include "Types.hpp"
#include "ActionPlanner.hpp"

namespace falcons::action_planning
{

class ActionPlanningROS : public rclcpp::Node {
public:
    ActionPlanningROS();

private:
    // subscriber/input handling
    types::WorldState world_state_;
    types::ActionInput action_input_;
    void handle_world_state(const types::WorldState::SharedPtr msg);
    void handle_action_input(const types::ActionInput::SharedPtr msg);

    // main tick function, call ActionPlanner.tick()
    void tick();

    // the planner
    std::unique_ptr<ActionPlanner> planner_;

    // subscribers
    rclcpp::Subscription<types::WorldState>::SharedPtr subscriber_world_state_;
    rclcpp::Subscription<types::ActionInput>::SharedPtr subscriber_action_;

    // publishers
    rclcpp::Publisher<types::Targets>::SharedPtr publisher_targets_;
    rclcpp::Publisher<types::ActionResult>::SharedPtr publisher_action_result_;

}; // class ActionPlanningROS

} // namespace falcons::action_planning
