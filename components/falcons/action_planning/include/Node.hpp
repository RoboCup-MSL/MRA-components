#pragma once

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "mra_action_base/ActionTypes.hpp"
#include "ActionPlanner.hpp"
#include "Configuration.hpp"

namespace falcons::action_planning
{

namespace types = mra::common::action_base::types;

class ActionPlanningROS : public rclcpp::Node {
public:
    ActionPlanningROS();

private:
    // synchronized callback for both inputs
    void synchronized_callback(const types::WorldState::ConstSharedPtr& world_state_msg,
                              const types::ActionInput::ConstSharedPtr& action_input_msg);

    // the planner
    std::unique_ptr<ActionPlanner> planner_;

    // configurator
    std::unique_ptr<ConfigurationROS> configurator_;
    types::Settings settings_;

    // synchronized subscribers using message_filters
    message_filters::Subscriber<types::WorldState> world_state_sub_;
    message_filters::Subscriber<types::ActionInput> action_input_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<types::WorldState, types::ActionInput>> sync_;

    // publishers
    rclcpp::Publisher<types::Targets>::SharedPtr publisher_targets_;
    rclcpp::Publisher<types::ActionResult>::SharedPtr publisher_action_result_;

}; // class ActionPlanningROS

} // namespace falcons::action_planning
