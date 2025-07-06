#include "Node.hpp"
#include "mra_falcons_configuration/ros_config.hpp"
#include "mra_tracing/tracing.hpp"

using namespace falcons::action_planning;
namespace types = mra::common::action_base::types;

ActionPlanningROS::ActionPlanningROS()
    : rclcpp::Node("mra_falcons_action_planning")
{
    std::string ns = this->get_namespace();
    TRACE_FUNCTION_INPUTS(ns);

    // publishers
    publisher_targets_ = this->create_publisher<types::Targets>("targets", FALCONS_ROS_QOS);
    publisher_action_result_ = this->create_publisher<types::ActionResult>("action_result", FALCONS_ROS_QOS);

    // configuration and planner
    //std::string config_file = "action_planning.yaml";
    // TODO: a generic ConfigurationROS facility should know where to find this, for now we overrule
    std::string config_file = "/workspace/MRA/components/falcons/action_planning/config/params.yaml";
    configurator_ = std::make_unique<ConfigurationROS>(this, config_file);
    planner_ = std::make_unique<ActionPlanner>(std::move(configurator_));

    // subscribers
    subscriber_world_state_ = this->create_subscription<types::WorldState>(
        "world_state", FALCONS_ROS_QOS,
        std::bind(&ActionPlanningROS::handle_world_state, this, std::placeholders::_1)
    );
    subscriber_action_ = this->create_subscription<types::ActionInput>(
        "action", FALCONS_ROS_QOS,
        std::bind(&ActionPlanningROS::handle_action_input, this, std::placeholders::_1)
    );
}

void ActionPlanningROS::handle_world_state(const types::WorldState::SharedPtr msg) {
    TRACE_FUNCTION();
    world_state_ = *msg;
}

void ActionPlanningROS::handle_action_input(const types::ActionInput::SharedPtr msg) {
    TRACE_FUNCTION();
    action_input_ = *msg;
    // TODO: guard against race condition with arrival of new world_state? if teamplay would be fast
    tick();
}

void ActionPlanningROS::tick() {
    TRACE_FUNCTION();
    types::Targets targets_msg;
    types::ActionResult action_result_msg;
    planner_->tick(world_state_, action_input_, action_result_msg, targets_msg);
    publisher_targets_->publish(targets_msg);
    publisher_action_result_->publish(action_result_msg);
}

// main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionPlanningROS>());
    rclcpp::shutdown();
    return 0;
}
