#include "Node.hpp"
#include "mra_tracing/tracing.hpp"

using namespace falcons::action_planning;

ActionPlanningROS::ActionPlanningROS()
    : rclcpp::Node("mra_falcons_action_planning")
    , planner_(std::make_unique<ActionPlanner>())
{
    std::string ns = this->get_namespace();
    TRACE_FUNCTION_INPUTS(ns);
    // publishers
    int queue_size = 10; // TODO: make configurable?
    publisher_targets_ = this->create_publisher<types::Targets>("targets", queue_size);
    publisher_action_result_ = this->create_publisher<types::ActionResult>("action_result", queue_size);
    // subscribers
    subscriber_world_state_ = this->create_subscription<types::WorldState>(
        "world_state", queue_size,
        std::bind(&ActionPlanningROS::handle_world_state, this, std::placeholders::_1)
    );
    subscriber_action_ = this->create_subscription<types::ActionInput>(
        "action", queue_size,
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
    //TODO
    //mra_falcons_msgs::msg::Setpoints setpoints_msg;
    //planner_->tick(world_state_, setpoints_msg);
    //publisher_targets_->publish(setpoints_msg);
}

// main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionPlanningROS>());
    rclcpp::shutdown();
    return 0;
}
