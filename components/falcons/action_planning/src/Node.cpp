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

    // synchronized subscribers using message_filters
    world_state_sub_.subscribe(this, "world_state", FALCONS_ROS_QOS.get_rmw_qos_profile());
    action_input_sub_.subscribe(this, "action", FALCONS_ROS_QOS.get_rmw_qos_profile());
    sync_ = std::make_shared<message_filters::TimeSynchronizer<types::WorldState, types::ActionInput>>(
        world_state_sub_, action_input_sub_, 10);
    sync_->registerCallback(std::bind(&ActionPlanningROS::synchronized_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void ActionPlanningROS::synchronized_callback(const types::WorldState::ConstSharedPtr& world_state_msg,
                                            const types::ActionInput::ConstSharedPtr& action_input_msg)
{
    TRACE_FUNCTION_INPUTS(world_state_msg->id, action_input_msg);

    types::Targets targets_msg;
    types::ActionResult action_result_msg;

    // Call the planner with synchronized inputs
    planner_->tick(*world_state_msg, *action_input_msg, action_result_msg, targets_msg);

    // Publish results
    publisher_targets_->publish(targets_msg);
    publisher_action_result_->publish(action_result_msg);

    TRACE_FUNCTION_OUTPUTS(targets_msg, action_result_msg);
}

// main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionPlanningROS>());
    rclcpp::shutdown();
    return 0;
}
