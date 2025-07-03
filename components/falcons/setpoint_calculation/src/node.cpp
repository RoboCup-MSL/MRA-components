#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include "mra_falcons_configuration/ros_config.hpp"
#include "mra_common_msgs/msg/targets.hpp"
#include "mra_common_msgs/msg/world_state.hpp"
#include "mra_falcons_msgs/msg/setpoints.hpp"
#include "mra_tracing/tracing.hpp"
#include "SetpointCalculation.hpp"

class SetpointCalculationNode : public rclcpp::Node {
public:
    SetpointCalculationNode() : Node("setpoint_calculation") {
        // Get our namespace
        std::string ns = this->get_namespace();
        TRACE_FUNCTION_INPUTS(ns);

        // Initialize the setpoint calculation engine
        setpoint_calculation_ = std::make_unique<SetpointCalculation>();

        // publishers
        publisher_setpoints_ = this->create_publisher<mra_falcons_msgs::msg::Setpoints>("setpoints", FALCONS_ROS_QOS);
        // subscribers
        subscriber_targets_ = this->create_subscription<mra_common_msgs::msg::Targets>(
            "targets", FALCONS_ROS_QOS,
            std::bind(&SetpointCalculationNode::handle_targets, this, std::placeholders::_1)
        );
        subscriber_world_state_ = this->create_subscription<mra_common_msgs::msg::WorldState>(
            "world_state", FALCONS_ROS_QOS,
            std::bind(&SetpointCalculationNode::handle_world_state, this, std::placeholders::_1)
        );
    }

private:
    std::unique_ptr<SetpointCalculation> setpoint_calculation_;
    mra_common_msgs::msg::Targets targets_;
    mra_common_msgs::msg::WorldState world_state_;
    bool targets_received_ = false;
    bool world_state_received_ = false;

    void handle_targets(const mra_common_msgs::msg::Targets::SharedPtr msg) {
        TRACE_FUNCTION();
        targets_ = *msg;
        targets_received_ = true;
        tick();
    }

    void handle_world_state(const mra_common_msgs::msg::WorldState::SharedPtr msg) {
        TRACE_FUNCTION();
        world_state_ = *msg;
        world_state_received_ = true;
        // TODO: address potential race condition
    }

    void tick() {
        TRACE_FUNCTION();

        // Only process if we have received both targets and world state
        if (!targets_received_ || !world_state_received_) {
            return;
        }

        // Use the setpoint calculation library to process the data
        auto setpoints_msg = setpoint_calculation_->process(targets_, world_state_);
        publisher_setpoints_->publish(setpoints_msg);
    }

    rclcpp::Subscription<mra_common_msgs::msg::WorldState>::SharedPtr subscriber_world_state_;
    rclcpp::Subscription<mra_common_msgs::msg::Targets>::SharedPtr subscriber_targets_;
    rclcpp::Publisher<mra_falcons_msgs::msg::Setpoints>::SharedPtr publisher_setpoints_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetpointCalculationNode>());
    rclcpp::shutdown();
    return 0;
}
