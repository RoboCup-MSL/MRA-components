#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include "mra_common_msgs/msg/targets.hpp"
#include "mra_common_msgs/msg/world_state.hpp"
#include "mra_falcons_msgs/msg/setpoints.hpp"
#include "mra_tracing/tracing.hpp"

class SetpointCalculation : public rclcpp::Node {
public:
    SetpointCalculation() : Node("setpoint_calculation") {
        // Get our namespace
        std::string ns = this->get_namespace();
        TRACE_FUNCTION_INPUTS(ns);
        // publishers
        int FALCONS_ROS_QOS = 10; // TODO: this magic should go to some central header file
        publisher_setpoints_ = this->create_publisher<mra_falcons_msgs::msg::Setpoints>("setpoints", FALCONS_ROS_QOS);
        // subscribers
        subscriber_targets_ = this->create_subscription<mra_common_msgs::msg::Targets>(
            "targets", FALCONS_ROS_QOS,
            std::bind(&SetpointCalculation::handle_targets, this, std::placeholders::_1)
        );
        subscriber_world_state_ = this->create_subscription<mra_common_msgs::msg::WorldState>(
            "world_state", FALCONS_ROS_QOS,
            std::bind(&SetpointCalculation::handle_world_state, this, std::placeholders::_1)
        );
    }

private:
    mra_common_msgs::msg::Targets targets_;
    mra_common_msgs::msg::WorldState world_state_;

    void handle_targets(const mra_common_msgs::msg::Targets::SharedPtr msg) {
        TRACE_FUNCTION();
        targets_ = *msg;
        tick();
    }

    void handle_world_state(const mra_common_msgs::msg::WorldState::SharedPtr msg) {
        TRACE_FUNCTION();
        world_state_ = *msg;
        // TODO: address potential race condition
    }

    void tick() {
        TRACE_FUNCTION();

        // TODO: implement

        auto setpoints_msg = mra_falcons_msgs::msg::Setpoints();
        publisher_setpoints_->publish(setpoints_msg);
    }

    rclcpp::Subscription<mra_common_msgs::msg::WorldState>::SharedPtr subscriber_world_state_;
    rclcpp::Subscription<mra_common_msgs::msg::Targets>::SharedPtr subscriber_targets_;
    rclcpp::Publisher<mra_falcons_msgs::msg::Setpoints>::SharedPtr publisher_setpoints_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetpointCalculation>());
    rclcpp::shutdown();
    return 0;
}
