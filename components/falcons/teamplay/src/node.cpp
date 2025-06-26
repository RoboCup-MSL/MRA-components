#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include "mra_falcons_configuration/ros_config.hpp"
#include "mra_common_msgs/msg/action.hpp"
#include "mra_common_msgs/msg/action_type.hpp"
#include "mra_common_msgs/msg/world_state.hpp"
#include "mra_tracing/tracing.hpp"

class Teamplay : public rclcpp::Node {
public:
    Teamplay() : Node("teamplay") {
        // Get our namespace
        std::string ns = this->get_namespace();
        TRACE_FUNCTION_INPUTS(ns);
        // publishers
        publisher_action_ = this->create_publisher<mra_common_msgs::msg::Action>("action", FALCONS_ROS_QOS);
        // subscribers
        subscriber_world_state_ = this->create_subscription<mra_common_msgs::msg::WorldState>(
            "world_state", FALCONS_ROS_QOS,
            std::bind(&Teamplay::handle_world_state, this, std::placeholders::_1)
        );
    }

private:
    mra_common_msgs::msg::WorldState world_state_;

    void handle_world_state(const mra_common_msgs::msg::WorldState::SharedPtr msg) {
        TRACE_FUNCTION();
        world_state_ = *msg;
        tick();
    }

    // TODO: also handle action_planning result

    void tick() {

        // MVP implementation: cycle through available actions
        static int ticks_remaining = 0;
        static auto action_msg = mra_common_msgs::msg::Action();
        TRACE_FUNCTION_INPUTS(ticks_remaining, action_msg.type);
        if (--ticks_remaining <= 0) {
            // Cycle through a few actions
            if (action_msg.type == mra_common_msgs::msg::ActionType::ACTION_GETBALL) {
                action_msg.type = mra_common_msgs::msg::ActionType::ACTION_STOP;
            } else if (action_msg.type == mra_common_msgs::msg::ActionType::ACTION_STOP) {
                action_msg.type = mra_common_msgs::msg::ActionType::ACTION_MOVE;
            } else {
                action_msg.type = mra_common_msgs::msg::ActionType::ACTION_GETBALL;
            }
            // Reset the tick counter
            ticks_remaining = 10;
        }

        // Publish the world_state message
        publisher_action_->publish(action_msg);
        TRACE_FUNCTION_OUTPUTS(ticks_remaining, action_msg.type);
    }

    rclcpp::Publisher<mra_common_msgs::msg::Action>::SharedPtr publisher_action_;
    rclcpp::Subscription<mra_common_msgs::msg::WorldState>::SharedPtr subscriber_world_state_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teamplay>());
    rclcpp::shutdown();
    return 0;
}
