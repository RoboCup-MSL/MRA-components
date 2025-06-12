#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include "mra_common_msgs/msg/world_state.hpp"
#include "mra_common_msgs/msg/targets.hpp"
#include "mra_falcons_msgs/msg/setpoints.hpp"
#include "mra_tracing/tracing.hpp"

//typedef mra_common_msgs::msg::Targets output_t;
// TODO: remove this temporary simulation PoC workaround (avoid having to create setpoint_processing already)
typedef mra_falcons_msgs::msg::Setpoints output_t;

class ActionPlanningROS : public rclcpp::Node {
public:
    ActionPlanningROS() : Node("mra_falcons_action_planning") {
        // Get our namespace
        std::string ns = this->get_namespace();
        TRACE_FUNCTION_INPUTS(ns);
        // Setup a QoS profile
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos.reliable();
        qos.keep_last(100);
        qos.transient_local();
        // Publisher for targets
        //publisher_targets_ = this->create_publisher<output_t>("targets", qos);
        publisher_targets_ = this->create_publisher<output_t>("setpoints", qos); // TODO: remove this temporary simulation PoC workaround
        // Subscriber for world_state
        subscriber_world_state_ = this->create_subscription<mra_common_msgs::msg::WorldState>(
            "world_state", qos,
            [this](const mra_common_msgs::msg::WorldState::SharedPtr msg) {
                this->handle_world_state(msg);
            });
        // TODO: subscriber for Action from teamplay
        // Publish ready signal (this might positively affect DDS discovery?)
        auto publisher_ready = this->create_publisher<std_msgs::msg::Empty>("ready", qos);
        std_msgs::msg::Empty empty_msg;
        publisher_ready->publish(empty_msg);
    }

private:
    mra_common_msgs::msg::WorldState world_state_;

    void handle_world_state(const mra_common_msgs::msg::WorldState::SharedPtr msg) {
        TRACE_FUNCTION();
        world_state_ = *msg;
        tick();
    }

    void tick() {
        TRACE_FUNCTION();
        auto targets_msg = output_t();
        publisher_targets_->publish(targets_msg);
    }

    rclcpp::Publisher<output_t>::SharedPtr publisher_targets_;
    rclcpp::Subscription<mra_common_msgs::msg::WorldState>::SharedPtr subscriber_world_state_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionPlanningROS>());
    rclcpp::shutdown();
    return 0;
}