#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include "mra_msgs/msg/world_state.hpp"
#include "mra_msgs/msg/targets.hpp"
#include "mra_tracing/tracing.hpp"

void count_primes_below(int lim)
{
    TRACE_FUNCTION_INPUTS(lim);
    int count = 0;
    for (int i = 2; i < lim; ++i) {
        bool is_prime = true;
        for (int j = 2; j * j <= i; ++j) {
            if (i % j == 0) {
                is_prime = false;
                break;
            }
        }
        if (is_prime) {
            count++;
        }
    }
    TRACE_FUNCTION_OUTPUTS(count);
}

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
        publisher_targets_ = this->create_publisher<mra_msgs::msg::Targets>("targets", qos);
        // Subscriber for world_state
        subscriber_world_state_ = this->create_subscription<mra_msgs::msg::WorldState>(
            "world_state", qos,
            [this](const mra_msgs::msg::WorldState::SharedPtr msg) {
                this->handle_world_state(msg);
            });
        // TODO: subscriber for Action from teamplay
        // Publish ready signal (this might positively affect DDS discovery?)
        auto publisher_ready = this->create_publisher<std_msgs::msg::Empty>("ready", qos);
        std_msgs::msg::Empty empty_msg;
        publisher_ready->publish(empty_msg);
    }

private:
    mra_msgs::msg::WorldState world_state_;

    void handle_world_state(const mra_msgs::msg::WorldState::SharedPtr msg) {
        TRACE_FUNCTION();
        world_state_ = *msg;
        tick();
    }

    void tick() {
        TRACE_FUNCTION();
        count_primes_below(50000);
        auto targets_msg = mra_msgs::msg::Targets();
        publisher_targets_->publish(targets_msg);
    }

    rclcpp::Publisher<mra_msgs::msg::Targets>::SharedPtr publisher_targets_;
    rclcpp::Subscription<mra_msgs::msg::WorldState>::SharedPtr subscriber_world_state_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionPlanningROS>());
    rclcpp::shutdown();
    return 0;
}