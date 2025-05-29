#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include "msl_msgs/msg/world_state.hpp"
#include "interfaces/msg/actuation.hpp"
#include "tracing/tracing.hpp"

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
    ActionPlanningROS() : Node("action_planning") {
        // Get our namespace
        std::string ns = this->get_namespace();
        TRACE_FUNCTION_INPUTS(ns);
        // Setup a QoS profile
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos.reliable();
        qos.keep_last(100);
        qos.transient_local();
        // Publisher for actuation
        publisher_actuation_ = this->create_publisher<interfaces::msg::Actuation>("actuation", qos);
        // Subscriber for world_state
        subscriber_world_state_ = this->create_subscription<msl_msgs::msg::WorldState>(
            "world_state", qos,
            [this](const msl_msgs::msg::WorldState::SharedPtr msg) {
                this->handle_world_state(msg);
            });
        // TODO: subscriber for Action from teamplay
        // Publish ready signal (this might positively affect DDS discovery?)
        auto publisher_ready = this->create_publisher<std_msgs::msg::Empty>("ready", qos);
        std_msgs::msg::Empty empty_msg;
        publisher_ready->publish(empty_msg);
    }

private:
    msl_msgs::msg::WorldState world_state_;

    void handle_world_state(const msl_msgs::msg::WorldState::SharedPtr msg) {
        TRACE_FUNCTION();
        world_state_ = *msg;
        tick();
    }

    void tick() {
        TRACE_FUNCTION();
        count_primes_below(50000);
        auto actuation_msg = interfaces::msg::Actuation();
        publisher_actuation_->publish(actuation_msg);
    }

    rclcpp::Publisher<interfaces::msg::Actuation>::SharedPtr publisher_actuation_;
    rclcpp::Subscription<msl_msgs::msg::WorldState>::SharedPtr subscriber_world_state_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionPlanningROS>());
    rclcpp::shutdown();
    return 0;
}