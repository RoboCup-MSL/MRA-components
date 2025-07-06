#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include "mra_falcons_configuration/ros_config.hpp"
#include "mra_common_msgs/msg/world_state.hpp"
#include "mra_common_msgs/msg/vision_objects.hpp"
#include "falcons_msgs/msg/feedback.hpp"
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

class WorldModel : public rclcpp::Node {
public:
    WorldModel() : Node("world_model") {
        // Get our namespace
        std::string ns = this->get_namespace();
        TRACE_FUNCTION_INPUTS(ns);
        // Publisher for world_state
        publisher_world_state_ = this->create_publisher<mra_common_msgs::msg::WorldState>("world_state", FALCONS_ROS_QOS);
        // Subscriber for feedback
        subscriber_feedback_ = this->create_subscription<falcons_msgs::msg::Feedback>(
            "feedback", FALCONS_ROS_QOS,
            [this](const falcons_msgs::msg::Feedback::SharedPtr msg) {
                this->handle_feedback(msg);
            });

        // Subscriber for vision_objects
        subscriber_vision_objects_ = this->create_subscription<mra_common_msgs::msg::VisionObjects>(
            "vision", FALCONS_ROS_QOS,
            [this](const mra_common_msgs::msg::VisionObjects::SharedPtr msg) {
                this->handle_vision_objects(msg);
            });
    }

private:
    falcons_msgs::msg::Feedback _feedback;
    mra_common_msgs::msg::VisionObjects _vision_objects;

    void handle_feedback(const falcons_msgs::msg::Feedback::SharedPtr msg) {
        TRACE_FUNCTION();
        // Process feedback message
        _feedback = *msg;
    }

    void handle_vision_objects(const mra_common_msgs::msg::VisionObjects::SharedPtr msg) {
        TRACE_FUNCTION();
        // Process vision_objects message
        _vision_objects = *msg;
        // Trigger the tick function
        tick();
    }

    void tick() {
        TRACE_FUNCTION();
        // Create and populate the world_state message
        auto world_state_msg = mra_common_msgs::msg::WorldState();
        count_primes_below(120000);
        // TODO: Populate world_state_msg with the current world state data
        /*
        world_state_msg.position_x = ...;
        world_state_msg.position_y = ...;
        world_state_msg.orientation = ...;
        world_state_msg.is_active = ...;
        world_state_msg.detected_objects = ...;
        */

        // Publish the world_state message
        publisher_world_state_->publish(world_state_msg);
    }

    rclcpp::Publisher<mra_common_msgs::msg::WorldState>::SharedPtr publisher_world_state_;
    rclcpp::Subscription<falcons_msgs::msg::Feedback>::SharedPtr subscriber_feedback_;
    rclcpp::Subscription<mra_common_msgs::msg::VisionObjects>::SharedPtr subscriber_vision_objects_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WorldModel>());
    rclcpp::shutdown();
    return 0;
}
