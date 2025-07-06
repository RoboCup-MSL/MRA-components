#include <rclcpp/rclcpp.hpp>
#include "mra_falcons_configuration/ros_config.hpp"
#include "mra_common_msgs/msg/world_state.hpp"
#include "mra_common_msgs/msg/vision_objects.hpp"
#include "falcons_msgs/msg/feedback.hpp"
#include "mra_tracing/tracing.hpp"

#include "WorldModelNode.hpp"
#include "WorldModelConvert.hpp"

class WorldModel : public rclcpp::Node
{
public:
    WorldModel() : Node("world_model")
    {
        std::string ns = this->get_namespace();
        TRACE_FUNCTION_INPUTS(ns);

        // Create the core WorldModel component
        _world_model = std::make_unique<falcons::WorldModelNode>();

        // Publisher for world_state
        _publisher_world_state = this->create_publisher<mra_common_msgs::msg::WorldState>(
            "world_state", FALCONS_ROS_QOS);

        // Subscriber for feedback
        _subscriber_feedback = this->create_subscription<falcons_msgs::msg::Feedback>(
            "feedback", FALCONS_ROS_QOS,
            [this](const falcons_msgs::msg::Feedback::SharedPtr msg) {
                this->handleFeedback(msg);
            });

        // Subscriber for vision_objects
        _subscriber_vision_objects = this->create_subscription<mra_common_msgs::msg::VisionObjects>(
            "vision", FALCONS_ROS_QOS,
            [this](const mra_common_msgs::msg::VisionObjects::SharedPtr msg) {
                this->handleVisionObjects(msg);
            });
    }

private:
    std::unique_ptr<falcons::WorldModelNode> _world_model;
    rclcpp::Publisher<mra_common_msgs::msg::WorldState>::SharedPtr _publisher_world_state;
    rclcpp::Subscription<falcons_msgs::msg::Feedback>::SharedPtr _subscriber_feedback;
    rclcpp::Subscription<mra_common_msgs::msg::VisionObjects>::SharedPtr _subscriber_vision_objects;

    void handleFeedback(const falcons_msgs::msg::Feedback::SharedPtr msg)
    {
        TRACE_FUNCTION();

        // Convert ROS2 message to internal type
        falcons::OdometryData odometry = falcons::WorldModelConvert::fromFeedback(*msg);

        // Process with WorldModel
        _world_model->processFeedback(odometry);

        // Publish updated world state
        publishWorldState();
    }

    void handleVisionObjects(const mra_common_msgs::msg::VisionObjects::SharedPtr msg)
    {
        TRACE_FUNCTION();

        // Convert ROS2 message to internal types
        std::vector<falcons::VisionLandmark> landmarks =
            falcons::WorldModelConvert::fromVisionObjects(*msg);

        // Use message timestamp
        auto timestamp = falcons::WorldModelConvert::fromRosTime(msg->timestamp);

        // Process with WorldModel
        _world_model->processVision(landmarks, timestamp);

        // Publish updated world state
        publishWorldState();
    }

    void publishWorldState()
    {
        TRACE_FUNCTION();

        // Get current world state
        falcons::WorldModelState state = _world_model->getWorldState();

        // Convert to ROS2 message and publish
        auto world_state_msg = falcons::WorldModelConvert::toWorldState(state);
        _publisher_world_state->publish(world_state_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WorldModel>());
    rclcpp::shutdown();
    return 0;
}
