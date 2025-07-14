#include "WorldModelRosNode.hpp"
#include "WorldModelNode.hpp"
#include "mra_tracing/tracing.hpp"

namespace falcons
{

class WorldModelRosNode::Implementation
{
public:
    Implementation(rclcpp::Node* node) : _node(node)
    {
        std::string ns = _node->get_namespace();
        TRACE_FUNCTION_INPUTS(ns);

        // Create the core WorldModel component
        _world_model = std::make_unique<WorldModelNode>();

        // Publisher for world_state
        _publisher_world_state = _node->create_publisher<mra_common_msgs::msg::WorldState>(
            "world_state", FALCONS_ROS_QOS);

        // Subscriber for feedback
        _subscriber_feedback = _node->create_subscription<falcons_msgs::msg::Feedback>(
            "feedback", FALCONS_ROS_QOS,
            [this](const falcons_msgs::msg::Feedback::SharedPtr msg) {
                this->handleFeedback(msg);
            });

        // Subscriber for vision_objects
        _subscriber_vision_objects = _node->create_subscription<mra_common_msgs::msg::VisionObjects>(
            "vision", FALCONS_ROS_QOS,
            [this](const mra_common_msgs::msg::VisionObjects::SharedPtr msg) {
                this->handleVisionObjects(msg);
            });
    }

private:
    rclcpp::Node* _node;
    std::unique_ptr<WorldModelNode> _world_model;
    rclcpp::Publisher<mra_common_msgs::msg::WorldState>::SharedPtr _publisher_world_state;
    rclcpp::Subscription<falcons_msgs::msg::Feedback>::SharedPtr _subscriber_feedback;
    rclcpp::Subscription<mra_common_msgs::msg::VisionObjects>::SharedPtr _subscriber_vision_objects;

    void handleFeedback(const falcons_msgs::msg::Feedback::SharedPtr msg)
    {
        TRACE_FUNCTION_INPUTS(msg);

        // Use current time since feedback doesn't have timestamp
        Time timestamp = _node->now();

        // Process with WorldModel using ROS types directly
        _world_model->processFeedback(msg->velocity, timestamp);

        // No publishing here - that is tied into vision callback, according to execution architecture
    }

    void handleVisionObjects(const mra_common_msgs::msg::VisionObjects::SharedPtr msg)
    {
        TRACE_FUNCTION_INPUTS(msg);

        // Process with WorldModel using ROS types directly
        _world_model->processVision(msg->objects, msg->timestamp);

        // Publish updated world state
        publishWorldState();
    }

    void publishWorldState()
    {
        TRACE_FUNCTION();

        // Get current world state (already a ROS2 message)
        WorldState world_state_msg = _world_model->getWorldState();
        _publisher_world_state->publish(world_state_msg);

        TRACE_FUNCTION_OUTPUTS(world_state_msg);
        // Tracing: this might be the only place where world_state is published in full,
        // for other functions it is more compact to just refer to the ID
    }
};

WorldModelRosNode::WorldModelRosNode()
    : Node("mra_falcons_world_model"), _impl(std::make_unique<Implementation>(this))
{
    TRACE_FUNCTION();
}

WorldModelRosNode::~WorldModelRosNode() = default;

} // namespace falcons
