#include "TeamplayRosNode.hpp"
#include "TeamplayNode.hpp"
#include "mra_tracing/tracing.hpp"

namespace falcons
{

class TeamplayRosNode::Implementation
{
public:
    Implementation(rclcpp::Node* node)
        : node_(node)
        , teamplay_node_()
    {
        // Get our namespace
        std::string ns = node_->get_namespace();
        TRACE_FUNCTION_INPUTS(ns);

        // Publishers
        publisher_action_ = node_->create_publisher<mra_common_msgs::msg::Action>(
            "action", FALCONS_ROS_QOS);

        // Subscribers
        subscriber_world_state_ = node_->create_subscription<mra_common_msgs::msg::WorldState>(
            "world_state", FALCONS_ROS_QOS,
            std::bind(&Implementation::handleWorldState, this, std::placeholders::_1));
    }

private:
    void handleWorldState(const mra_common_msgs::msg::WorldState::SharedPtr msg)
    {
        TRACE_FUNCTION();

        // Process through core teamplay logic
        auto action = teamplay_node_.processWorldState(*msg);

        // Publish the action
        publisher_action_->publish(action);
    }

    rclcpp::Node* node_;
    TeamplayNode teamplay_node_;
    rclcpp::Publisher<mra_common_msgs::msg::Action>::SharedPtr publisher_action_;
    rclcpp::Subscription<mra_common_msgs::msg::WorldState>::SharedPtr subscriber_world_state_;
};

TeamplayRosNode::TeamplayRosNode()
    : Node("teamplay")
    , _impl(std::make_unique<Implementation>(this))
{
    TRACE_FUNCTION();
}

TeamplayRosNode::~TeamplayRosNode() = default;

} // namespace falcons
