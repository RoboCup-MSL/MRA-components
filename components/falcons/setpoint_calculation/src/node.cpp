#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "mra_falcons_configuration/ros_config.hpp"
#include "mra_common_msgs/msg/targets.hpp"
#include "mra_common_msgs/msg/world_state.hpp"
#include "mra_falcons_msgs/msg/setpoints.hpp"
#include "mra_tracing/tracing.hpp"
#include "SetpointCalculation.hpp"

class SetpointCalculationNode : public rclcpp::Node
{
public:
    SetpointCalculationNode() : Node("setpoint_calculation")
    {
        // Get our namespace
        std::string ns = this->get_namespace();
        TRACE_FUNCTION_INPUTS(ns);
        // Initialize the setpoint calculation engine
        setpoint_calculation_ = std::make_unique<SetpointCalculation>();
        // publishers
        publisher_setpoints_ = this->create_publisher<mra_falcons_msgs::msg::Setpoints>("setpoints", FALCONS_ROS_QOS);
        // synchronized subscribers using message_filters
        targets_sub_.subscribe(this, "targets", FALCONS_ROS_QOS.get_rmw_qos_profile());
        world_state_sub_.subscribe(this, "world_state", FALCONS_ROS_QOS.get_rmw_qos_profile());
        sync_ = std::make_shared<message_filters::TimeSynchronizer<mra_common_msgs::msg::Targets, mra_common_msgs::msg::WorldState>>(
            targets_sub_, world_state_sub_, 10);
        sync_->registerCallback(std::bind(&SetpointCalculationNode::synchronized_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    std::unique_ptr<SetpointCalculation> setpoint_calculation_;
    message_filters::Subscriber<mra_common_msgs::msg::Targets> targets_sub_;
    message_filters::Subscriber<mra_common_msgs::msg::WorldState> world_state_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<mra_common_msgs::msg::Targets, mra_common_msgs::msg::WorldState>> sync_;
    rclcpp::Publisher<mra_falcons_msgs::msg::Setpoints>::SharedPtr publisher_setpoints_;

    void synchronized_callback(const mra_common_msgs::msg::Targets::ConstSharedPtr& targets_msg,
                              const mra_common_msgs::msg::WorldState::ConstSharedPtr& world_state_msg)
    {
        TRACE_FUNCTION_INPUTS(targets_msg, world_state_msg);
        // Use the setpoint calculation library to process the data
        auto setpoints_msg = setpoint_calculation_->process(*targets_msg, *world_state_msg);
        publisher_setpoints_->publish(setpoints_msg);
        TRACE_FUNCTION_OUTPUTS(setpoints_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetpointCalculationNode>());
    rclcpp::shutdown();
    return 0;
}
