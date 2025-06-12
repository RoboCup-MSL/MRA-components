#include "rclcpp/rclcpp.hpp"
#include "action_fetch_ball.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("action_planning_node");
    auto action_fetch_ball = std::make_shared<ActionFetchBall>(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}