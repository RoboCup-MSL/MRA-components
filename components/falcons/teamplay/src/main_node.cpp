#include <rclcpp/rclcpp.hpp>
#include "TeamplayRosNode.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<falcons::TeamplayRosNode>());
    rclcpp::shutdown();
    return 0;
}
