#include <rclcpp/rclcpp.hpp>
#include "WorldModelRosNode.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<falcons::WorldModelRosNode>());
    rclcpp::shutdown();
    return 0;
}
