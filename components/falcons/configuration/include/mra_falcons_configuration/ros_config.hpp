#pragma once

#define FALCONS_ROS_QOS make_qos()

#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>

inline rclcpp::QoS make_qos()
{
    // Setup a QoS profile
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos.reliable();
    qos.keep_last(100);
    qos.transient_local();
    return qos;
}
