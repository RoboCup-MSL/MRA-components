#include "WorldModelConvert.hpp"
#include <cmath>

namespace falcons
{

std::vector<VisionLandmark> WorldModelConvert::fromVisionObjects(const mra_common_msgs::msg::VisionObjects& msg)
{
    std::vector<VisionLandmark> landmarks;

    for (const auto& obj : msg.objects)
    {
        VisionLandmark landmark;
        landmark.type = getVisionLandmarkType(obj.type);
        landmark.azimuth = obj.azimuth;
        landmark.distance = obj.distance;
        landmark.confidence = obj.confidence;

        landmarks.push_back(landmark);
    }

    return landmarks;
}

OdometryData WorldModelConvert::fromFeedback(const falcons_msgs::msg::Feedback& msg)
{
    OdometryData odometry;
    odometry.timestamp = std::chrono::system_clock::now(); // Use current time for now
    odometry.velocity.vx = msg.velocity.linear.x;
    odometry.velocity.vy = msg.velocity.linear.y;
    odometry.velocity.vtheta = msg.velocity.angular.z;

    return odometry;
}

mra_common_msgs::msg::WorldState WorldModelConvert::toWorldState(const WorldModelState& state)
{
    mra_common_msgs::msg::WorldState msg;

    // Set timestamp
    msg.time = toRosTime(state.timestamp);

    // Set robot pose
    msg.robot.id = 0; // TODO: Get actual robot ID
    msg.robot.active = true;
    msg.robot.human = false;

    // Set pose
    msg.robot.pose.position.x = state.robot_pose.pose.x;
    msg.robot.pose.position.y = state.robot_pose.pose.y;
    msg.robot.pose.position.z = 0.0;

    // Convert theta to quaternion (rotation around z-axis)
    double theta = state.robot_pose.pose.theta;
    msg.robot.pose.orientation.x = 0.0;
    msg.robot.pose.orientation.y = 0.0;
    msg.robot.pose.orientation.z = std::sin(theta / 2.0);
    msg.robot.pose.orientation.w = std::cos(theta / 2.0);

    // Set velocity
    msg.robot.velocity.linear.x = state.robot_pose.velocity.vx;
    msg.robot.velocity.linear.y = state.robot_pose.velocity.vy;
    msg.robot.velocity.linear.z = 0.0;
    msg.robot.velocity.angular.x = 0.0;
    msg.robot.velocity.angular.y = 0.0;
    msg.robot.velocity.angular.z = state.robot_pose.velocity.vtheta;

    // Set ball possession (false for now)
    msg.robot.hasball = false;

    // TODO: Set balls, teammates, opponents, obstacles when implemented

    return msg;
}

VisionLandmark::Type WorldModelConvert::getVisionLandmarkType(const std::string& type_str)
{
    if (type_str == "goalpost")
    {
        return VisionLandmark::Type::GOALPOST;
    }
    else if (type_str == "line" || type_str == "line_point")
    {
        return VisionLandmark::Type::LINE_POINT;
    }
    else if (type_str == "corner")
    {
        return VisionLandmark::Type::CORNER;
    }
    else
    {
        return VisionLandmark::Type::GOALPOST; // Default fallback
    }
}

std::chrono::system_clock::time_point WorldModelConvert::fromRosTime(const builtin_interfaces::msg::Time& ros_time)
{
    auto duration = std::chrono::seconds(ros_time.sec) + std::chrono::nanoseconds(ros_time.nanosec);
    return std::chrono::system_clock::time_point(duration);
}

builtin_interfaces::msg::Time WorldModelConvert::toRosTime(const std::chrono::system_clock::time_point& time_point)
{
    builtin_interfaces::msg::Time ros_time;

    auto duration = time_point.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);

    ros_time.sec = static_cast<int32_t>(seconds.count());
    ros_time.nanosec = static_cast<uint32_t>(nanoseconds.count());

    return ros_time;
}

} // namespace falcons
