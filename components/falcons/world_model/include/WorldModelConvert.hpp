#ifndef WORLD_MODEL_CONVERT_HPP
#define WORLD_MODEL_CONVERT_HPP

#include "WorldModelTypes.hpp"
#include "mra_common_msgs/msg/world_state.hpp"
#include "mra_common_msgs/msg/vision_objects.hpp"
#include "falcons_msgs/msg/feedback.hpp"
#include <vector>

namespace falcons
{

/**
 * Conversion functions between ROS2 messages and WorldModel types
 */
class WorldModelConvert
{
public:
    // Convert ROS2 messages to internal types
    static std::vector<VisionLandmark> fromVisionObjects(const mra_common_msgs::msg::VisionObjects& msg);
    static OdometryData fromFeedback(const falcons_msgs::msg::Feedback& msg);
    static std::chrono::system_clock::time_point fromRosTime(const builtin_interfaces::msg::Time& ros_time);

    // Convert internal types to ROS2 messages
    static mra_common_msgs::msg::WorldState toWorldState(const WorldModelState& state);
    static builtin_interfaces::msg::Time toRosTime(const std::chrono::system_clock::time_point& time_point);

private:
    static VisionLandmark::Type getVisionLandmarkType(const std::string& type_str);
};

} // namespace falcons

#endif // WORLD_MODEL_CONVERT_HPP
