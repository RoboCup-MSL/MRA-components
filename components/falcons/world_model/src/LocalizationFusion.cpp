#include "LocalizationFusion.hpp"
#include "mra_tracing/tracing.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>

namespace falcons
{

class LocalizationFusion::Implementation
{
public:
    Implementation()
    {
        TRACE_FUNCTION();
        reset();
    }

    Pose tick(const std::vector<VisionObject>& vision_objects,
              const Twist& odometry_velocity,
              const Time& timestamp)
    {
        TRACE_FUNCTION_INPUTS(vision_objects, odometry_velocity, timestamp);
        // Simple implementation for now - will be enhanced later
        // For MVP, just use odometry integration with basic vision correction
        // Update pose based on odometry
        updatePoseFromOdometry(odometry_velocity, timestamp);
        // Correct pose using vision objects if available
        if (!vision_objects.empty())
        {
            correctPoseFromVision(vision_objects);
        }
        TRACE_FUNCTION_OUTPUTS(_current_pose);
        return _current_pose;
    }

    Pose getCurrentPose() const
    {
        TRACE_FUNCTION();
        TRACE_FUNCTION_OUTPUTS(_current_pose);
        return _current_pose;
    }

    void reset()
    {
        TRACE_FUNCTION();
        _current_pose = Pose();
        // Initialize pose at field center
        _current_pose.position.x = 0.0;
        _current_pose.position.y = 0.0;
        _current_pose.position.z = 0.0;
        _current_pose.orientation.w = 1.0; // No rotation
        _current_pose.orientation.x = 0.0;
        _current_pose.orientation.y = 0.0;
        _current_pose.orientation.z = 0.0;
        _last_timestamp = Time();
        _has_previous_timestamp = false;
        _confidence = 0.1; // Low initial confidence
    }

private:
    Pose _current_pose;
    Time _last_timestamp;
    bool _has_previous_timestamp = false;
    double _confidence = 0.0;

    void updatePoseFromOdometry(const Twist& velocity, const Time& timestamp)
    {
        TRACE_FUNCTION_INPUTS(velocity, timestamp);
        if (!_has_previous_timestamp)
        {
            _last_timestamp = timestamp;
            _has_previous_timestamp = true;
            return;
        }
        // Calculate time delta
        double dt = timeToSeconds(timestamp) - timeToSeconds(_last_timestamp);
        if (dt <= 0.0 || dt > 1.0) // Sanity check: reject too large or negative time steps
        {
            _last_timestamp = timestamp;
            return;
        }
        // Simple integration: x' = x + vx*dt, y' = y + vy*dt
        // For now, ignore angular velocity and assume robot moves forward
        _current_pose.position.x += velocity.linear.x * dt;
        _current_pose.position.y += velocity.linear.y * dt;
        // Update orientation based on angular velocity
        // Convert quaternion to yaw, update, convert back
        double yaw = quaternionToYaw(_current_pose.orientation);
        yaw += velocity.angular.z * dt;
        _current_pose.orientation = yawToQuaternion(yaw);
        _last_timestamp = timestamp;
        // Decrease confidence over time when relying only on odometry
        _confidence = std::max(0.0, _confidence - 0.01);
        TRACE_FUNCTION_OUTPUTS(_current_pose);
    }

    void correctPoseFromVision(const std::vector<VisionObject>& vision_objects)
    {
        TRACE_FUNCTION_INPUTS(vision_objects);
        // Simple vision correction - just increase confidence if we see objects
        // TODO: Implement actual landmark-based pose correction
        double total_confidence = 0.0;
        for (size_t i = 0; i < vision_objects.size(); ++i)
        {
            // For now, just use the fact that we see objects to boost confidence
            total_confidence += 0.1; // Each object adds some confidence
        }
        if (total_confidence > 0.0)
        {
            // Gradually increase confidence when we see good objects
            _confidence = std::min(1.0, _confidence + total_confidence);
        }
        TRACE_FUNCTION_OUTPUTS(_confidence);
    }

    double timeToSeconds(const Time& time)
    {
        return time.sec + time.nanosec * 1e-9;
    }

    double quaternionToYaw(const geometry_msgs::msg::Quaternion& q)
    {
        // Convert quaternion to yaw (rotation around z-axis)
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
    {
        geometry_msgs::msg::Quaternion q;
        q.w = std::cos(yaw * 0.5);
        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(yaw * 0.5);
        return q;
    }
};

LocalizationFusion::LocalizationFusion()
    : _impl(std::make_unique<Implementation>())
{
    TRACE_FUNCTION();
}

LocalizationFusion::~LocalizationFusion() = default;

Pose LocalizationFusion::tick(const std::vector<VisionObject>& vision_objects,
                              const Twist& odometry_velocity,
                              const Time& timestamp)
{
    TRACE_FUNCTION_INPUTS(vision_objects, odometry_velocity, timestamp);
    auto result = _impl->tick(vision_objects, odometry_velocity, timestamp);
    TRACE_FUNCTION_OUTPUTS(result);
    return result;
}

Pose LocalizationFusion::getCurrentPose() const
{
    TRACE_FUNCTION();
    auto result = _impl->getCurrentPose();
    TRACE_FUNCTION_OUTPUTS(result);
    return result;
}

void LocalizationFusion::reset()
{
    TRACE_FUNCTION();
    _impl->reset();
}

} // namespace falcons
