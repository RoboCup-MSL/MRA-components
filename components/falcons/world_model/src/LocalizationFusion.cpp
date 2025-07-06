#include "LocalizationFusion.hpp"
#include <algorithm>
#include <cmath>

namespace falcons
{

class LocalizationFusion::Implementation
{
public:
    Implementation()
    {
        reset();
    }

    RobotPose tick(const LocalizationInput& input)
    {
        // Simple implementation for now - will be enhanced later
        // For MVP, just use odometry integration with basic vision correction

        // Update pose based on odometry
        updatePoseFromOdometry(input.odometry);

        // Correct pose using vision landmarks if available
        if (!input.landmarks.empty())
        {
            correctPoseFromVision(input.landmarks);
        }

        // Update timestamp and return current pose
        _current_pose.timestamp = input.timestamp;
        return _current_pose;
    }

    RobotPose getCurrentPose() const
    {
        return _current_pose;
    }

    void reset()
    {
        _current_pose = RobotPose();
        _current_pose.pose = Pose2D(0.0, 0.0, 0.0);  // Start at field center, facing forward
        _current_pose.confidence = 0.1;  // Low initial confidence
        _last_odometry_time = std::chrono::system_clock::time_point{};
    }

private:
    RobotPose _current_pose;
    std::chrono::system_clock::time_point _last_odometry_time;

    void updatePoseFromOdometry(const OdometryData& odometry)
    {
        if (_last_odometry_time.time_since_epoch().count() == 0)
        {
            // First odometry reading - just set velocity
            _current_pose.velocity = odometry.velocity;
            _last_odometry_time = odometry.timestamp;
            return;
        }

        // Calculate time delta
        auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(
            odometry.timestamp - _last_odometry_time).count();

        if (dt <= 0.0 || dt > 1.0)  // Sanity check
        {
            _last_odometry_time = odometry.timestamp;
            return;
        }

        // Simple integration (Euler method)
        double cos_theta = std::cos(_current_pose.pose.theta);
        double sin_theta = std::sin(_current_pose.pose.theta);

        _current_pose.pose.x += (odometry.velocity.vx * cos_theta - odometry.velocity.vy * sin_theta) * dt;
        _current_pose.pose.y += (odometry.velocity.vx * sin_theta + odometry.velocity.vy * cos_theta) * dt;
        _current_pose.pose.theta += odometry.velocity.vtheta * dt;

        // Normalize theta to [-pi, pi]
        while (_current_pose.pose.theta > M_PI) _current_pose.pose.theta -= 2.0 * M_PI;
        while (_current_pose.pose.theta < -M_PI) _current_pose.pose.theta += 2.0 * M_PI;

        // Update velocity
        _current_pose.velocity = odometry.velocity;

        _last_odometry_time = odometry.timestamp;
    }

    void correctPoseFromVision(const std::vector<VisionLandmark>& landmarks)
    {
        // Simple vision correction - just increase confidence if we see landmarks
        // TODO: Implement actual landmark-based pose correction

        double total_confidence = 0.0;
        for (const auto& landmark : landmarks)
        {
            total_confidence += landmark.confidence;
        }

        if (total_confidence > 0.0)
        {
            // Gradually increase confidence when we see good landmarks
            _current_pose.confidence = std::min(1.0, _current_pose.confidence + 0.1 * total_confidence);
        }
        else
        {
            // Gradually decrease confidence when no good landmarks
            _current_pose.confidence = std::max(0.0, _current_pose.confidence - 0.05);
        }
    }
};

LocalizationFusion::LocalizationFusion()
    : _impl(std::make_unique<Implementation>())
{
}

LocalizationFusion::~LocalizationFusion() = default;

RobotPose LocalizationFusion::tick(const LocalizationInput& input)
{
    return _impl->tick(input);
}

RobotPose LocalizationFusion::getCurrentPose() const
{
    return _impl->getCurrentPose();
}

void LocalizationFusion::reset()
{
    _impl->reset();
}

} // namespace falcons
