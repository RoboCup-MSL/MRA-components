#pragma once

#include "WorldModelTypes.hpp"
#include <vector>

namespace falcons
{

/**
 * LocalizationFusion implements sensor fusion for robot pose estimation.
 * It combines odometry data with vision-based landmark observations to
 * provide robust pose estimation in the field coordinate system.
 */
class LocalizationFusion
{
public:
    LocalizationFusion();
    ~LocalizationFusion();

    /**
     * Process new localization input and update pose estimate
     * @param vision_objects Vision objects from camera
     * @param odometry_velocity Current velocity from odometry
     * @param timestamp Time of the measurement
     * @return Updated robot pose
     */
    Pose tick(const std::vector<VisionObject>& vision_objects,
              const Twist& odometry_velocity,
              const Time& timestamp);

    /**
     * Get the current best pose estimate
     * @return Current robot pose
     */
    Pose getCurrentPose() const;

    /**
     * Reset the localization state (e.g., when robot is repositioned)
     */
    void reset();

private:
    // Helper methods
    void updatePoseFromOdometry(const Twist& velocity, const Time& timestamp);
    void correctPoseFromVision(const std::vector<VisionObject>& vision_objects);
    double timeToSeconds(const Time& time);
    double quaternionToYaw(const geometry_msgs::msg::Quaternion& q);
    geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);

    // Member variables
    Pose _current_pose;
    Time _last_timestamp;
    bool _has_previous_timestamp = false;
    double _confidence = 0.0;
};

} // namespace falcons
