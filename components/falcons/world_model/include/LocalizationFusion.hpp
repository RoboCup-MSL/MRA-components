#ifndef LOCALIZATION_FUSION_HPP
#define LOCALIZATION_FUSION_HPP

#include "WorldModelTypes.hpp"
#include <vector>
#include <memory>

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
    class Implementation;
    std::unique_ptr<Implementation> _impl;
};

} // namespace falcons

#endif // LOCALIZATION_FUSION_HPP
