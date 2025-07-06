#ifndef LOCALIZATION_FUSION_HPP
#define LOCALIZATION_FUSION_HPP

#include "WorldModelTypes.hpp"
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
     * @param input Combined vision and odometry data
     * @return Updated robot pose with confidence
     */
    RobotPose tick(const LocalizationInput& input);

    /**
     * Get the current best pose estimate
     * @return Current robot pose
     */
    RobotPose getCurrentPose() const;

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
