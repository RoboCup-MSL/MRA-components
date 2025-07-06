#pragma once

#include "WorldModelTypes.hpp"
#include <memory>
#include <vector>

namespace falcons
{

/**
 * WorldModelNode implements the core WorldModel functionality.
 * It orchestrates the various sub-components and maintains the world state.
 */
class WorldModelNode
{
public:
    WorldModelNode();
    ~WorldModelNode();

    /**
     * Process vision objects input
     * @param vision_objects Vision objects from ROS2 message
     * @param timestamp Time of the observation
     */
    void processVision(const std::vector<VisionObject>& vision_objects,
                      const Time& timestamp);

    /**
     * Process odometry feedback input
     * @param velocity Odometry velocity from ROS2 message
     * @param timestamp Time of the measurement
     */
    void processFeedback(const Twist& velocity, const Time& timestamp);

    /**
     * Get the current world state
     * @return Current world state as ROS2 message
     */
    WorldState getWorldState() const;

    /**
     * Reset the world model state
     */
    void reset();

private:
    class Implementation;
    std::unique_ptr<Implementation> _impl;
};

} // namespace falcons
