#ifndef WORLD_MODEL_NODE_HPP
#define WORLD_MODEL_NODE_HPP

#include "WorldModelTypes.hpp"
#include "LocalizationFusion.hpp"
#include <memory>
#include <chrono>

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
     * @param landmarks Vision landmarks converted from ROS2 message
     * @param timestamp Time of the observation
     */
    void processVision(const std::vector<VisionLandmark>& landmarks,
                      const std::chrono::system_clock::time_point& timestamp);

    /**
     * Process odometry feedback input
     * @param odometry Odometry data converted from ROS2 message
     */
    void processFeedback(const OdometryData& odometry);

    /**
     * Get the current world state
     * @return Current world state including robot pose
     */
    WorldModelState getWorldState() const;

    /**
     * Reset the world model state
     */
    void reset();

private:
    class Implementation;
    std::unique_ptr<Implementation> _impl;
};

} // namespace falcons

#endif // WORLD_MODEL_NODE_HPP
