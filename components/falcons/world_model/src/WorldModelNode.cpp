#include "WorldModelNode.hpp"
#include "LocalizationFusion.hpp"

namespace falcons
{

class WorldModelNode::Implementation
{
public:
    Implementation()
        : _localization_fusion(std::make_unique<LocalizationFusion>())
    {
        reset();
    }

    void processVision(const std::vector<VisionLandmark>& landmarks,
                      const std::chrono::system_clock::time_point& timestamp)
    {
        _pending_landmarks = landmarks;
        _vision_timestamp = timestamp;
        _has_vision_data = true;

        // Trigger tick if we also have odometry data
        if (_has_odometry_data)
        {
            tick();
        }
    }

    void processFeedback(const OdometryData& odometry)
    {
        _pending_odometry = odometry;
        _has_odometry_data = true;

        // Always trigger tick on odometry (vision is optional)
        tick();
    }

    WorldModelState getWorldState() const
    {
        return _current_state;
    }

    void reset()
    {
        _localization_fusion->reset();
        _current_state = WorldModelState();
        _has_vision_data = false;
        _has_odometry_data = false;
        _pending_landmarks.clear();
    }

private:
    std::unique_ptr<LocalizationFusion> _localization_fusion;
    WorldModelState _current_state;

    // Pending data for processing
    std::vector<VisionLandmark> _pending_landmarks;
    OdometryData _pending_odometry;
    std::chrono::system_clock::time_point _vision_timestamp;
    bool _has_vision_data = false;
    bool _has_odometry_data = false;

    void tick()
    {
        // Create localization input
        LocalizationInput input;
        input.timestamp = _pending_odometry.timestamp;
        input.odometry = _pending_odometry;

        // Add vision data if available
        if (_has_vision_data)
        {
            input.landmarks = _pending_landmarks;
            // Use vision timestamp if it's more recent
            if (_vision_timestamp > input.timestamp)
            {
                input.timestamp = _vision_timestamp;
            }
        }

        // Process localization
        RobotPose updated_pose = _localization_fusion->tick(input);

        // Update world state
        _current_state.timestamp = input.timestamp;
        _current_state.robot_pose = updated_pose;

        // Reset pending data flags
        _has_vision_data = false;
        // Keep odometry data available for next tick
    }
};

WorldModelNode::WorldModelNode()
    : _impl(std::make_unique<Implementation>())
{
}

WorldModelNode::~WorldModelNode() = default;

void WorldModelNode::processVision(const std::vector<VisionLandmark>& landmarks,
                                  const std::chrono::system_clock::time_point& timestamp)
{
    _impl->processVision(landmarks, timestamp);
}

void WorldModelNode::processFeedback(const OdometryData& odometry)
{
    _impl->processFeedback(odometry);
}

WorldModelState WorldModelNode::getWorldState() const
{
    return _impl->getWorldState();
}

void WorldModelNode::reset()
{
    _impl->reset();
}

} // namespace falcons
