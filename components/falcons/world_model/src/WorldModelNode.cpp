#include "WorldModelNode.hpp"
#include "LocalizationFusion.hpp"
#include "mra_tracing/tracing.hpp"

namespace falcons
{

class WorldModelNode::Implementation
{
public:
    Implementation()
        : _localization_fusion(std::make_unique<LocalizationFusion>())
    {
        TRACE_FUNCTION();
        reset();
    }

    void processVision(const std::vector<VisionObject>& vision_objects,
                      const Time& timestamp)
    {
        TRACE_FUNCTION_INPUTS(vision_objects, timestamp);
        _pending_vision_objects = vision_objects;
        _vision_timestamp = timestamp;
        _has_vision_data = true;
        // Trigger tick if we also have odometry data
        if (_has_odometry_data)
        {
            tick();
        }
    }

    void processFeedback(const Twist& velocity, const Time& timestamp)
    {
        TRACE_FUNCTION_INPUTS(velocity, timestamp);
        _pending_velocity = velocity;
        _odometry_timestamp = timestamp;
        _has_odometry_data = true;
    }

    WorldState getWorldState() const
    {
        TRACE_FUNCTION();
        TRACE_FUNCTION_OUTPUTS(_current_state.id);
        return _current_state;
    }

    void reset()
    {
        TRACE_FUNCTION();
        _localization_fusion->reset();
        _current_state = WorldState();
        _has_vision_data = false;
        _has_odometry_data = false;
        _pending_vision_objects.clear();
    }

private:
    std::unique_ptr<LocalizationFusion> _localization_fusion;
    WorldState _current_state;

    // Pending data for processing
    std::vector<VisionObject> _pending_vision_objects;
    Twist _pending_velocity;
    Time _vision_timestamp;
    Time _odometry_timestamp;
    bool _has_vision_data = false;
    bool _has_odometry_data = false;

    void tick()
    {
        _current_state.id++;
        TRACE_FUNCTION_INPUTS(_current_state.id);
        // Determine which timestamp to use (latest available)
        Time processing_timestamp = _odometry_timestamp;
        if (_has_vision_data && timeToSeconds(_vision_timestamp) > timeToSeconds(_odometry_timestamp))
        {
            processing_timestamp = _vision_timestamp;
        }
        // Process localization with ROS types directly
        Pose updated_pose = _localization_fusion->tick(
            _has_vision_data ? _pending_vision_objects : std::vector<VisionObject>(),
            _pending_velocity,
            processing_timestamp
        );
        // Update world state with ROS types directly
        _current_state.time = processing_timestamp;
        _current_state.robot.pose = updated_pose;
        // Reset vision data flag (odometry is kept for next iteration)
        _has_vision_data = false;
        TRACE_FUNCTION_OUTPUTS(_current_state.id);
    }

    double timeToSeconds(const Time& time)
    {
        return time.sec + time.nanosec * 1e-9;
    }
};

WorldModelNode::WorldModelNode()
    : _impl(std::make_unique<Implementation>())
{
    TRACE_FUNCTION();
}

WorldModelNode::~WorldModelNode() = default;

void WorldModelNode::processVision(const std::vector<VisionObject>& vision_objects,
                                  const Time& timestamp)
{
    TRACE_FUNCTION_INPUTS(vision_objects, timestamp);
    _impl->processVision(vision_objects, timestamp);
}

void WorldModelNode::processFeedback(const Twist& velocity, const Time& timestamp)
{
    TRACE_FUNCTION_INPUTS(velocity, timestamp);
    _impl->processFeedback(velocity, timestamp);
}

WorldState WorldModelNode::getWorldState() const
{
    TRACE_FUNCTION();
    auto result = _impl->getWorldState();
    TRACE_FUNCTION_OUTPUTS(result.id);
    return result;
}

void WorldModelNode::reset()
{
    TRACE_FUNCTION();
    _impl->reset();
}

} // namespace falcons
