#pragma once

#include "VelocityControlTypes.hpp"
#include <memory>

// Forward declarations for algorithm classes
class VelocityControlAlgorithm;

class VelocityControl {
public:
    VelocityControl();
    ~VelocityControl();

    /**
     * Calculate velocity setpoint based on current state and target
     * @param input Current robot state and target setpoint
     * @param params Configuration parameters
     * @return Velocity setpoint in Robot Coordinate System (RCS)
     */
    VelocityControlTypes::VelocityControlOutput calculate(
        const VelocityControlTypes::VelocityControlInput& input,
        const VelocityControlTypes::VelocityControlParams& params = VelocityControlTypes::VelocityControlParams()
    );

    /**
     * Reset the internal state (useful for initialization or after interruptions)
     */
    void reset();

    /**
     * Get the current internal state (for debugging/monitoring)
     */
    const VelocityControlTypes::VelocityControlState& getState() const;

    // Public for testing purposes
    struct VelocityControlData {
        VelocityControlTypes::VelocityControlInput input;
        VelocityControlTypes::VelocityControlParams config;
        VelocityControlTypes::VelocityControlState state;
        VelocityControlTypes::VelocityControlOutput output;

        // Algorithm execution control
        bool done = false;
        int num_algorithms_executed = 0;

        // Internal processing variables
        VelocityControlTypes::ControlMode control_mode = VelocityControlTypes::ControlMode::INVALID;
        VelocityControlTypes::Limits current_limits;

        // Coordinate system variables
        VelocityControlTypes::Position2D current_position_fcs;
        VelocityControlTypes::Velocity2D current_velocity_fcs;
        VelocityControlTypes::Position2D target_position_fcs;
        VelocityControlTypes::Velocity2D target_velocity_fcs;
        VelocityControlTypes::Position2D previous_position_setpoint_fcs;
        VelocityControlTypes::Velocity2D previous_velocity_setpoint_fcs;

        // SPG internal variables
        VelocityControlTypes::Position2D delta_position_rcs;
        VelocityControlTypes::Velocity2D current_velocity_rcs;
        VelocityControlTypes::Velocity2D target_velocity_rcs;

        // Result
        VelocityControlTypes::Velocity2D result_velocity_rcs;

        double timestamp = 0.0;
    } data;

private:
    VelocityControlTypes::VelocityControlState state_;
    std::vector<std::shared_ptr<VelocityControlAlgorithm>> algorithms_;

    // Algorithm setup and execution
    void setupAlgorithms();
    void addAlgorithm(std::shared_ptr<VelocityControlAlgorithm> algorithm, bool unskippable = false);
    void iterate();
};
