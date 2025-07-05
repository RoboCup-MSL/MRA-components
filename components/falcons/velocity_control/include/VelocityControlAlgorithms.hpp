#pragma once

#include "VelocityControl.hpp"

/**
 * Abstract base class for velocity control algorithms
 * Each algorithm performs a specific step in the velocity control pipeline
 */
class VelocityControlAlgorithm {
public:
    VelocityControlAlgorithm() = default;
    virtual ~VelocityControlAlgorithm() = default;

    /**
     * Execute the algorithm on the provided data
     * @param data The velocity control data structure containing all inputs, outputs, and intermediate results
     */
    virtual void execute(VelocityControl::VelocityControlData& data) = 0;

    bool unskippable = false;  // If true, algorithm runs even when data.done is set
};

// Algorithm implementations

/**
 * Check and prepare inputs
 * - Validates robot state and setpoint data
 * - Determines control mode (POSVEL, POS_ONLY, VEL_ONLY, STOP)
 * - Sets internal variables for processing
 */
class CheckPrepareInputs : public VelocityControlAlgorithm {
public:
    void execute(VelocityControl::VelocityControlData& data) override;

private:
    void checkWorldState(VelocityControl::VelocityControlData& data);
    VelocityControlTypes::ControlMode checkTargetSetpoint(VelocityControl::VelocityControlData& data);
    void setInternalVariables(VelocityControl::VelocityControlData& data);
};

/**
 * Configure limits based on motion profile
 * - Selects appropriate limits based on motion profile ID
 * - Handles default fallback if profile not found
 */
class ConfigureLimits : public VelocityControlAlgorithm {
public:
    void execute(VelocityControl::VelocityControlData& data) override;
};

/**
 * Check for STOP command
 * - Immediately stops robot when no valid setpoint is provided
 */
class CheckStop : public VelocityControlAlgorithm {
public:
    void execute(VelocityControl::VelocityControlData& data) override;
};

/**
 * Watchdog check
 * - Prevents runaway setpoints by checking timestamp freshness
 */
class Watchdog : public VelocityControlAlgorithm {
public:
    void execute(VelocityControl::VelocityControlData& data) override;
};

/**
 * Deadzone check
 * - Prevents energy waste by stopping robot when target is very close
 */
class Deadzone : public VelocityControlAlgorithm {
public:
    void execute(VelocityControl::VelocityControlData& data) override;
};

/**
 * Shift ball offset for dribbling
 * - Adjusts coordinate system for ball handling
 */
class ShiftBallOffset : public VelocityControlAlgorithm {
public:
    void execute(VelocityControl::VelocityControlData& data) override;
};

/**
 * Unshift ball offset after velocity calculation
 * - Reverts coordinate system adjustment for ball handling
 */
class UnShiftBallOffset : public VelocityControlAlgorithm {
public:
    void execute(VelocityControl::VelocityControlData& data) override;
};

/**
 * Calculate velocity using SPG (SetPoint Generator)
 * - Main velocity calculation algorithm using trajectory generation
 * - Handles acceleration/deceleration limits dynamically
 * - Includes convergence workaround for edge cases
 */
class CalculateVelocity : public VelocityControlAlgorithm {
public:
    void execute(VelocityControl::VelocityControlData& data) override;

private:
    bool calculateSPG(VelocityControl::VelocityControlData& data,
                      const VelocityControlTypes::SpgLimits& limits,
                      VelocityControlTypes::Position2D& result_position,
                      VelocityControlTypes::Velocity2D& result_velocity);

    bool isDofAccelerating(const VelocityControl::VelocityControlData& data,
                          const VelocityControlTypes::Velocity2D& result_velocity,
                          int dof, double threshold);

    // Linear controller fallback for convergence issues
    void calculateLinear(VelocityControl::VelocityControlData& data);
};

/**
 * Set outputs and prepare for next iteration
 * - Stores results and updates state for next calculation
 */
class SetOutputsPrepareNext : public VelocityControlAlgorithm {
public:
    void execute(VelocityControl::VelocityControlData& data) override;
};
