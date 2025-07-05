#include "VelocityControlAlgorithms.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>

// CheckPrepareInputs implementation
void CheckPrepareInputs::execute(VelocityControl::VelocityControlData& data) {
    // Check world state
    checkWorldState(data);

    // Determine control mode
    data.control_mode = checkTargetSetpoint(data);

    // Set internal variables
    setInternalVariables(data);
}

void CheckPrepareInputs::checkWorldState(VelocityControl::VelocityControlData& data) {
    // Stop processing if robot is not active
    if (!data.input.world_state.robot.active) {
        data.done = true;
        return;
    }

    // Basic validation - robot state should be valid
    // In the original, this checked for protobuf "has_" fields
    // Here we assume the state is always populated
}

VelocityControlTypes::ControlMode CheckPrepareInputs::checkTargetSetpoint(VelocityControl::VelocityControlData& data) {
    const auto& setpoint = data.input.setpoint;

    if (setpoint.has_position) {
        if (setpoint.has_velocity) {
            return VelocityControlTypes::ControlMode::POSVEL;
        } else {
            return VelocityControlTypes::ControlMode::POS_ONLY;
        }
    } else if (setpoint.has_velocity) {
        return VelocityControlTypes::ControlMode::VEL_ONLY;
    } else {
        // No setpoint given - STOP
        data.done = true;
        return VelocityControlTypes::ControlMode::STOP;
    }
}

void CheckPrepareInputs::setInternalVariables(VelocityControl::VelocityControlData& data) {
    data.current_position_fcs = data.input.world_state.robot.position;
    data.current_velocity_fcs = data.input.world_state.robot.velocity;
    data.target_position_fcs = data.input.setpoint.position;
    data.target_velocity_fcs = data.input.setpoint.velocity;

    data.previous_position_setpoint_fcs = data.state.previous_position_setpoint_fcs;
    data.previous_velocity_setpoint_fcs = data.state.previous_velocity_setpoint_fcs;
}

// ConfigureLimits implementation
void ConfigureLimits::execute(VelocityControl::VelocityControlData& data) {
    int32_t profile_id = data.input.motion_profile_id;

    // Find the appropriate limits
    if (profile_id >= 0 && profile_id < static_cast<int32_t>(data.config.limits.size())) {
        data.current_limits = data.config.limits[profile_id];
    } else {
        // Use default profile (first one)
        if (!data.config.limits.empty()) {
            data.current_limits = data.config.limits[0];
        } else {
            throw std::runtime_error("No motion profiles configured");
        }
    }
}

// CheckStop implementation
void CheckStop::execute(VelocityControl::VelocityControlData& data) {
    if (data.control_mode == VelocityControlTypes::ControlMode::STOP) {
        data.result_velocity_rcs = VelocityControlTypes::Velocity2D(0.0, 0.0, 0.0);
        data.output.velocity_rcs = data.result_velocity_rcs;
        data.output.success = true;
        data.done = true;
    }
}

// Watchdog implementation
void Watchdog::execute(VelocityControl::VelocityControlData& data) {
    double current_time = data.timestamp;
    double last_time = data.state.last_calculation_time;

    if (data.state.initialized && (current_time - last_time) > data.config.timeout) {
        // Timeout exceeded - stop the robot
        data.result_velocity_rcs = VelocityControlTypes::Velocity2D(0.0, 0.0, 0.0);
        data.output.velocity_rcs = data.result_velocity_rcs;
        data.output.success = true;
        data.done = true;
    }
}

// Deadzone implementation
void Deadzone::execute(VelocityControl::VelocityControlData& data) {
    if (!data.config.deadzone.enabled) {
        return;
    }

    if (data.control_mode == VelocityControlTypes::ControlMode::POS_ONLY ||
        data.control_mode == VelocityControlTypes::ControlMode::POSVEL) {

        double dx = data.target_position_fcs.x - data.current_position_fcs.x;
        double dy = data.target_position_fcs.y - data.current_position_fcs.y;
        double drz = VelocityControlTypes::wrap_pi(data.target_position_fcs.rz - data.current_position_fcs.rz);

        double distance_xy = std::sqrt(dx * dx + dy * dy);

        if (distance_xy < data.config.deadzone.tolerance_xy &&
            std::abs(drz) < data.config.deadzone.tolerance_rz) {
            // Within deadzone - stop
            data.result_velocity_rcs = VelocityControlTypes::Velocity2D(0.0, 0.0, 0.0);
            data.output.velocity_rcs = data.result_velocity_rcs;
            data.output.success = true;
            data.done = true;
        }
    }
}

// ShiftBallOffset implementation
void ShiftBallOffset::execute(VelocityControl::VelocityControlData& data) {
    if (!data.config.dribble.apply_limits_to_ball || !data.input.world_state.robot.has_ball) {
        return;
    }

    // Shift positions to account for ball offset
    double radius = data.config.dribble.radius_robot_to_ball;
    double angle = data.current_position_fcs.rz;

    double offset_x = radius * std::cos(angle);
    double offset_y = radius * std::sin(angle);

    data.current_position_fcs.x += offset_x;
    data.current_position_fcs.y += offset_y;

    data.target_position_fcs.x += offset_x;
    data.target_position_fcs.y += offset_y;
}

// UnShiftBallOffset implementation
void UnShiftBallOffset::execute(VelocityControl::VelocityControlData& data) {
    if (!data.config.dribble.apply_limits_to_ball || !data.input.world_state.robot.has_ball) {
        return;
    }

    // The velocity calculated is for the ball position, transform back to robot center
    // In practice, this is often a no-op for velocity, but included for completeness
}

// CalculateVelocity implementation
void CalculateVelocity::execute(VelocityControl::VelocityControlData& data) {
    // Weighted averaging for closed-loop control
    double w_pos = data.config.spg.weight_factor_closed_loop_pos;
    double w_vel = data.config.spg.weight_factor_closed_loop_vel;

    // Handle rotation wrapping for weighted position
    double delta_rz = VelocityControlTypes::wrap_pi(
        data.previous_position_setpoint_fcs.rz - data.current_position_fcs.rz) * w_pos;
    double weighted_rz = data.current_position_fcs.rz + delta_rz;

    VelocityControlTypes::Position2D weighted_current_position_fcs =
        data.current_position_fcs * w_pos + data.previous_position_setpoint_fcs * (1.0 - w_pos);
    weighted_current_position_fcs.rz = weighted_rz;

    VelocityControlTypes::Velocity2D weighted_current_velocity_fcs =
        data.current_velocity_fcs * w_vel + data.previous_velocity_setpoint_fcs * (1.0 - w_vel);

    // Transform to RCS for SPG calculation
    data.delta_position_rcs = data.target_position_fcs.transformFcsToRcs(weighted_current_position_fcs);
    data.delta_position_rcs.rz = VelocityControlTypes::wrap_pi(
        data.target_position_fcs.rz - weighted_current_position_fcs.rz);

    data.current_velocity_rcs = weighted_current_velocity_fcs.transformFcsToRcs(weighted_current_position_fcs);
    data.target_velocity_rcs = data.target_velocity_fcs.transformFcsToRcs(weighted_current_position_fcs);

    // Set up SPG limits with deceleration first
    VelocityControlTypes::SpgLimits spg_limits;
    spg_limits.vx = data.current_limits.max_velocity.x;
    spg_limits.vy = data.current_limits.max_velocity.getY(true);  // forward
    spg_limits.vRz = data.current_limits.max_velocity.rz;
    spg_limits.ax = data.current_limits.max_deceleration.x;
    spg_limits.ay = data.current_limits.max_deceleration.y;
    spg_limits.aRz = data.current_limits.max_deceleration.rz;

    VelocityControlTypes::Position2D result_position;
    VelocityControlTypes::Velocity2D result_velocity;

    // First calculation with deceleration limits
    bool success = calculateSPG(data, spg_limits, result_position, result_velocity);

    // Check if we need to recalculate with acceleration limits
    bool recalculate = false;

    if (isDofAccelerating(data, result_velocity, 0, data.current_limits.acceleration_threshold.x)) {
        recalculate = true;
        spg_limits.ax = data.current_limits.max_acceleration.x;
    }
    if (isDofAccelerating(data, result_velocity, 1, data.current_limits.acceleration_threshold.y)) {
        recalculate = true;
        spg_limits.ay = (result_velocity.y < 0.0) ?
            data.current_limits.max_acceleration.getY(false) :
            data.current_limits.max_acceleration.getY(true);
    }
    if (isDofAccelerating(data, result_velocity, 2, data.current_limits.acceleration_threshold.rz)) {
        recalculate = true;
        spg_limits.aRz = data.current_limits.max_acceleration.rz;
    }
    if (result_velocity.y < 0.0) {
        recalculate = true;
        spg_limits.vy = data.current_limits.max_velocity.getY(false);  // backward
    }

    // Recalculate if needed
    if (recalculate) {
        success = calculateSPG(data, spg_limits, result_position, result_velocity);
    }

    if (!success) {
        throw std::runtime_error("SPG velocity calculation failed");
    }

    // Store result
    data.result_velocity_rcs = result_velocity;

    // Optional convergence workaround
    if (data.config.spg.convergence_workaround) {
        double tolerance = 1e-5;

        // Check if delta is small enough
        bool xy_delta_small = (std::abs(data.delta_position_rcs.x) < tolerance) &&
                             (std::abs(data.delta_position_rcs.y) < tolerance);
        bool rz_delta_small = std::abs(data.delta_position_rcs.rz) < tolerance;

        // Check if SPG converged
        bool xy_spg_converged = (std::abs(result_velocity.x) < tolerance) &&
                               (std::abs(result_velocity.y) < tolerance);
        bool rz_spg_converged = std::abs(result_velocity.rz) < tolerance;

        // Apply linear controller as fallback
        VelocityControl::VelocityControlData tmp_data = data;
        calculateLinear(tmp_data);

        // Overrule if needed
        if (xy_spg_converged && !xy_delta_small) {
            data.result_velocity_rcs.x = tmp_data.result_velocity_rcs.x;
            data.result_velocity_rcs.y = tmp_data.result_velocity_rcs.y;
        }
        if (rz_spg_converged && !rz_delta_small) {
            data.result_velocity_rcs.rz = tmp_data.result_velocity_rcs.rz;
        }
    }

    // Store setpoints for next iteration
    VelocityControlTypes::Position2D tmp_pos = result_position;
    data.previous_position_setpoint_fcs = tmp_pos.transformRcsToFcs(weighted_current_position_fcs);

    VelocityControlTypes::Velocity2D tmp_vel = result_velocity;
    data.previous_velocity_setpoint_fcs = tmp_vel.transformRcsToFcs(weighted_current_position_fcs);
}

bool CalculateVelocity::calculateSPG(VelocityControl::VelocityControlData& data,
                                     const VelocityControlTypes::SpgLimits& limits,
                                     VelocityControlTypes::Position2D& result_position,
                                     VelocityControlTypes::Velocity2D& result_velocity) {
    // Simplified SPG implementation
    // In the original, this used the Reflexxes Type II library
    // Here we implement a basic trajectory generator

    double dt = data.config.dt;

    // For each DOF, calculate the velocity needed
    VelocityControlTypes::Velocity2D target_vel = data.target_velocity_rcs;

    // Position control component
    if (data.control_mode == VelocityControlTypes::ControlMode::POSVEL ||
        data.control_mode == VelocityControlTypes::ControlMode::POS_ONLY) {

        double kp = 3.0;  // Proportional gain

        target_vel.x += kp * data.delta_position_rcs.x;
        target_vel.y += kp * data.delta_position_rcs.y;
        target_vel.rz += kp * data.delta_position_rcs.rz;
    }

    // Apply velocity limits
    target_vel.x = std::max(-limits.vx, std::min(limits.vx, target_vel.x));
    target_vel.y = std::max(-limits.vy, std::min(limits.vy, target_vel.y));
    target_vel.rz = std::max(-limits.vRz, std::min(limits.vRz, target_vel.rz));

    // Apply acceleration limits
    VelocityControlTypes::Velocity2D velocity_diff;
    velocity_diff.x = target_vel.x - data.current_velocity_rcs.x;
    velocity_diff.y = target_vel.y - data.current_velocity_rcs.y;
    velocity_diff.rz = target_vel.rz - data.current_velocity_rcs.rz;

    double max_accel_x = limits.ax * dt;
    double max_accel_y = limits.ay * dt;
    double max_accel_rz = limits.aRz * dt;

    velocity_diff.x = std::max(-max_accel_x, std::min(max_accel_x, velocity_diff.x));
    velocity_diff.y = std::max(-max_accel_y, std::min(max_accel_y, velocity_diff.y));
    velocity_diff.rz = std::max(-max_accel_rz, std::min(max_accel_rz, velocity_diff.rz));

    result_velocity.x = data.current_velocity_rcs.x + velocity_diff.x;
    result_velocity.y = data.current_velocity_rcs.y + velocity_diff.y;
    result_velocity.rz = data.current_velocity_rcs.rz + velocity_diff.rz;

    // Calculate resulting position (for state tracking)
    result_position.x = data.current_position_fcs.x + result_velocity.x * dt;
    result_position.y = data.current_position_fcs.y + result_velocity.y * dt;
    result_position.rz = data.current_position_fcs.rz + result_velocity.rz * dt;

    return true;
}

bool CalculateVelocity::isDofAccelerating(const VelocityControl::VelocityControlData& data,
                                         const VelocityControlTypes::Velocity2D& result_velocity,
                                         int dof, double threshold) {
    std::vector<double> current_vel = {data.current_velocity_rcs.x, data.current_velocity_rcs.y, data.current_velocity_rcs.rz};
    std::vector<double> new_vel = {result_velocity.x, result_velocity.y, result_velocity.rz};

    if (current_vel[dof] < 0.0) {
        return (new_vel[dof] - current_vel[dof]) < (-threshold);
    } else {
        return (new_vel[dof] - current_vel[dof]) > threshold;
    }
}

void CalculateVelocity::calculateLinear(VelocityControl::VelocityControlData& data) {
    // Simple linear controller fallback
    double kp = 2.0;

    data.result_velocity_rcs.x = kp * data.delta_position_rcs.x;
    data.result_velocity_rcs.y = kp * data.delta_position_rcs.y;
    data.result_velocity_rcs.rz = kp * data.delta_position_rcs.rz;

    // Apply limits
    const auto& limits = data.current_limits;
    data.result_velocity_rcs.x = std::max(-limits.max_velocity.x,
                                         std::min(limits.max_velocity.x, data.result_velocity_rcs.x));
    data.result_velocity_rcs.y = std::max(-limits.max_velocity.getY(data.result_velocity_rcs.y >= 0),
                                         std::min(limits.max_velocity.getY(data.result_velocity_rcs.y >= 0), data.result_velocity_rcs.y));
    data.result_velocity_rcs.rz = std::max(-limits.max_velocity.rz,
                                          std::min(limits.max_velocity.rz, data.result_velocity_rcs.rz));
}

// SetOutputsPrepareNext implementation
void SetOutputsPrepareNext::execute(VelocityControl::VelocityControlData& data) {
    // Set output
    data.output.velocity_rcs = data.result_velocity_rcs;
    data.output.success = true;

    // Update state for next iteration
    data.state.previous_position_setpoint_fcs = data.previous_position_setpoint_fcs;
    data.state.previous_velocity_setpoint_fcs = data.previous_velocity_setpoint_fcs;
    data.state.last_calculation_time = data.timestamp;
    data.state.initialized = true;
    data.state.num_algorithms_executed = data.num_algorithms_executed;
    data.state.control_mode = data.control_mode;
}
