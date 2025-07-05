#include "VelocityControl.hpp"
#include "VelocityControlAlgorithms.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

VelocityControl::VelocityControl() {
    setupAlgorithms();
    reset();
}

VelocityControl::~VelocityControl() {
}

void VelocityControl::setupAlgorithms() {
    // The sequence of algorithms matters and is managed here
    algorithms_.clear();
    bool unskippable = true;

    // Check and prepare inputs
    addAlgorithm(std::make_shared<CheckPrepareInputs>());

    // Configure limits based on motion profile
    addAlgorithm(std::make_shared<ConfigureLimits>());

    // Check for stop command
    addAlgorithm(std::make_shared<CheckStop>());

    // Watchdog check (commented out as in original)
    // addAlgorithm(std::make_shared<Watchdog>());

    // Deadzone check
    addAlgorithm(std::make_shared<Deadzone>());

    // Ball offset for dribbling
    addAlgorithm(std::make_shared<ShiftBallOffset>());

    // Main velocity calculation
    addAlgorithm(std::make_shared<CalculateVelocity>());

    // Unshift ball offset
    addAlgorithm(std::make_shared<UnShiftBallOffset>());

    // Set outputs and prepare for next iteration
    addAlgorithm(std::make_shared<SetOutputsPrepareNext>(), unskippable);
}

void VelocityControl::addAlgorithm(std::shared_ptr<VelocityControlAlgorithm> algorithm, bool unskippable) {
    algorithms_.push_back(algorithm);
    algorithm->unskippable = unskippable;
}

VelocityControlTypes::VelocityControlOutput VelocityControl::calculate(
    const VelocityControlTypes::VelocityControlInput& input,
    const VelocityControlTypes::VelocityControlParams& params) {

    // Prepare data structure for algorithm pipeline
    data.input = input;
    data.config = params;
    data.state = state_;
    data.timestamp = input.timestamp;

    // Clear output and processing flags
    data.output = VelocityControlTypes::VelocityControlOutput();
    data.done = false;
    data.num_algorithms_executed = 0;

    try {
        // Execute the algorithm pipeline
        iterate();

        // Update internal state
        state_ = data.state;

        return data.output;
    } catch (const std::exception& e) {
        VelocityControlTypes::VelocityControlOutput error_output;
        error_output.error_message = std::string("VelocityControl error: ") + e.what();
        error_output.success = false;
        return error_output;
    }
}

void VelocityControl::iterate() {
    // Execute the sequence of algorithms
    for (auto& algorithm : algorithms_) {
        if (!data.done || algorithm->unskippable) {
            algorithm->execute(data);
            data.num_algorithms_executed++;
        }
    }
}

void VelocityControl::reset() {
    state_ = VelocityControlTypes::VelocityControlState();
    data = VelocityControlData();
}

const VelocityControlTypes::VelocityControlState& VelocityControl::getState() const {
    return state_;
}

// Utility functions implementation
namespace VelocityControlTypes {

double wrap_pi(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double project_angle_0_2pi(double angle) {
    while (angle < 0) angle += 2 * M_PI;
    while (angle >= 2 * M_PI) angle -= 2 * M_PI;
    return angle;
}

// Transform implementations for Position2D
Position2D Position2D::transformFcsToRcs(const Position2D& robot_position) const {
    double cos_rz = std::cos(robot_position.rz);
    double sin_rz = std::sin(robot_position.rz);

    double dx = x - robot_position.x;
    double dy = y - robot_position.y;

    return Position2D(
        cos_rz * dx + sin_rz * dy,
        -sin_rz * dx + cos_rz * dy,
        wrap_pi(rz - robot_position.rz)
    );
}

Position2D Position2D::transformRcsToFcs(const Position2D& robot_position) const {
    double cos_rz = std::cos(robot_position.rz);
    double sin_rz = std::sin(robot_position.rz);

    return Position2D(
        robot_position.x + cos_rz * x - sin_rz * y,
        robot_position.y + sin_rz * x + cos_rz * y,
        robot_position.rz + rz
    );
}

// Transform implementations for Velocity2D
Velocity2D Velocity2D::transformFcsToRcs(const Position2D& robot_position) const {
    double cos_rz = std::cos(robot_position.rz);
    double sin_rz = std::sin(robot_position.rz);

    return Velocity2D(
        cos_rz * x + sin_rz * y,
        -sin_rz * x + cos_rz * y,
        rz
    );
}

Velocity2D Velocity2D::transformRcsToFcs(const Position2D& robot_position) const {
    double cos_rz = std::cos(robot_position.rz);
    double sin_rz = std::sin(robot_position.rz);

    return Velocity2D(
        cos_rz * x - sin_rz * y,
        sin_rz * x + cos_rz * y,
        rz
    );
}

} // namespace VelocityControlTypes
