#include "VelocityControl.hpp"
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "Testing VelocityControl migration..." << std::endl;

    // Create VelocityControl instance
    VelocityControl velocity_control;

    // Create test input
    VelocityControlTypes::VelocityControlInput input;

    // Setup robot state
    input.world_state.robot.position = VelocityControlTypes::Position2D(0.0, 0.0, 0.0);
    input.world_state.robot.velocity = VelocityControlTypes::Velocity2D(0.0, 0.0, 0.0);
    input.world_state.robot.active = true;
    input.world_state.robot.has_ball = false;
    input.world_state.timestamp = 1.0;

    // Setup target setpoint
    input.setpoint.position = VelocityControlTypes::Position2D(1.0, 1.0, 0.5);
    input.setpoint.velocity = VelocityControlTypes::Velocity2D(0.0, 0.0, 0.0);
    input.setpoint.has_position = true;
    input.setpoint.has_velocity = true;

    input.motion_profile_id = 0;
    input.timestamp = 1.0;

    // Test basic calculation
    auto output = velocity_control.calculate(input);

    std::cout << "Test 1 - Basic position control:" << std::endl;
    std::cout << "  Success: " << (output.success ? "YES" : "NO") << std::endl;
    if (output.success) {
        std::cout << "  Velocity RCS: ("
                  << std::fixed << std::setprecision(3)
                  << output.velocity_rcs.x << ", "
                  << output.velocity_rcs.y << ", "
                  << output.velocity_rcs.rz << ")" << std::endl;
    } else {
        std::cout << "  Error: " << output.error_message << std::endl;
    }

    // Test velocity-only control
    input.setpoint.has_position = false;
    input.setpoint.has_velocity = true;
    input.setpoint.velocity = VelocityControlTypes::Velocity2D(0.5, 0.3, 0.2);

    output = velocity_control.calculate(input);

    std::cout << "\nTest 2 - Velocity-only control:" << std::endl;
    std::cout << "  Success: " << (output.success ? "YES" : "NO") << std::endl;
    if (output.success) {
        std::cout << "  Velocity RCS: ("
                  << std::fixed << std::setprecision(3)
                  << output.velocity_rcs.x << ", "
                  << output.velocity_rcs.y << ", "
                  << output.velocity_rcs.rz << ")" << std::endl;
    } else {
        std::cout << "  Error: " << output.error_message << std::endl;
    }

    // Test STOP command
    input.setpoint.has_position = false;
    input.setpoint.has_velocity = false;

    output = velocity_control.calculate(input);

    std::cout << "\nTest 3 - STOP command:" << std::endl;
    std::cout << "  Success: " << (output.success ? "YES" : "NO") << std::endl;
    if (output.success) {
        std::cout << "  Velocity RCS: ("
                  << std::fixed << std::setprecision(3)
                  << output.velocity_rcs.x << ", "
                  << output.velocity_rcs.y << ", "
                  << output.velocity_rcs.rz << ")" << std::endl;
    } else {
        std::cout << "  Error: " << output.error_message << std::endl;
    }

    // Test coordinate transformations
    VelocityControlTypes::Position2D robot_pos(1.0, 2.0, M_PI/4);
    VelocityControlTypes::Position2D target_pos(3.0, 4.0, M_PI/2);

    auto target_rcs = target_pos.transformFcsToRcs(robot_pos);
    auto target_back = target_rcs.transformRcsToFcs(robot_pos);

    std::cout << "\nTest 4 - Coordinate transformations:" << std::endl;
    std::cout << "  Original FCS: (" << target_pos.x << ", " << target_pos.y << ", " << target_pos.rz << ")" << std::endl;
    std::cout << "  Transformed RCS: (" << target_rcs.x << ", " << target_rcs.y << ", " << target_rcs.rz << ")" << std::endl;
    std::cout << "  Back to FCS: (" << target_back.x << ", " << target_back.y << ", " << target_back.rz << ")" << std::endl;

    double error = std::abs(target_back.x - target_pos.x) +
                  std::abs(target_back.y - target_pos.y) +
                  std::abs(target_back.rz - target_pos.rz);
    std::cout << "  Transformation error: " << error << std::endl;

    if (error < 1e-10) {
        std::cout << "  Coordinate transformations: PASS" << std::endl;
    } else {
        std::cout << "  Coordinate transformations: FAIL" << std::endl;
    }

    std::cout << "\nVelocityControl migration test completed!" << std::endl;
    return 0;
}
