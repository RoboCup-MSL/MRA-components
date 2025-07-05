#include "gtest/gtest.h"
#include "VelocityControl.hpp"
#include "VelocityControlTypes.hpp"

using namespace VelocityControlTypes;

class VelocityControlTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        velocity_control_ = std::make_unique<VelocityControl>();
    }

    void TearDown() override
    {
        velocity_control_.reset();
    }

    std::unique_ptr<VelocityControl> velocity_control_;
};

// Basic calculation shall run OK and return success.
TEST_F(VelocityControlTest, basicCalculation)
{
    // Arrange
    VelocityControlInput input;
    input.world_state.robot.position = Position2D(0.0, 0.0, 0.0);
    input.world_state.robot.velocity = Velocity2D(0.0, 0.0, 0.0);
    input.world_state.robot.active = true;
    input.setpoint.position = Position2D(1.0, 1.0, 0.0);
    input.setpoint.has_position = true;
    input.motion_profile_id = 0;
    input.timestamp = 1.0;

    // Act
    auto output = velocity_control_->calculate(input);

    // Assert
    EXPECT_TRUE(output.success);
}

// When robot is inactive, the robot shall STOP.
TEST_F(VelocityControlTest, robotInactive)
{
    // Arrange
    VelocityControlInput input;
    input.world_state.robot.position = Position2D(1.0, 0.0, 0.0);
    input.world_state.robot.velocity = Velocity2D(0.0, 0.0, 0.0);
    input.world_state.robot.active = false;  // Robot is inactive
    input.setpoint.position = Position2D(2.0, 0.0, 0.0);
    input.setpoint.has_position = true;
    input.motion_profile_id = 0;
    input.timestamp = 1.0;

    // Act
    auto output = velocity_control_->calculate(input);

    // Assert
    EXPECT_TRUE(output.success);
    EXPECT_FLOAT_EQ(output.velocity_rcs.x, 0.0);
    EXPECT_FLOAT_EQ(output.velocity_rcs.y, 0.0);
    EXPECT_FLOAT_EQ(output.velocity_rcs.rz, 0.0);
}

// When no setpoint is given, the robot shall STOP.
TEST_F(VelocityControlTest, noSetpoint)
{
    // Arrange
    VelocityControlInput input;
    input.world_state.robot.position = Position2D(1.0, 1.0, 0.0);
    input.world_state.robot.velocity = Velocity2D(0.0, 0.0, 0.0);
    input.world_state.robot.active = true;
    // No setpoint provided (has_position = false, has_velocity = false)
    input.motion_profile_id = 0;
    input.timestamp = 1.0;

    // Act
    auto output = velocity_control_->calculate(input);

    // Assert
    EXPECT_TRUE(output.success);
    EXPECT_FLOAT_EQ(output.velocity_rcs.x, 0.0);
    EXPECT_FLOAT_EQ(output.velocity_rcs.y, 0.0);
    EXPECT_FLOAT_EQ(output.velocity_rcs.rz, 0.0);
}

// Section: basic moves, stateless

TEST_F(VelocityControlTest, moveX)
{
    // Arrange
    VelocityControlInput input;
    input.world_state.robot.position = Position2D(1.0, 0.0, 0.0);
    input.world_state.robot.velocity = Velocity2D(0.0, 0.0, 0.0);
    input.world_state.robot.active = true;
    input.setpoint.position = Position2D(2.0, 0.0, 0.0);  // Move 1m in X
    input.setpoint.has_position = true;
    input.motion_profile_id = 0;
    input.timestamp = 1.0;

    // Configure motion profile for predictable acceleration
    VelocityControlParams params;
    params.limits[0].max_acceleration.x = 1.5;  // 1.5 m/s²
    params.dt = 1.0 / 40.0;  // 40 Hz

    // Act
    auto output = velocity_control_->calculate(input, params);

    // Assert
    EXPECT_TRUE(output.success);
    // Should accelerate in X direction
    EXPECT_GT(output.velocity_rcs.x, 0.0);
    EXPECT_FLOAT_EQ(output.velocity_rcs.y, 0.0);
    EXPECT_FLOAT_EQ(output.velocity_rcs.rz, 0.0);
}

TEST_F(VelocityControlTest, moveY)
{
    // Arrange
    VelocityControlInput input;
    input.world_state.robot.position = Position2D(0.0, 1.0, 0.0);
    input.world_state.robot.velocity = Velocity2D(0.0, 0.0, 0.0);
    input.world_state.robot.active = true;
    input.setpoint.position = Position2D(0.0, 2.0, 0.0);  // Move 1m in Y
    input.setpoint.has_position = true;
    input.motion_profile_id = 0;
    input.timestamp = 1.0;

    // Configure motion profile for predictable acceleration
    VelocityControlParams params;
    params.limits[0].max_acceleration.y = 1.8;  // 1.8 m/s²
    params.dt = 1.0 / 30.0;  // 30 Hz

    // Act
    auto output = velocity_control_->calculate(input, params);

    // Assert
    EXPECT_TRUE(output.success);
    // Should accelerate in Y direction
    EXPECT_FLOAT_EQ(output.velocity_rcs.x, 0.0);
    EXPECT_GT(output.velocity_rcs.y, 0.0);
    EXPECT_FLOAT_EQ(output.velocity_rcs.rz, 0.0);
}

TEST_F(VelocityControlTest, moveRz)
{
    // Arrange
    VelocityControlInput input;
    input.world_state.robot.position = Position2D(0.0, 0.0, 1.0);
    input.world_state.robot.velocity = Velocity2D(0.0, 0.0, 0.0);
    input.world_state.robot.active = true;
    input.setpoint.position = Position2D(0.0, 0.0, 2.0);  // Rotate 1 rad
    input.setpoint.has_position = true;
    input.motion_profile_id = 0;
    input.timestamp = 1.0;

    // Configure motion profile for predictable acceleration
    VelocityControlParams params;
    params.limits[0].max_acceleration.rz = 1.7;  // 1.7 rad/s²
    params.dt = 1.0 / 35.0;  // 35 Hz

    // Act
    auto output = velocity_control_->calculate(input, params);

    // Assert
    EXPECT_TRUE(output.success);
    // Should accelerate in rotation
    EXPECT_FLOAT_EQ(output.velocity_rcs.x, 0.0);
    EXPECT_FLOAT_EQ(output.velocity_rcs.y, 0.0);
    EXPECT_GT(output.velocity_rcs.rz, 0.0);
}

TEST_F(VelocityControlTest, stopCommand)
{
    // Arrange
    VelocityControlInput input;
    input.world_state.robot.position = Position2D(1.0, 1.0, 0.0);
    input.world_state.robot.velocity = Velocity2D(1.0, 1.0, 0.5);  // Robot moving
    input.world_state.robot.active = true;
    // STOP command is given by VEL_ONLY with (0,0,0)
    input.setpoint.velocity = Velocity2D(0.0, 0.0, 0.0);
    input.setpoint.has_velocity = true;
    input.setpoint.has_position = false;
    input.motion_profile_id = 0;
    input.timestamp = 1.0;

    // Act
    auto output = velocity_control_->calculate(input);

    // Assert
    EXPECT_TRUE(output.success);
    // Should decelerate towards zero
    EXPECT_LT(std::abs(output.velocity_rcs.x), std::abs(input.world_state.robot.velocity.x));
    EXPECT_LT(std::abs(output.velocity_rcs.y), std::abs(input.world_state.robot.velocity.y));
    EXPECT_LT(std::abs(output.velocity_rcs.rz), std::abs(input.world_state.robot.velocity.rz));
}

// Test motion profile selection
TEST_F(VelocityControlTest, motionProfileWithBall)
{
    // Arrange
    VelocityControlInput input;
    input.world_state.robot.position = Position2D(0.0, 0.0, 0.0);
    input.world_state.robot.velocity = Velocity2D(0.0, 0.0, 0.0);
    input.world_state.robot.active = true;
    input.world_state.robot.has_ball = true;  // Robot has ball
    input.setpoint.position = Position2D(1.0, 0.0, 0.0);
    input.setpoint.has_position = true;
    input.motion_profile_id = 1;  // Use "withBall" profile
    input.timestamp = 1.0;

    // Act
    auto output = velocity_control_->calculate(input);

    // Assert
    EXPECT_TRUE(output.success);
    // Should use more conservative acceleration with ball
    EXPECT_GT(output.velocity_rcs.x, 0.0);
}

// Test error handling
TEST_F(VelocityControlTest, invalidMotionProfile)
{
    // Arrange
    VelocityControlInput input;
    input.world_state.robot.position = Position2D(0.0, 0.0, 0.0);
    input.world_state.robot.velocity = Velocity2D(0.0, 0.0, 0.0);
    input.world_state.robot.active = true;
    input.setpoint.position = Position2D(1.0, 0.0, 0.0);
    input.setpoint.has_position = true;
    input.motion_profile_id = 999;  // Invalid profile ID
    input.timestamp = 1.0;

    // Act
    auto output = velocity_control_->calculate(input);

    // Assert - should gracefully handle invalid profile
    EXPECT_TRUE(output.success);  // Should fallback to default profile
}

// Test coordinate system transforms
TEST_F(VelocityControlTest, coordinateTransforms)
{
    // Test Position2D transforms
    Position2D robot_pos(1.0, 2.0, M_PI_2);  // Robot at (1,2) facing 90°
    Position2D target_fcs(2.0, 2.0, 0.0);    // Target at (2,2) in FCS

    // Transform target from FCS to RCS
    Position2D target_rcs = target_fcs.transformFcsToRcs(robot_pos);

    // In RCS, target should be at (0, -1) relative to robot
    EXPECT_NEAR(target_rcs.x, 0.0, 1e-6);
    EXPECT_NEAR(target_rcs.y, -1.0, 1e-6);

    // Transform back to FCS
    Position2D target_fcs_back = target_rcs.transformRcsToFcs(robot_pos);
    EXPECT_NEAR(target_fcs_back.x, target_fcs.x, 1e-6);
    EXPECT_NEAR(target_fcs_back.y, target_fcs.y, 1e-6);
}
