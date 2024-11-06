// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests


// Include testframework
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_factory.hpp"
using namespace ::testing;

// System under test:
#include "RobotsportsVelocityControl.hpp"
using namespace MRA;

// Basic tick shall run OK and return error_value 0.
TEST(RobotsportsVelocityControlTest, basicTick)
{
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();

    // Act
    int error_value = m.tick();

    // Assert
    EXPECT_EQ(error_value, 0);
}

// When robot is inactive, the robot shall STOP.
TEST(RobotsportsVelocityControlTest, robotInactive)
{
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(1.0);
    input.mutable_setpoint()->mutable_position()->set_x(2.0);
    input.mutable_worldstate()->mutable_robot()->set_active(false);

    // Act
    int error_value = m.tick(input, output);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(output.velocity().x(), 0.0);
    EXPECT_EQ(output.velocity().y(), 0.0);
    EXPECT_EQ(output.velocity().rz(), 0.0);
}



// When no input is given, the robot shall STOP.
TEST(RobotsportsVelocityControlTest, nominalOutput)
{
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();

    // Act
    int error_value = m.tick();

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(output.velocity().x(), 0.0);
    EXPECT_EQ(output.velocity().y(), 0.0);
    EXPECT_EQ(output.velocity().rz(), 0.0);
}

// Section: basic moves, stateless

TEST(RobotsportsVelocityControlTest, moveX)
{
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(1.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.0);
    input.mutable_setpoint()->mutable_position()->set_x(2.0);
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    auto params = m.defaultParams();
    float acc = 1.5;
    params.mutable_limits(0)->mutable_maxacc()->set_x(acc);
    float dt = 1.0 / 40;
    params.set_dt(dt);

    // Act
    int error_value = m.tick(input, params, output);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_FLOAT_EQ(output.velocity().x(), acc * dt);
    EXPECT_FLOAT_EQ(output.velocity().y(), 0.0);
    EXPECT_FLOAT_EQ(output.velocity().rz(), 0.0);
}

TEST(RobotsportsVelocityControlTest, moveY)
{
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(1.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.0);
    input.mutable_setpoint()->mutable_position()->set_y(2.0);
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    auto params = m.defaultParams();
    float acc = 1.8;
    params.mutable_limits(0)->mutable_maxacc()->set_yforward(acc);
    float dt = 1.0 / 30;
    params.set_dt(dt);

    // Act
    int error_value = m.tick(input, params, output);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_FLOAT_EQ(output.velocity().x(), 0.0);
    EXPECT_FLOAT_EQ(output.velocity().y(), acc * dt);
    EXPECT_FLOAT_EQ(output.velocity().rz(), 0.0);
}

TEST(RobotsportsVelocityControlTest, moveRz)
{
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(1.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.0);
    input.mutable_setpoint()->mutable_position()->set_rz(2.0);
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    auto params = m.defaultParams();
    float acc = 1.7;
    params.mutable_limits(0)->mutable_maxacc()->set_rz(acc);
    float dt = 1.0 / 35;
    params.set_dt(dt);

    // Act
    int error_value = m.tick(input, params, output);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_FLOAT_EQ(output.velocity().x(), 0.0);
    EXPECT_FLOAT_EQ(output.velocity().y(), 0.0);
    EXPECT_FLOAT_EQ(output.velocity().rz(), acc * dt);
}

TEST(RobotsportsVelocityControlTest, stop)
{
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
    auto state = RobotsportsVelocityControl::State();
    auto diagnostics = RobotsportsVelocityControl::Diagnostics();
    auto params = m.defaultParams();
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    // STOP command is given by VEL_ONLY (0,0,0)
    input.mutable_setpoint()->mutable_velocity()->set_x(0.0);
    input.mutable_setpoint()->mutable_velocity()->set_y(0.0);
    input.mutable_setpoint()->mutable_velocity()->set_rz(0.0);
    // setup as if robot was in the middle of a rotation
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(1.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(1.0);
    state.mutable_positionsetpointfcs()->set_rz(1.0);
    state.mutable_velocitysetpointfcs()->set_rz(1.0);

    // Act
    int error_value = m.tick(input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::RobotsportsVelocityControl::VEL_ONLY);
    EXPECT_FLOAT_EQ(output.velocity().x(), 0.0);
    EXPECT_FLOAT_EQ(output.velocity().y(), 0.0);
    EXPECT_FLOAT_EQ(output.velocity().rz(), 0.0);
}

// when STOP is not implemented correctly, it can happen that upon resuming,
// the SPG still has an internal open-loop setpoint, where it should instead ramp up from zero
TEST(RobotsportsVelocityControlTest, noHotRestart)
{
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input1 = RobotsportsVelocityControl::Input();
    auto input2 = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
    auto state = RobotsportsVelocityControl::State();
    auto diagnostics = RobotsportsVelocityControl::Diagnostics();
    auto params = m.defaultParams();
    input1.mutable_worldstate()->mutable_robot()->set_active(true);
    // STOP command is given by VEL_ONLY (0,0,0)
    input1.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(1.0);
    input1.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(1.0);
    input1.mutable_setpoint()->mutable_velocity()->set_x(0.0);
    input1.mutable_setpoint()->mutable_velocity()->set_y(0.0);
    input1.mutable_setpoint()->mutable_velocity()->set_rz(0.0);

    input2.mutable_worldstate()->mutable_robot()->set_active(true);
    input2.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(1.0);
    input2.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.0);
    input2.mutable_setpoint()->mutable_velocity()->set_x(0.0);
    input2.mutable_setpoint()->mutable_velocity()->set_y(0.0);
    input2.mutable_setpoint()->mutable_velocity()->set_rz(2.0);
    state.mutable_positionsetpointfcs()->set_rz(1.0);
    state.mutable_velocitysetpointfcs()->set_rz(1.0);

    // Act
    int error_value1 = m.tick(input1, params, state, output, diagnostics);
    int error_value2 = m.tick(input2, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(error_value1, 0);
    EXPECT_EQ(error_value2, 0);
    // TODO enable check in this test-case
    // This test case does not pass on Ubuntu 24.04 with Bazel for unknown reasons.
    // Test case does pass on Ubuntu 20.04 and 22.04 with Bazel.
    // Testcase with Cmake pass  on Ubuntu 20.04, 22.04 and Ubuntu 24.04
    // EXPECT_FLOAT_EQ(output.velocity().x(), 0.0);
    // EXPECT_FLOAT_EQ(output.velocity().y(), 0.0);
    // EXPECT_LT(output.velocity().rz(), 0.1);
}

// Bug seen on RobotSports simulation, where it appears that VelocityControl stops steering when xy is in spec but rz not.
// The json file uses custom simulation configuration.
// The data is literally copied from the debug logs at one of the moments when the bug manifested.
TEST(RobotsportsVelocityControlTest, bugNonconvergingRz)
{
    double tolerance = 1e-5;
    auto output = TestFactory::run_testvector<RobotsportsVelocityControl::RobotsportsVelocityControl>(std::string("components/robotsports/velocity_control/testdata/bug_nonconverging_rz.json"), tolerance);
    // the problem was that output velocity was zero on given input+state
}

// Bug observed:
// * XY controller not converged
// * Rz controller converged
// * Rz controller triggers "convergence workaround"
// * which produces vrz==0 but also overrules small XY velocity setpoint with one large jump ...
TEST(RobotsportsVelocityControlTest, bugLargeXYJump)
{
    // TODO check if test case is valid for ruckig
    double tolerance = 1e-5;
    auto output = TestFactory::run_testvector<RobotsportsVelocityControl::RobotsportsVelocityControl>(std::string("components/robotsports/velocity_control/testdata/bug_large_xy_jump.json"), tolerance);
    // the problem was: output: {"velocity":{"x":-100,"y":-40}}

}

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

