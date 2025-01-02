// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests


// Include testframework
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_factory.hpp"
using namespace ::testing;

// System under test:
#include "FalconsVelocityControl.hpp"
using namespace MRA;

// Basic tick shall run OK and return error_value 0.
TEST(FalconsVelocityControlTest, basicTick)
{
    // Arrange
    auto m = FalconsVelocityControl::FalconsVelocityControl();

    // Act
    int error_value = m.tick();

    // Assert
    EXPECT_EQ(error_value, 0);
}

// When robot is inactive, the robot shall STOP.
TEST(FalconsVelocityControlTest, robotInactive)
{
    // Arrange
    auto m = FalconsVelocityControl::FalconsVelocityControl();
    auto input = FalconsVelocityControl::Input();
    auto output = FalconsVelocityControl::Output();
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
TEST(FalconsVelocityControlTest, nominalOutput)
{
    // Arrange
    auto m = FalconsVelocityControl::FalconsVelocityControl();
    auto input = FalconsVelocityControl::Input();
    auto output = FalconsVelocityControl::Output();

    // Act
    int error_value = m.tick();

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(output.velocity().x(), 0.0);
    EXPECT_EQ(output.velocity().y(), 0.0);
    EXPECT_EQ(output.velocity().rz(), 0.0);
}

// Section: basic moves, stateless

TEST(FalconsVelocityControlTest, moveX)
{
    // Arrange
    auto m = FalconsVelocityControl::FalconsVelocityControl();
    auto input = FalconsVelocityControl::Input();
    auto output = FalconsVelocityControl::Output();
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

TEST(FalconsVelocityControlTest, moveY)
{
    // Arrange
    auto m = FalconsVelocityControl::FalconsVelocityControl();
    auto input = FalconsVelocityControl::Input();
    auto output = FalconsVelocityControl::Output();
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

TEST(FalconsVelocityControlTest, moveRz)
{
    // Arrange
    auto m = FalconsVelocityControl::FalconsVelocityControl();
    auto input = FalconsVelocityControl::Input();
    auto output = FalconsVelocityControl::Output();
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

TEST(FalconsVelocityControlTest, stop)
{
    // Arrange
    auto m = FalconsVelocityControl::FalconsVelocityControl();
    auto input = FalconsVelocityControl::Input();
    auto output = FalconsVelocityControl::Output();
    auto state = FalconsVelocityControl::State();
    auto diagnostics = FalconsVelocityControl::Diagnostics();
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
    EXPECT_EQ(diagnostics.controlmode(), MRA::FalconsVelocityControl::VEL_ONLY);
    EXPECT_FLOAT_EQ(output.velocity().x(), 0.0);
    EXPECT_FLOAT_EQ(output.velocity().y(), 0.0);
    EXPECT_FLOAT_EQ(output.velocity().rz(), 0.0);
}

// when STOP is not implemented correctly, it can happen that upon resuming,
// the SPG still has an internal open-loop setpoint, where it should instead ramp up from zero
TEST(FalconsVelocityControlTest, noHotRestart)
{
    // Arrange
    auto m = FalconsVelocityControl::FalconsVelocityControl();
    auto input1 = FalconsVelocityControl::Input();
    auto input2 = FalconsVelocityControl::Input();
    auto output = FalconsVelocityControl::Output();
    auto state = FalconsVelocityControl::State();
    auto diagnostics = FalconsVelocityControl::Diagnostics();
    auto params = m.defaultParams();
    input1.mutable_worldstate()->mutable_robot()->set_active(true);
    input2.mutable_worldstate()->mutable_robot()->set_active(true);
    // STOP command is given by VEL_ONLY (0,0,0)
    input1.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(1.0);
    input1.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(1.0);
    input1.mutable_setpoint()->mutable_velocity()->set_x(0.0);
    input1.mutable_setpoint()->mutable_velocity()->set_y(0.0);
    input1.mutable_setpoint()->mutable_velocity()->set_rz(0.0);
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
    EXPECT_FLOAT_EQ(output.velocity().x(), 0.0);
    EXPECT_FLOAT_EQ(output.velocity().y(), 0.0);
    EXPECT_LT(output.velocity().rz(), 0.1);
}

// Bug seen on Falcons simulation, where it appears that VelocityControl stops steering when xy is in spec but rz not.
// The json file uses custom simulation configuration.
// The data is literally copied from the debug logs at one of the moments when the bug manifested.
TEST(FalconsVelocityControlTest, bugNonconvergingRz)
{
    double tolerance = 1e-5;
    auto output = TestFactory::run_testvector<FalconsVelocityControl::FalconsVelocityControl>(std::string("components/falcons/velocity_control/testdata/bug_nonconverging_rz.json"), tolerance);
    // the problem was that output velocity was zero on given input+state
}

// Bug observed:
// * XY controller not converged
// * Rz controller converged
// * Rz controller triggers "convergence workaround"
// * which produces vrz==0 but also overrules small XY velocity setpoint with one large jump ...
TEST(FalconsVelocityControlTest, bugLargeXYJump)
{
    double tolerance = 1e-5;
    auto output = TestFactory::run_testvector<FalconsVelocityControl::FalconsVelocityControl>(std::string("components/falcons/velocity_control/testdata/bug_large_xy_jump.json"), tolerance);
    // the problem was: output: {"velocity":{"x":-100,"y":-40}}
}


// Section: basic moves, stateless
TEST(FalconsVelocityControlTest, moveYwhenRotated) {
    // Arrange
    auto m = FalconsVelocityControl::FalconsVelocityControl();
    auto input = FalconsVelocityControl::Input();
    auto output = FalconsVelocityControl::Output();
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-1.0);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(3.0);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(0.5*M_PI);
    // input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.0);
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    input.mutable_setpoint()->mutable_position()->set_x(-1.0);
    input.mutable_setpoint()->mutable_position()->set_y(5.0);
    input.mutable_setpoint()->mutable_position()->set_rz(0.5*M_PI);
    // input.mutable_setpoint()->mutable_position()->set_rz(0.0);
    auto params = m.defaultParams();
    auto state = FalconsVelocityControl::State();
    auto diagnostics = FalconsVelocityControl::Diagnostics();

    // Act
    // std::cout << "input: " << MRA::convert_proto_to_json_str(input) << std::endl << std::flush;
    // std::cout << "state in: " << MRA::convert_proto_to_json_str(state) << std::endl << std::flush;
    int error_value = m.tick(input, params, state, output, diagnostics);
    // std::cout << "state: out: " << MRA::convert_proto_to_json_str(params) << std::endl << std::flush;
    // std::cout << "diagnostics: out: " << MRA::convert_proto_to_json_str(diagnostics) << std::endl << std::flush;
    // std::cout << "output: " << MRA::convert_proto_to_json_str(output) << std::endl << std::flush;

    // Assert
    // difference between the FCS positions is only in Y direction
    // but player is rotated 90 degrees on the field 
    // Then in RCS this is a X change
    EXPECT_EQ(error_value, 0);
    EXPECT_FLOAT_EQ(output.velocity().x(), 0.025); 
    EXPECT_FLOAT_EQ(output.velocity().y(), 0.0);
    EXPECT_FLOAT_EQ(output.velocity().rz(), 0.0);
}

// Section: basic moves, stateless
TEST(FalconsVelocityControlTest, velocityYwhenRotated) {
    // Arrange
    auto m = FalconsVelocityControl::FalconsVelocityControl();
    auto input = FalconsVelocityControl::Input();
    auto output = FalconsVelocityControl::Output();
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(1.0);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(3.0);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(0.5*M_PI);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.0);
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    input.mutable_setpoint()->mutable_velocity()->set_x(0.0);
    input.mutable_setpoint()->mutable_velocity()->set_y(1.0);
    input.mutable_setpoint()->mutable_velocity()->set_rz(0);
    // input.mutable_setpoint()->mutable_position()->set_rz(0.0);
    auto params = m.defaultParams();
    auto state = FalconsVelocityControl::State();
    auto diagnostics = FalconsVelocityControl::Diagnostics();

    // Act
    // std::cout << "input: " << MRA::convert_proto_to_json_str(input) << std::endl << std::flush;
    // std::cout << "state in: " << MRA::convert_proto_to_json_str(state) << std::endl << std::flush;
    int error_value = m.tick(input, params, state, output, diagnostics);
    // std::cout << "state: out: " << MRA::convert_proto_to_json_str(params) << std::endl << std::flush;
    // std::cout << "diagnostics: out: " << MRA::convert_proto_to_json_str(diagnostics) << std::endl << std::flush;
    // std::cout << "output: " << MRA::convert_proto_to_json_str(output) << std::endl << std::flush;

    // Assert
    // difference between the FCS positions is only in Y direction
    // but player is rotated 90 degrees on the field 
    // Then in RCS this is a X change
    EXPECT_EQ(error_value, 0);
    EXPECT_FLOAT_EQ(output.velocity().x(), 0.025); 
    EXPECT_NEAR(output.velocity().y(), 0.0, 1e-15);
    EXPECT_FLOAT_EQ(output.velocity().rz(), 0.0);
}


TEST(FalconsVelocityControlTest, velocityOnly) {
    // Arrange
    auto m = FalconsVelocityControl::FalconsVelocityControl();
    auto input = FalconsVelocityControl::Input();
    auto output = FalconsVelocityControl::Output();
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-1.00);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-6.50);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.0);
    input.mutable_setpoint()->mutable_velocity()->set_x(-1.0);
    input.mutable_setpoint()->mutable_velocity()->set_y(1.0);
    input.mutable_setpoint()->mutable_velocity()->set_rz(1.25);
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    auto params = m.defaultParams();
    auto state = FalconsVelocityControl::State();
    auto diagnostics = FalconsVelocityControl::Diagnostics();

    // Act
    int error_value = m.tick(input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::FalconsVelocityControl::VEL_ONLY);
    EXPECT_LT(output.velocity().x(), 0.0);  // negative x direction (exact number is not checked)
    EXPECT_GT(output.velocity().y(), 0.0);  // positive y direction (exact number is not checked)
    EXPECT_GT(output.velocity().rz(), 0.0); // positive rz direction (exact number is not checked)
}

TEST(FalconsVelocityControlTest, velocityXmin) {
    // Arrange
    auto m = FalconsVelocityControl::FalconsVelocityControl();
    auto input = FalconsVelocityControl::Input();
    auto output = FalconsVelocityControl::Output();
    // input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-1.00);
    // input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-6.50);
    // input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(-1.57);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(0.00);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.0);
    input.mutable_setpoint()->mutable_velocity()->set_x(-1.0);
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    auto params = m.defaultParams();
    auto state = FalconsVelocityControl::State();
    auto diagnostics = FalconsVelocityControl::Diagnostics();

    // Act
    int error_value = m.tick(input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::FalconsVelocityControl::VEL_ONLY);
    EXPECT_LT(output.velocity().x(), 0.0);
    EXPECT_EQ(output.velocity().y(), 0.0);
    EXPECT_EQ(output.velocity().rz(), 0.0);
}

TEST(FalconsVelocityControlTest, velocityXplus) {
    // Arrange
    auto m = FalconsVelocityControl::FalconsVelocityControl();
    auto input = FalconsVelocityControl::Input();
    auto output = FalconsVelocityControl::Output();
    // input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-1.00);
    // input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-6.50);
    // input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(-1.57);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(0.00);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.0);
    input.mutable_setpoint()->mutable_velocity()->set_x(1.0);
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    auto params = m.defaultParams();
    auto state = FalconsVelocityControl::State();
    auto diagnostics = FalconsVelocityControl::Diagnostics();

    int error_value = m.tick(input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::FalconsVelocityControl::VEL_ONLY);
    EXPECT_GT(output.velocity().x(), 0.0);
    EXPECT_EQ(output.velocity().y(), 0.0);
    EXPECT_EQ(output.velocity().rz(), 0.0);
}


TEST(FalconsVelocityControlTest, velocityYmin) {
    // Arrange
    auto m = FalconsVelocityControl::FalconsVelocityControl();
    auto input = FalconsVelocityControl::Input();
    auto output = FalconsVelocityControl::Output();
    // input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-1.00);
    // input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-6.50);
    // input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(-1.57);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(0.00);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.0);
    input.mutable_setpoint()->mutable_velocity()->set_y(-1.0);
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    auto params = m.defaultParams();
    auto state = FalconsVelocityControl::State();
    auto diagnostics = FalconsVelocityControl::Diagnostics();

    // Act
    int error_value = m.tick(input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::FalconsVelocityControl::VEL_ONLY);
    EXPECT_EQ(output.velocity().x(), 0.0);
    EXPECT_LT(output.velocity().y(), 0.0);
    EXPECT_EQ(output.velocity().rz(), 0.0);
}

TEST(FalconsVelocityControlTest, velocityYplus) {
    // Arrange
    auto m = FalconsVelocityControl::FalconsVelocityControl();
    auto input = FalconsVelocityControl::Input();
    auto output = FalconsVelocityControl::Output();
    // input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-1.00);
    // input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-6.50);
    // input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(-1.57);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(0.00);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.0);
    input.mutable_setpoint()->mutable_velocity()->set_y(+1.0);
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    auto params = m.defaultParams();
    auto state = FalconsVelocityControl::State();
    auto diagnostics = FalconsVelocityControl::Diagnostics();

    // Act
    int error_value = m.tick(input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::FalconsVelocityControl::VEL_ONLY);
    EXPECT_EQ(output.velocity().x(), 0.0);
    EXPECT_GT(output.velocity().y(), 0.0);
    EXPECT_EQ(output.velocity().rz(), 0.0);
}

TEST(FalconsVelocityControlTest, velocityRz) {
    // Arrange
    auto m = FalconsVelocityControl::FalconsVelocityControl();
    auto input = FalconsVelocityControl::Input();
    auto output = FalconsVelocityControl::Output();
    // input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-1.00);
    // input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-6.50);
    // input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(-1.57);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(0.00);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.0);
    input.mutable_setpoint()->mutable_velocity()->set_rz(1.0);
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    auto params = m.defaultParams();
    auto state = FalconsVelocityControl::State();
    auto diagnostics = FalconsVelocityControl::Diagnostics();

    // Act
    int error_value = m.tick(input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::FalconsVelocityControl::VEL_ONLY);
    EXPECT_EQ(output.velocity().x(), 0.0);
    EXPECT_EQ(output.velocity().y(), 0.0);
    EXPECT_GT(output.velocity().rz(), 0.0);
}

int main(int argc, char **argv) {
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}


