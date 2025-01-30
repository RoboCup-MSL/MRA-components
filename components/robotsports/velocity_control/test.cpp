// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests

// Include testframework
#include "test_factory.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
using namespace ::testing;

// System under test:
#include "RobotsportsVelocityControl.hpp"
using namespace MRA;

// Basic tick shall run OK and return error_value 0.
TEST(RobotsportsVelocityControlTest, basicTick) {
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();

    // Act
    int error_value = m.tick();

    // Assert
    EXPECT_EQ(error_value, 0);
}

// When robot is inactive, the robot shall STOP.
TEST(RobotsportsVelocityControlTest, robotInactive) {
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
TEST(RobotsportsVelocityControlTest, nominalOutput) {
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

TEST(RobotsportsVelocityControlTest, moveX) {
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

TEST(RobotsportsVelocityControlTest, moveY) {
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

TEST(RobotsportsVelocityControlTest, moveRz) {
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

TEST(RobotsportsVelocityControlTest, stop) {
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
TEST(RobotsportsVelocityControlTest, noHotRestart) {
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

// Bug seen on RobotSports simulation, where it appears that VelocityControl stops steering when xy is in spec but rz
// not. The json file uses custom simulation configuration. The data is literally copied from the debug logs at one of
// the moments when the bug manifested.
TEST(RobotsportsVelocityControlTest, bugNonconvergingRz) {
    double tolerance = 1e-5;
    auto output = TestFactory::run_testvector<RobotsportsVelocityControl::RobotsportsVelocityControl>(
        std::string("components/robotsports/velocity_control/testdata/bug_nonconverging_rz.json"), tolerance);
    // the problem was that output velocity was zero on given input+state
}

// Bug observed:
// * XY controller not converged
// * Rz controller converged
// * Rz controller triggers "convergence workaround"
// * which produces vrz==0 but also overrules small XY velocity setpoint with one large jump ...
TEST(RobotsportsVelocityControlTest, bugLargeXYJump) {
    // TODO check if test case is valid for ruckig
    double tolerance = 1e-5;
    auto output = TestFactory::run_testvector<RobotsportsVelocityControl::RobotsportsVelocityControl>(
        std::string("components/robotsports/velocity_control/testdata/bug_large_xy_jump.json"), tolerance);
    // the problem was: output: {"velocity":{"x":-100,"y":-40}}
}

TEST(RobotsportsVelocityControlTest, velocityOnly) {
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-1.00);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-6.50);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(-1.57);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.0);
    input.mutable_setpoint()->mutable_velocity()->set_x(-1.0);
    input.mutable_setpoint()->mutable_velocity()->set_y(1.0);
    input.mutable_setpoint()->mutable_velocity()->set_rz(1.25);
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    auto params = m.defaultParams();
    auto state = RobotsportsVelocityControl::State();
    auto diagnostics = RobotsportsVelocityControl::Diagnostics();

    // Act
    int error_value = m.tick(input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::RobotsportsVelocityControl::VEL_ONLY);
    EXPECT_LT(output.velocity().x(), 0.0);
    EXPECT_LT(output.velocity().y(), 0.0);
    EXPECT_GT(output.velocity().rz(), 0.0);
}

TEST(RobotsportsVelocityControlTest, velocityXmin) {
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
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
    auto state = RobotsportsVelocityControl::State();
    auto diagnostics = RobotsportsVelocityControl::Diagnostics();

    // Act
    int error_value = m.tick(input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::RobotsportsVelocityControl::VEL_ONLY);
    EXPECT_LT(output.velocity().x(), 0.0);
    EXPECT_EQ(output.velocity().y(), 0.0);
    EXPECT_EQ(output.velocity().rz(), 0.0);
}

TEST(RobotsportsVelocityControlTest, velocityXplus) {
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
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
    auto state = RobotsportsVelocityControl::State();
    auto diagnostics = RobotsportsVelocityControl::Diagnostics();

    // std::cout << "input: " << MRA::convert_proto_to_json_str(input) << std::endl << std::flush;
    // std::cout << "params : " << MRA::convert_proto_to_json_str(params) << std::endl << std::flush;
    // std::cout << "state: in" << MRA::convert_proto_to_json_str(state) << std::endl << std::flush;

    // Act
    int error_value = m.tick(input, params, state, output, diagnostics);

    // std::cout << "state: out: " << MRA::convert_proto_to_json_str(params) << std::endl << std::flush;
    // std::cout << "diagnostics: out: " << MRA::convert_proto_to_json_str(diagnostics) << std::endl << std::flush;
    // std::cout << "output: " << MRA::convert_proto_to_json_str(output) << std::endl << std::flush;

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::RobotsportsVelocityControl::VEL_ONLY);
    EXPECT_GT(output.velocity().x(), 0.0);
    EXPECT_EQ(output.velocity().y(), 0.0);
    EXPECT_EQ(output.velocity().rz(), 0.0);
}


TEST(RobotsportsVelocityControlTest, velocityYmin) {
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
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
    auto state = RobotsportsVelocityControl::State();
    auto diagnostics = RobotsportsVelocityControl::Diagnostics();

    // Act
    int error_value = m.tick(input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::RobotsportsVelocityControl::VEL_ONLY);
    EXPECT_EQ(output.velocity().x(), 0.0);
    EXPECT_LT(output.velocity().y(), 0.0);
    EXPECT_EQ(output.velocity().rz(), 0.0);
}

TEST(RobotsportsVelocityControlTest, velocityYplus) {
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
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
    auto state = RobotsportsVelocityControl::State();
    auto diagnostics = RobotsportsVelocityControl::Diagnostics();

    // Act
    int error_value = m.tick(input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::RobotsportsVelocityControl::VEL_ONLY);
    EXPECT_EQ(output.velocity().x(), 0.0);
    EXPECT_GT(output.velocity().y(), 0.0);
    EXPECT_EQ(output.velocity().rz(), 0.0);
}

TEST(RobotsportsVelocityControlTest, velocityRz) {
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
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
    auto state = RobotsportsVelocityControl::State();
    auto diagnostics = RobotsportsVelocityControl::Diagnostics();

    // Act
    int error_value = m.tick(input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::RobotsportsVelocityControl::VEL_ONLY);
    EXPECT_EQ(output.velocity().x(), 0.0);
    EXPECT_EQ(output.velocity().y(), 0.0);
    EXPECT_GT(output.velocity().rz(), 0.0);
}

TEST(RobotsportsVelocityControlTest, velocityRequestAboveLimit) {
   auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
    input.mutable_worldstate()->mutable_robot()->set_active(true);

    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-6.1062859763517805);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-0.592884603155586);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(-1.2719768475271225);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.047766801145174911);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.014720211058855398);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.0);
    input.mutable_setpoint()->mutable_velocity()->set_y(1.8);
    auto params = m.defaultParams();
    auto state = RobotsportsVelocityControl::State();
    state.mutable_positionsetpointfcs()->set_x(-6.1063780534136143);
    state.mutable_positionsetpointfcs()->set_y(-0.59286236871876907);
    state.mutable_velocitysetpointfcs()->set_y(0.05000000074505806);
    auto diagnostics = RobotsportsVelocityControl::Diagnostics();

    // std::cout << "input: " << MRA::convert_proto_to_json_str(input) << std::endl << std::flush;
    // std::cout << "params : " << MRA::convert_proto_to_json_str(params) << std::endl << std::flush;
    // std::cout << "state: in" << MRA::convert_proto_to_json_str(state) << std::endl << std::flush;

    // Act
    int error_value = m.tick(input, params, state, output, diagnostics);

    // std::cout << "state: out: " << MRA::convert_proto_to_json_str(params) << std::endl << std::flush;
    // std::cout << "diagnostics: out: " << MRA::convert_proto_to_json_str(diagnostics) << std::endl << std::flush;
    // std::cout << "output: " << MRA::convert_proto_to_json_str(output) << std::endl << std::flush;

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::RobotsportsVelocityControl::VEL_ONLY);
    EXPECT_EQ(output.velocity().x(), 0.0);
    EXPECT_GT(output.velocity().y(), 0.0);
    EXPECT_EQ(output.velocity().rz(), 0.0);
}




TEST(RobotsportsVelocityControlTest, velocityRequestAboveLimit2) {
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
    input.mutable_worldstate()->mutable_robot()->set_active(true);

    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-2.6900212760939319);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(0.57369125183351255);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(-1.2059715777447386);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.046696402131037519);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.0178268879302128);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.0);
    input.mutable_setpoint()->mutable_velocity()->set_y(2.4);
    auto params = RobotsportsVelocityControl::Params();
    params.set_dt(0.025);
    params.set_timeout(0.1);
    params.mutable_spg()->set_weightfactorclosedlooppos(1.0);
    params.mutable_dribble()->set_applylimitstoball(true);
    params.mutable_dribble()->set_radiusrobottoball(0.26);
    params.mutable_deadzone()->set_enabled(true);
    params.mutable_deadzone()->set_tolerancexy(0.01);
    params.mutable_deadzone()->set_tolerancerz(0.005);
    params.mutable_limits()->Clear();
    RobotsportsVelocityControl::Limits limit_default = RobotsportsVelocityControl::Limits();
    limit_default.set_name("default");
    limit_default.mutable_maxvel()->set_x(2.5);
    limit_default.mutable_maxvel()->set_rz(6);
    limit_default.mutable_maxvel()->set_yforward(2.5);
    limit_default.mutable_maxvel()->set_ybackward(1.6);
    limit_default.mutable_maxacc()->set_x(2);
    limit_default.mutable_maxacc()->set_rz(2);
    limit_default.mutable_maxacc()->set_yforward(2);
    limit_default.mutable_maxacc()->set_ybackward(2);
    limit_default.mutable_maxdec()->set_x(2);
    limit_default.mutable_maxdec()->set_y(2);
    limit_default.mutable_maxdec()->set_rz(5);
    params.mutable_limits()->Add()->CopyFrom(limit_default);
    RobotsportsVelocityControl::Limits limit_withBall = RobotsportsVelocityControl::Limits();
    limit_withBall.set_name("withBall");
    limit_withBall.mutable_maxvel()->set_x(0.5);
    limit_withBall.mutable_maxvel()->set_rz(2.5);
    limit_withBall.mutable_maxvel()->set_yforward(1.4);
    limit_withBall.mutable_maxvel()->set_ybackward(0.5);
    limit_withBall.mutable_maxacc()->set_x(0.5);
    limit_withBall.mutable_maxacc()->set_rz(3);
    limit_withBall.mutable_maxacc()->set_yforward(1);
    limit_withBall.mutable_maxacc()->set_ybackward(0.5);
    params.mutable_limits()->Add()->CopyFrom(limit_withBall);

    auto state = RobotsportsVelocityControl::State();
    state.mutable_positionsetpointfcs()->set_x(-2.6908319014261544);
    state.mutable_positionsetpointfcs()->set_y(0.57230806612855667);
    state.mutable_velocitysetpointfcs()->set_y(0.05000000074505806);
    auto diagnostics = RobotsportsVelocityControl::Diagnostics();

    // std::cout << "input: " << MRA::convert_proto_to_json_str(input) << std::endl << std::flush;
    // std::cout << "params : " << MRA::convert_proto_to_json_str(params) << std::endl << std::flush;
    // std::cout << "state: in" << MRA::convert_proto_to_json_str(state) << std::endl << std::flush;

    // Act
    int error_value = m.tick(input, params, state, output, diagnostics);

    // std::cout << "state: out: " << MRA::convert_proto_to_json_str(params) << std::endl << std::flush;
    // std::cout << "diagnostics: out: " << MRA::convert_proto_to_json_str(diagnostics) << std::endl << std::flush;
    std::cout << "output: " << MRA::convert_proto_to_json_str(output) << std::endl << std::flush;

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::RobotsportsVelocityControl::VEL_ONLY);
    EXPECT_EQ(output.velocity().x(), 0.0);
    EXPECT_NEAR(output.velocity().y(), 0.1000, 1e-6);
    EXPECT_EQ(output.velocity().rz(), 0.0);
}


TEST(RobotsportsVelocityControlTest, velocityYplusTraject) {
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(0.00);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.0);
    input.mutable_setpoint()->mutable_velocity()->set_y(+1.0);
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    auto params = m.defaultParams();
    auto state = RobotsportsVelocityControl::State();
    auto diagnostics = RobotsportsVelocityControl::Diagnostics();

    int error_value = 0;

    // Act
    for (auto sample_idx = 0; sample_idx < 3; sample_idx++) {
        std::cout << "\n\n==========================================\nsample_idx: " << sample_idx << std::endl << std::flush;
        std::cout << "input: " << MRA::convert_proto_to_json_str(input) << std::endl << std::flush;
        std::cout << "params : " << MRA::convert_proto_to_json_str(params) << std::endl << std::flush;
        std::cout << "state: in" << MRA::convert_proto_to_json_str(state) << std::endl << std::flush;

        error_value = m.tick(input, params, state, output, diagnostics);
        // auto delta_y = (params.dt()* output.velocity().y()) + input.worldstate().robot().position().y();
        input.mutable_worldstate()->mutable_robot()->mutable_position()->CopyFrom(diagnostics.newpositionrcs());
        input.mutable_worldstate()->mutable_robot()->mutable_velocity()->CopyFrom(diagnostics.newvelocityrcs());

        std::cout << "state: out: " << MRA::convert_proto_to_json_str(state) << std::endl << std::flush;
        std::cout << "diagnostics: out: " << MRA::convert_proto_to_json_str(diagnostics) << std::endl << std::flush;
        std::cout << "output: " << MRA::convert_proto_to_json_str(output) << std::endl << std::flush;
    }

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::RobotsportsVelocityControl::VEL_ONLY);
    EXPECT_EQ(output.velocity().x(), 0.0);
    EXPECT_GT(output.velocity().y(), 0.0);
    EXPECT_EQ(output.velocity().rz(), 0.0);
}


TEST(RobotsportsVelocityControlTest, trajectory1) {
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    auto params = m.defaultParams();
    auto state = RobotsportsVelocityControl::State();
    auto diagnostics = RobotsportsVelocityControl::Diagnostics();

    int error_value = 0;

    // ============================== sample_idx = 1;
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-6.5077355892075683);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-1.0006645120032562);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(-1.5707941870251312);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.0);
    input.mutable_setpoint()->mutable_velocity()->set_x(-1.524043945040972);
    input.mutable_setpoint()->mutable_velocity()->set_y(1.9817391487236535);
    input.mutable_setpoint()->mutable_velocity()->set_rz(1.2485809576986913);

    // std::cout << "input: " << MRA::convert_proto_to_json_str(input) << std::endl << std::flush;
    // std::cout << "state: in" << MRA::convert_proto_to_json_str(state) << std::endl << std::flush;
    error_value = m.tick(input, params, state, output, diagnostics);
    // std::cout << "state: out: " << MRA::convert_proto_to_json_str(state) << std::endl << std::flush;
    // std::cout << "diagnostics: out: " << MRA::convert_proto_to_json_str(diagnostics) << std::endl << std::flush;
    // std::cout << "output: " << MRA::convert_proto_to_json_str(output) << std::endl << std::flush;

    // ============================== sample_idx = 2;
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-6.5075421994773794);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-1.0006478992031749);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(-1.5707359924316022);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.0);
    input.mutable_setpoint()->mutable_velocity()->set_x(-1.5240039243791799);
    input.mutable_setpoint()->mutable_velocity()->set_y(1.9817699257171253);
    input.mutable_setpoint()->mutable_velocity()->set_rz(1.2483977210924615);
    error_value = m.tick(input, params, state, output, diagnostics);


    // ============================== sample_idx = 3;
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-6.5075421994773794);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-1.0006478992031749);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(-1.5707359924316022);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.0);
    input.mutable_setpoint()->mutable_velocity()->set_x(-1.5240039243791799);
    input.mutable_setpoint()->mutable_velocity()->set_y(1.9817699257171253);
    input.mutable_setpoint()->mutable_velocity()->set_rz(1.2483977210924615);
    error_value = m.tick(input, params, state, output, diagnostics);

    // ============================== sample_idx = 4;
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-6.5069407467181311);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-1.0033622316545352);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(-1.570536687880457);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.042467034752696718);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.040453612099819083);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.033132478594779968);
    input.mutable_setpoint()->mutable_velocity()->set_x(-1.5254186439173423);
    input.mutable_setpoint()->mutable_velocity()->set_y(1.9806811860542768);
    input.mutable_setpoint()->mutable_velocity()->set_rz(1.2482755460157777);
    error_value = m.tick(input, params, state, output, diagnostics);

    // ============================== sample_idx = 5;
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-6.50510949038625);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-1.0017138085839965);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(-1.5691365623162474);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.092026534247380387);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.087696890165874439);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.071826376020908356);
    input.mutable_setpoint()->mutable_velocity()->set_x(-1.5225086276427682);
    input.mutable_setpoint()->mutable_velocity()->set_y(1.9829189289412052);
    input.mutable_setpoint()->mutable_velocity()->set_rz(1.2450051693366471);
    error_value = m.tick(input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::RobotsportsVelocityControl::VEL_ONLY);
    EXPECT_NEAR(output.velocity().x(), -0.250000, 1e-5);
    EXPECT_NEAR(output.velocity().y(), -0.238119, 1e-5);
    EXPECT_NEAR(output.velocity().rz(),   0.194941, 1e-5);
}



int main(int argc, char **argv) {
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}
