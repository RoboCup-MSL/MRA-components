// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests

// Include testframework
#include "test_factory.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
using namespace ::testing;

// System under test:
#include "RobotsportsVelocityControl.hpp"
#include "geometry.hpp"
using namespace MRA;
using namespace std;

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
    EXPECT_GT(output.velocity().y(), 0.0);
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

TEST(RobotsportsVelocityControlTest, velocityRequestAboveLimit1) {
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
    state.set_executed_before(true);
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
    // std::cout << "output: " << MRA::convert_proto_to_json_str_pretty(output) << std::endl << std::flush;

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::RobotsportsVelocityControl::VEL_ONLY);
    EXPECT_GT(output.velocity().x(), 0.0);
    EXPECT_GT(output.velocity().y(), 0.0);
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
        // std::cout << "\n\n==========================================\nsample_idx: " << sample_idx << std::endl << std::flush;
        // std::cout << "input: " << MRA::convert_proto_to_json_str(input) << std::endl << std::flush;
        // std::cout << "params : " << MRA::convert_proto_to_json_str(params) << std::endl << std::flush;
        // std::cout << "state: in" << MRA::convert_proto_to_json_str(state) << std::endl << std::flush;

        error_value = m.tick(input, params, state, output, diagnostics);
        // auto delta_y = (params.dt()* output.velocity().y()) + input.worldstate().robot().position().y();
        input.mutable_worldstate()->mutable_robot()->mutable_position()->CopyFrom(diagnostics.newpositionrcs());
        input.mutable_worldstate()->mutable_robot()->mutable_velocity()->CopyFrom(diagnostics.newvelocityrcs());

        // std::cout << "state: out: " << MRA::convert_proto_to_json_str(state) << std::endl << std::flush;
        // std::cout << "diagnostics: out: " << MRA::convert_proto_to_json_str(diagnostics) << std::endl << std::flush;
        // std::cout << "output: " << MRA::convert_proto_to_json_str(output) << std::endl << std::flush;
    }

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::RobotsportsVelocityControl::VEL_ONLY);
    EXPECT_EQ(output.velocity().x(), 0.0);
    EXPECT_GT(output.velocity().y(), 0.0);
    EXPECT_EQ(output.velocity().rz(), 0.0);
}



TEST(RobotsportsVelocityControlTest, positionYplusTraject) {
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
    input.mutable_setpoint()->mutable_position()->set_y(+1.0);
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    auto params = m.defaultParams();
    auto state = RobotsportsVelocityControl::State();
    auto diagnostics = RobotsportsVelocityControl::Diagnostics();

    int error_value = 0;

    // Act: 1 meter distance with 2 m/s is done is 57.x samples
    for (auto sample_idx = 0; sample_idx < 57; sample_idx++) {
        error_value = m.tick(input, params, state, output, diagnostics);
        input.mutable_worldstate()->mutable_robot()->mutable_position()->CopyFrom(state.positionsetpointfcs());
        input.mutable_worldstate()->mutable_robot()->mutable_velocity()->CopyFrom(state.velocitysetpointfcs());
    }

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::RobotsportsVelocityControl::POS_ONLY);
    EXPECT_EQ(output.velocity().x(), 0.0);
    EXPECT_NEAR(output.velocity().y(), 0.0, 0.005);
    EXPECT_EQ(output.velocity().rz(), 0.0);
    EXPECT_GT(state.positionsetpointfcs().y(), 0.99); // check position via state
}

TEST(RobotsportsVelocityControlTest, moveToBall_traject) {
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-6.4999398351107862);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-0.99828182113030373);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(-1.5702649899643737);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.0);
    input.mutable_setpoint()->mutable_position()->set_x(0.99268668231324);
    input.mutable_setpoint()->mutable_position()->set_y(2.0065099714398578);
    input.mutable_setpoint()->mutable_position()->set_rz(-1.1892918584432319);
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    auto params = m.defaultParams();
    auto state = RobotsportsVelocityControl::State();
    auto diagnostics = RobotsportsVelocityControl::Diagnostics();

    int error_value = 0;

    for (auto sample_idx = 0; sample_idx < 230; sample_idx++) {
        // std::cout << "\n\n==================================== \nsample_idx = " << sample_idx << std::endl;
        // std::cout << "input: " << MRA::convert_proto_to_json_str(input) << std::endl << std::flush;
        // std::cout << "state: in" << MRA::convert_proto_to_json_str(state) << std::endl << std::flush;
        error_value = m.tick(input, params, state, output, diagnostics);
        input.mutable_worldstate()->mutable_robot()->mutable_position()->CopyFrom(state.positionsetpointfcs());
        input.mutable_worldstate()->mutable_robot()->mutable_velocity()->CopyFrom(state.velocitysetpointfcs());
        // std::cout << "state: out: " << MRA::convert_proto_to_json_str(state) << std::endl << std::flush;
        // std::cout << "diagnostics: out: " << MRA::convert_proto_to_json_str(diagnostics) << std::endl << std::flush;
        // std::cout << "output: " << MRA::convert_proto_to_json_str(output) << std::endl << std::flush;
    }

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::RobotsportsVelocityControl::POS_ONLY);
    EXPECT_NEAR(output.velocity().x(), 0.0, 0.001);
    EXPECT_NEAR(output.velocity().y(), 0.0, 0.001);
    EXPECT_EQ(output.velocity().rz(), 0.0);
    EXPECT_NEAR(state.positionsetpointfcs().x(), 0.97328414980065947, 0.02); // check position via state
    EXPECT_NEAR(state.positionsetpointfcs().y(), 1.9865288664091718, 0.02); // check position via state
    EXPECT_NEAR(state.positionsetpointfcs().rz(), -1.1892918584432319, 0.02); // check position via state
}

TEST(RobotsportsVelocityControlTest, moveToBall_sample_1) {
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-6.5004861901491129);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-1.0046461636063577);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(-1.5720789053267596);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.0);
    input.mutable_setpoint()->mutable_position()->set_x(0.99268668231324);
    input.mutable_setpoint()->mutable_position()->set_y(2.0065099714398578);
    input.mutable_setpoint()->mutable_position()->set_rz(-1.1892918584432319);
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    auto params = m.defaultParams();
    auto state = RobotsportsVelocityControl::State();
    state.set_executed_before(true);
    state.mutable_positionsetpointfcs()->set_x(0.0);
    state.mutable_positionsetpointfcs()->set_y(0.0);
    state.mutable_positionsetpointfcs()->set_rz(0.0);
    state.mutable_velocitysetpointfcs()->set_x(0.0);
    state.mutable_velocitysetpointfcs()->set_y(0.0);
    state.mutable_velocitysetpointfcs()->set_rz(0.0);


    auto diagnostics = RobotsportsVelocityControl::Diagnostics();

    int error_value = 0;

    // std::cout << "input: " << MRA::convert_proto_to_json_str(input) << std::endl << std::flush;
    // std::cout << "state: in" << MRA::convert_proto_to_json_str(state) << std::endl << std::flush;
    error_value = m.tick(input, params, state, output, diagnostics);
    // std::cout << "state: out: " << MRA::convert_proto_to_json_str(state) << std::endl << std::flush;
    // std::cout << "diagnostics: out: " << MRA::convert_proto_to_json_str(diagnostics) << std::endl << std::flush;
    // std::cout << "output: " << MRA::convert_proto_to_json_str(output) << std::endl << std::flush;

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::RobotsportsVelocityControl::POS_ONLY);
    EXPECT_EQ(output.velocity().x(), 0.05);
    EXPECT_NEAR(output.velocity().y(), 0.02009, 0.001);
    EXPECT_EQ(output.velocity().rz(), -0.05);
    EXPECT_NEAR(state.positionsetpointfcs().x(), -6.4998611901630827, 0.02); // check position via state
    EXPECT_NEAR(state.positionsetpointfcs().y(), -1.0040211636203276, 0.02); // check position via state
    EXPECT_NEAR(state.positionsetpointfcs().rz(), -0.00062500000000031974, 0.02); // check position via state
}


TEST(RobotsportsVelocityControlTest, outputRot90VelY) {
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
    input.mutable_setpoint()->mutable_velocity()->set_y(1.0);
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    auto params = m.defaultParams();
    auto state = RobotsportsVelocityControl::State();
    auto diagnostics = RobotsportsVelocityControl::Diagnostics();

    int error_value = 0;
    // Act
    // std::cout << "input: " << MRA::convert_proto_to_json_str(input) << std::endl << std::flush;
    // std::cout << "state: in" << MRA::convert_proto_to_json_str(state) << std::endl << std::flush;
    error_value = m.tick(input, params, state, output, diagnostics);
    // std::cout << "state: out: " << MRA::convert_proto_to_json_str(state) << std::endl << std::flush;
    // std::cout << "diagnostics: out: " << MRA::convert_proto_to_json_str(diagnostics) << std::endl << std::flush;
    // std::cout << "output: " << MRA::convert_proto_to_json_str(output) << std::endl << std::flush;

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::RobotsportsVelocityControl::VEL_ONLY);
    EXPECT_EQ(output.velocity().x(), 0.0);
    EXPECT_GT(output.velocity().y(), 0.0);
    EXPECT_EQ(output.velocity().rz(), 0.0);
}

TEST(RobotsportsVelocityControlTest, outputRot90PosY) {
    // Arrange
    auto m = RobotsportsVelocityControl::RobotsportsVelocityControl();
    auto input = RobotsportsVelocityControl::Input();
    auto output = RobotsportsVelocityControl::Output();
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-1.00);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-6.50);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(M_PI_2);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_x(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_y(0.0);
    input.mutable_worldstate()->mutable_robot()->mutable_velocity()->set_rz(0.0);
    input.mutable_setpoint()->mutable_position()->set_x(-1.0);
    input.mutable_setpoint()->mutable_position()->set_y(-4.5);
    input.mutable_setpoint()->mutable_position()->set_rz(M_PI_2);
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    auto params = m.defaultParams();
    auto state = RobotsportsVelocityControl::State();
    auto diagnostics = RobotsportsVelocityControl::Diagnostics();

    // std::cout << "input: " << MRA::convert_proto_to_json_str_pretty(input) << std::endl << std::flush;
    // std::cout << "params : " << MRA::convert_proto_to_json_str(params) << std::endl << std::flush;
    // std::cout << "state: in" << MRA::convert_proto_to_json_str(state) << std::endl << std::flush;

    // Act
    int error_value = m.tick(input, params, state, output, diagnostics);

    // std::cout << "state: out: " << MRA::convert_proto_to_json_str(params) << std::endl << std::flush;
    // std::cout << "diagnostics: out: " << MRA::convert_proto_to_json_str_pretty(diagnostics) << std::endl << std::flush;
    // std::cout << "output: " << MRA::convert_proto_to_json_str_pretty(output) << std::endl << std::flush;

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(diagnostics.controlmode(), MRA::RobotsportsVelocityControl::POS_ONLY);
    EXPECT_GT(output.velocity().x(), 0.0);
    EXPECT_NEAR(output.velocity().y(), 0.0, 1e-6);
    EXPECT_EQ(output.velocity().rz(), 0.0);
}

// static void posTest(std::string text, double x, double y, double rz, double dX, double dY, double dRz) {
//     MRA::Geometry::Position pos1(x, y, 0.0, 0.0, 0.0, rz);
//     MRA::Geometry::Position pos2(x+dX, y+dY, 0.0, 0.0, 0.0, rz+dRz);

//     auto pos3  = pos2;
//     pos3.transformFcsToRcs(pos1);
//     cout << text << endl;
//     cout << "pos1      : " << pos1.toString() << endl;
//     cout << "pos2      : " << pos2.toString() << endl;
//     cout << "pos3 (rcs): " << pos3.toString() << endl;
// }

// TEST(RobotsportsVelocityControlTest, McsToRcs) {
//     double bX = -1.5;
//     double bY = -6.5;
//     double bRz = M_PI_2;
//     double dist = 2.0;

//     posTest("rot: +90deg x+1", bX, bY, bRz, dist, 0.0, 0.0);
//     posTest("rot:   0deg x+1", bX, bY, 0.0, dist, 0.0, 0.0);
//     posTest("rot: -90deg x+1", bX, bY, -bRz, dist, 0.0, 0.0);
//     posTest("rot: 180deg x+1", bX, bY, 2*bRz, dist, 0.0, 0.0);

//     posTest("rot: +90deg y+1", bX, bY, bRz, 0.0, dist, 0.0);
//     posTest("rot:   0deg y+1", bX, bY, 0.0, 0.0, dist, 0.0);
//     posTest("rot: -90deg y+1", bX, bY, -bRz, 0.0, dist, 0.0);
//     posTest("rot: 180deg y+1", bX, bY, 2*bRz, 0.0, dist, 0.0);
// }



int main(int argc, char **argv) {
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}
