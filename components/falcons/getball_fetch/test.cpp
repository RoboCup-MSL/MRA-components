// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests


// Include testframework
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_factory.hpp"
using namespace ::testing;

// System under test:
#include "FalconsGetballFetch.hpp"
using namespace MRA;



// Basic tick shall run OK and return error_value 0.
TEST(FalconsGetballFetchTest, basicTick)
{
    // Arrange
    auto m = FalconsGetballFetch::FalconsGetballFetch();

    // Act
    int error_value = m.tick();

    // Assert
    EXPECT_EQ(error_value, 0);
}

// When robot is inactive, the action shall fail.
TEST(FalconsGetballFetchTest, robotInactive)
{
    // Arrange
    auto m = FalconsGetballFetch::FalconsGetballFetch();
    auto input = FalconsGetballFetch::Input();
    auto output = FalconsGetballFetch::Output();
    input.mutable_worldstate()->mutable_robot()->set_active(false);

    // Act
    int error_value = m.tick(input, output);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(output.actionresult(), MRA::Datatypes::FAILED);
}

// Move towards stationary ball in positive x direction.
TEST(FalconsGetballFetchTest, getStationaryBall)
{
    // Arrange
    auto m = FalconsGetballFetch::FalconsGetballFetch();
    auto input = FalconsGetballFetch::Input();
    auto output = FalconsGetballFetch::Output();
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-2.0);
    input.mutable_worldstate()->mutable_ball()->mutable_position()->set_x(2.0);

    // Act
    int error_value = m.tick(input, output);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(output.actionresult(), MRA::Datatypes::RUNNING);
    EXPECT_EQ(output.target().position().x(), 2.0);
}

// Verify target position in case of stationary ball close to robot at left side and not in front of robot
TEST(FalconsGetballFetchTest, getStationaryBallCloseToRobotOnLeft)
{
    // Arrange
    auto m = FalconsGetballFetch::FalconsGetballFetch();
    auto input = FalconsGetballFetch::Input();
    auto output = FalconsGetballFetch::Output();
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-1.5);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-1.5);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(0.0);
    input.mutable_worldstate()->mutable_ball()->mutable_position()->set_x(-1.90);
    input.mutable_worldstate()->mutable_ball()->mutable_position()->set_y(-1.90);

    // Act
    int error_value = m.tick(input, output);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(output.actionresult(), MRA::Datatypes::RUNNING);
    // robot should rotate on current position before translating
    EXPECT_FLOAT_EQ(output.target().position().x(), -1.5);
    EXPECT_FLOAT_EQ(output.target().position().y(), -1.5);
}

// Verify target position in case of stationary ball close to robot at right side and not in front of robot
TEST(FalconsGetballFetchTest, getStationaryBallCloseToRobotOnRight)
{
    // Arrange
    auto m = FalconsGetballFetch::FalconsGetballFetch();
    auto input = FalconsGetballFetch::Input();
    auto output = FalconsGetballFetch::Output();
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-1.5);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-1.5);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(0.0);
    input.mutable_worldstate()->mutable_ball()->mutable_position()->set_x(-1.10);
    input.mutable_worldstate()->mutable_ball()->mutable_position()->set_y(-1.10);

    // Act
    int error_value = m.tick(input, output);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(output.actionresult(), MRA::Datatypes::RUNNING);
    // robot should rotate on current position before translating
    EXPECT_FLOAT_EQ(output.target().position().x(), -1.5);
    EXPECT_FLOAT_EQ(output.target().position().y(), -1.5);
}

// Verify target position in case of stationary ball close to robot at right side and in front of robot
TEST(FalconsGetballFetchTest, getStationaryBallCloseToRobotInFront)
{
    // Arrange
    auto m = FalconsGetballFetch::FalconsGetballFetch();
    auto input = FalconsGetballFetch::Input();
    auto output = FalconsGetballFetch::Output();
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(0.5);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-0.5);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(M_PI * 0.25); // face towards ball at (0,0)
    input.mutable_worldstate()->mutable_ball()->mutable_position()->set_x(0.0);
    input.mutable_worldstate()->mutable_ball()->mutable_position()->set_y(0.0);

    // Act
    int error_value = m.tick(input, output);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(output.actionresult(), MRA::Datatypes::RUNNING);
    // robot should just drive forward to ball
    EXPECT_FLOAT_EQ(output.target().position().x(), 0.0);
    EXPECT_FLOAT_EQ(output.target().position().y(), 0.0);
    EXPECT_FLOAT_EQ(output.target().position().rz(), M_PI * 0.25);
}

// Verify target position in case of stationary ball far from robot
TEST(FalconsGetballFetchTest, getStationaryBallFarFromRobot)
{
    // Arrange
    auto m = FalconsGetballFetch::FalconsGetballFetch();
    auto input = FalconsGetballFetch::Input();
    auto output = FalconsGetballFetch::Output();
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_x(-1.5);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_y(-1.5);
    input.mutable_worldstate()->mutable_robot()->mutable_position()->set_rz(0.0);
    input.mutable_worldstate()->mutable_ball()->mutable_position()->set_x(1.9);
    input.mutable_worldstate()->mutable_ball()->mutable_position()->set_y(-1.9);

    // Act
    int error_value = m.tick(input, output);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(output.actionresult(), MRA::Datatypes::RUNNING);
    // robot should move towards the ball
    EXPECT_FLOAT_EQ(output.target().position().x(), 1.9);
    EXPECT_FLOAT_EQ(output.target().position().y(), -1.9);
}


// When robot has the ball, the action PASSED.
TEST(FalconsGetballFetchTest, hasBallPassed)
{
    // Arrange
    auto m = FalconsGetballFetch::FalconsGetballFetch();
    auto input = FalconsGetballFetch::Input();
    auto output = FalconsGetballFetch::Output();
    input.mutable_worldstate()->mutable_robot()->set_active(true);
    input.mutable_worldstate()->mutable_robot()->set_hasball(true);

    // Act
    int error_value = m.tick(input, output);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(output.actionresult(), MRA::Datatypes::PASSED);
    // TODO: what do we require on target? target==current or not? prevent move. Current: target shall be empty
    std::string json_output;
    EXPECT_TRUE(google::protobuf::util::MessageToJsonString(output, &json_output).ok());
    EXPECT_EQ(json_output, "{\"actionresult\":\"PASSED\"}");
}

// Match setup, full/realistic data, kickoff-prepare.
TEST(FalconsGetballFetchTest, matchKickoff)
{
    // A test vector contains Input, Output, Params
    // The factory will run a tick with provided data and compare against expected output
    auto output = TestFactory::run_testvector<FalconsGetballFetch::FalconsGetballFetch>(std::string("components/falcons/getball_fetch/testdata/kickoff_prepare.json"));

    EXPECT_EQ(output.actionresult(), MRA::Datatypes::RUNNING);
}

int main(int argc, char **argv)
{
	InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

