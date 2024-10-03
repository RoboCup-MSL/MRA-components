// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests


// Include testframework
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_factory.hpp"
using namespace ::testing;

// System under test:
#include "RobotsportsRobotStrategy.hpp"
using namespace MRA;

// Basic tick shall run OK and return error_value 0.
TEST(RobotsportsRobotStrategyTest, basicTick)
{
    // Arrange
    auto m = RobotsportsRobotStrategy::RobotsportsRobotStrategy();

    // Act
    int error_value = m.tick();

    // Assert
    EXPECT_EQ(error_value, 0);
}

// Basic tick shall run OK and return error_value 0.
TEST(RobotsportsRobotStrategyTest, normal_with_ball_free)
{
    // Arrange
    auto m = RobotsportsRobotStrategy::RobotsportsRobotStrategy();
    auto input = RobotsportsRobotStrategy::Input();
    auto output = RobotsportsRobotStrategy::Output();
    auto parameters = m.defaultParams();

    input.set_game_state(::MRA::RobotsportsRobotStrategy::Input_GameState_NORMAL);
    input.set_ball_status(::MRA::RobotsportsRobotStrategy::Input_BallStatus_FREE);
    // Act
    int error_value = m.tick(input, parameters, output);

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(output.dynamic_roles_size(), 4);
    EXPECT_EQ(output.dynamic_roles(0), MRA::Datatypes::ATTACKER_MAIN);
    EXPECT_EQ(output.dynamic_roles(1), MRA::Datatypes::ATTACKER_GENERIC);
    EXPECT_EQ(output.dynamic_roles(2), MRA::Datatypes::DEFENDER_MAIN);
    EXPECT_EQ(output.dynamic_roles(3), MRA::Datatypes::ATTACKER_GENERIC);
}

// More testing via combined tests of role-assigner

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

