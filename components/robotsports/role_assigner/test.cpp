// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests


// Include testframework
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_factory.hpp"
using namespace ::testing;
#include "xmlTeamPlanner.h"

// System under test:
#include "RobotsportsRoleAssigner.hpp"
using namespace MRA;

// Basic tick shall run OK and return error_value 0.
TEST(RobotsportsRoleAssignerTest, basicTick)
{
    // Arrange
    auto m = RobotsportsRoleAssigner::RobotsportsRoleAssigner();

    // Act
    int error_value = m.tick();

    // Assert
    EXPECT_EQ(error_value, 0);
}

// Basic tick shall run OK and return error_value 0.
TEST(RobotsportsRoleAssignerTest, xmlTest)
{
    unsigned runs_needed = 1;
    auto input_filename ="/home/jurge/MRA-components/components/robotsports/role_assigner/testdata/normal_defend_4.xml";
    xmlplanner(input_filename, runs_needed);
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

