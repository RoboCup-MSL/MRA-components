// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests


// Include testframework
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_factory.hpp"
using namespace ::testing;

#include "angles.hpp" // MRA::Geometry

// System under test:
#include "RobotsportsObstacleTracking.hpp"

// Basic tick shall run OK and return error_value 0.
TEST(RobotsportsObstacleTrackingTest, basicTick)
{
    // Arrange
    auto m = MRA::RobotsportsObstacleTracking::RobotsportsObstacleTracking();

    // Act
    int error_value = m.tick();

    // Assert
    EXPECT_EQ(error_value, 0);
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

