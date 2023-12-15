// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests

#include <cstdint>

#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_factory.hpp"
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <stdexcept>
#include <random>

#include "angles.hpp"

using namespace ::testing;

// System under test:
#include "RobotsportsLocalBallTracking.hpp"
#include "logging.hpp"
using namespace MRA;
using namespace std;

// Basic tick shall run OK and return error_value 0.
TEST(RobotsportsLocalBallTrackingTest, basicTick)
{
    MRA_TRACE_TEST_FUNCTION();
    std::string testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();

    // Arrange
    auto m = RobotsportsLocalBallTracking::RobotsportsLocalBallTracking();

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

