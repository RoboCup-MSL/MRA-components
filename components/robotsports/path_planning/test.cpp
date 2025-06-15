// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests


// Include testframework
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_factory.hpp"
using namespace ::testing;

// System under test:
#include "RobotsportsPathPlanning.hpp"
#include "internal/include/PathPlanning.hpp"

using namespace MRA;

// Basic tick shall run OK and return error_value 0.
TEST(RobotsportsPathPlanningTest, basicTick)
{
    // Arrange
    auto m = RobotsportsPathPlanning::RobotsportsPathPlanning();

    // Act
    int error_value = m.tick();

    // Assert
    EXPECT_EQ(error_value, 0);
}

TEST(RobotsportsPathPlanningTest, nativeBasic)
{
    double ts = 0.0;
    path_planner_input_t input;
    path_planner_parameters_t params;
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    auto path_planning = PathPlanning();

    // // raw calculation based on inputs
    path_planning.calculate(ts, input, params, state, output, diagnostics);

}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

