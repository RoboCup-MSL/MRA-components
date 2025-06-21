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

const double PATH_PLANNING_NUMERICAL_TOLERANCE = 1e-5;


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

TEST(RobotsportsPathPlanningTest, native_basic)
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

TEST(RobotsportsPathPlanningTest, native_inactive_shouldFail)
{
    double ts = 0.0;
    path_planner_input_t input;
    input.myRobotState.active = false;
    path_planner_parameters_t params;
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    auto path_planning = PathPlanning();

    // // raw calculation based on inputs
    path_planning.calculate(ts, input, params, state, output, diagnostics);
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::FAILED);
    EXPECT_EQ(output.positionSetpointValid, false);
    EXPECT_EQ(output.velocitySetpointValid, true);
    EXPECT_EQ(output.robotVelocitySetpoint.is_zero(1e-6), true);
}

 path_planner_input_t getDefaultInput() {
    path_planner_input_t input  = {};
    input.myRobotState.active = true;
    input.myRobotState.position = {};
    input.myRobotState.velocity = {};
    input.myRobotState.hasBall = false;
    input.motionSetpoint.positionValid = true;
    input.motionSetpoint.position = {};
    input.motionSetpoint.velocityValid = false;
    input.motionSetpoint.velocity = {};
    input.motionSetpoint.move_action = true;

    return input;
 }

path_planner_parameters_t getDefaultParameters() {
    path_planner_parameters_t params = {};

    params.numExtraSettlingTicks = 0;
    params.obstacleAvoidance.enabled = true;
    params.obstacleAvoidance.robotRadius = 0.26;
    params.obstacleAvoidance.obstacleRadius = 0.26;
    params.obstacleAvoidance.distanceScalingFactor = 0.0;
    params.obstacleAvoidance.speedScalingFactor = 1.0;
    params.obstacleAvoidance.speedLowerThreshold = 0.2;
    params.obstacleAvoidance.speedUpperThreshold = 4.0;
    params.obstacleAvoidance.generatedObstacleSpacing = 0.5;
    params.obstacleAvoidance.ballClearance = 1.0;
    params.obstacleAvoidance.groupGapDistance = 0.622;
    params.obstacleAvoidance.subTargetDistance = 0.571;
    params.obstacleAvoidance.subTargetExtensionFactor = 0.0;
    params.forwardDriving.withoutBall.enabled = false;
    params.forwardDriving.withoutBall.minimumDistance = 3.0;
    params.forwardDriving.withBall.enabled = true;
    params.forwardDriving.withBall.minimumDistance = 2.0;

    params.boundaries.targetInsideForbiddenArea = BoundaryOptionEnum::STOP_AND_FAIL;
    params.boundaries.targetOutsideField = BoundaryOptionEnum::CLIP;
    params.boundaries.fieldMarginX = 0.0;
    params.boundaries.fieldMarginY = 0.0;
    params.boundaries.targetOnOwnHalf = BoundaryOptionEnum::ALLOW;
    params.boundaries.targetOnOpponentHalf = BoundaryOptionEnum::ALLOW;

    // params.deadzone;
    // params.tokyoDrift;
    
    params.slowFactor = 0.5;
    params.frequency = 40.0; // [Hz]     The frequency of the heartbeat (tick), e.g., 40Hz -> 40 ticks per second.
    
    return params;
};

TEST(RobotsportsPathPlanningTest, native_onTarget_shouldPass)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::PASSED);
    EXPECT_EQ(diagnostics.stop, false);
    EXPECT_EQ(output.robotPositionSetpoint.x, input.myRobotState.position.x);
    EXPECT_EQ(output.robotPositionSetpoint.y, input.myRobotState.position.y);
    EXPECT_EQ(output.robotPositionSetpoint.rz, input.myRobotState.position.rz);
    EXPECT_NEAR(output.robotVelocitySetpoint.x, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
    EXPECT_NEAR(output.robotVelocitySetpoint.y, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
    EXPECT_NEAR(output.robotVelocitySetpoint.rz, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
}

TEST(RobotsportsPathPlanningTest, native_simpleMove_shouldMoveForward)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.y = 1.0;

    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
    EXPECT_EQ(diagnostics.stop, false);

    EXPECT_NEAR(output.robotPositionSetpoint.x,  input.motionSetpoint.position.x, PATH_PLANNING_NUMERICAL_TOLERANCE);
    EXPECT_NEAR(output.robotPositionSetpoint.y,  input.motionSetpoint.position.y, PATH_PLANNING_NUMERICAL_TOLERANCE);
    EXPECT_NEAR(output.robotPositionSetpoint.rz, input.motionSetpoint.position.rz, PATH_PLANNING_NUMERICAL_TOLERANCE);
}

TEST(RobotsportsPathPlanningTest, native_stop)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.x = 1.0;
    input.motionSetpoint.move_action = false; // force stop inside

    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::PASSED);
    EXPECT_EQ(diagnostics.stop, true);
}

TEST(RobotsportsPathPlanningTest, native_closebyXYnotRz_shouldContinue)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.rz = 1.0;

    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
}

TEST(RobotsportsPathPlanningTest, native_closebyRznotXY_shouldContinue)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.y = 1.0;

    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
}

// boundary limiters
TEST(RobotsportsPathPlanningTest, native_targetOutOfBounds_shouldFail)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    params.boundaries.targetOutsideField = BoundaryOptionEnum::STOP_AND_FAIL;
    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.x = 99.0;
    input.motionSetpoint.position.y = 99.0;

    // Act
    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::FAILED);
    EXPECT_EQ(diagnostics.stop, true);
}

TEST(RobotsportsPathPlanningTest, native_targetX_OutOfBounds_shouldClip)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    params.boundaries.targetOutsideField = BoundaryOptionEnum::CLIP;
    params.boundaries.fieldMarginX = 0.9; // TODO stub environmentField
    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.x = 99.0;
    
    // Act
    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
    EXPECT_NEAR(output.robotPositionSetpoint.x,  6.9, PATH_PLANNING_NUMERICAL_TOLERANCE);
    EXPECT_NEAR(output.robotPositionSetpoint.y,  0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
    EXPECT_NEAR(output.robotPositionSetpoint.rz, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
}

TEST(RobotsportsPathPlanningTest, native_targetY_OutOfBounds_shouldClip)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;
    
    params.boundaries.targetOutsideField = BoundaryOptionEnum::CLIP;
    params.boundaries.fieldMarginY = 0.4; // TODO stub environmentField
    
    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.y = 99.0;

    // Act
    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);


    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
    EXPECT_NEAR(output.robotPositionSetpoint.x, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
    EXPECT_NEAR(output.robotPositionSetpoint.y, 9.4, PATH_PLANNING_NUMERICAL_TOLERANCE);
    EXPECT_NEAR(output.robotPositionSetpoint.rz, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
}

TEST(RobotsportsPathPlanningTest, native_targetY_ownHalf_notAllowed)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;
    params.boundaries.targetOnOwnHalf = BoundaryOptionEnum::STOP_AND_FAIL;
    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.y = -6.0;

    // Act
    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::FAILED);
    EXPECT_EQ(diagnostics.stop, true);
}

TEST(RobotsportsPathPlanningTest, native_targetY_ownHalf_clip)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    params.boundaries.targetOnOwnHalf = BoundaryOptionEnum::CLIP;
    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.x = 1.0;
    input.motionSetpoint.position.y = -6.0;

    // Act
    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
    EXPECT_NEAR(output.robotPositionSetpoint.x, 1.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
    EXPECT_NEAR(output.robotPositionSetpoint.y, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
    EXPECT_NEAR(output.robotPositionSetpoint.rz, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
}


TEST(RobotsportsPathPlanningTest, native_targetY_oppHalf_notAllowed)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;
    
    params.boundaries.targetOnOpponentHalf = BoundaryOptionEnum::STOP_AND_FAIL;
    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.y = 6.0;

    // Act
    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::FAILED);
    EXPECT_EQ(diagnostics.stop, true);
}

TEST(RobotsportsPathPlanningTest, native_targetY_oppHalf_clip)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    params.boundaries.targetOnOpponentHalf = BoundaryOptionEnum::CLIP;
    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.x = 1.0;
    input.motionSetpoint.position.y = 6.0;

    // Act
    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
    EXPECT_NEAR(output.robotPositionSetpoint.x, 1.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
    EXPECT_NEAR(output.robotPositionSetpoint.y, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
    EXPECT_NEAR(output.robotPositionSetpoint.rz, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
}

// forbidden areas

TEST(RobotsportsPathPlanningTest, native_targetInForbiddenArea_shouldFail)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.y = 6.0;

    forbiddenArea_t f; // construct a forbidden area around the target (penalty marker)
    f.points.push_back(MRA::Geometry::Point(-1.0,  5.0));
    f.points.push_back(MRA::Geometry::Point(-1.0,  7.0));
    f.points.push_back(MRA::Geometry::Point( 1.0,  7.0));
    f.points.push_back(MRA::Geometry::Point( 1.0,  5.0));
    input.forbiddenAreas.push_back(f);

    // Act
    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::FAILED);
    EXPECT_EQ(diagnostics.stop, true);
}

TEST(RobotsportsPathPlanningTest, native_currentInForbiddenArea_shouldMoveOut)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    input.myRobotState.position = MRA::Geometry::Pose(0.0, 6.0);
    forbiddenArea_t f; // construct a forbidden area around the target (penalty marker)
    f.points.push_back(MRA::Geometry::Point(-1.0,  5.0));
    f.points.push_back(MRA::Geometry::Point(-1.0,  7.0));
    f.points.push_back(MRA::Geometry::Point( 1.0,  7.0));
    f.points.push_back(MRA::Geometry::Point( 1.0,  5.0));
    input.forbiddenAreas.push_back(f);

    // Act
    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
    EXPECT_NEAR(output.robotPositionSetpoint.x, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
    EXPECT_NEAR(output.robotPositionSetpoint.y, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
    EXPECT_NEAR(output.robotPositionSetpoint.rz, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
}

// obstacle avoidance

TEST(RobotsportsPathPlanningTest, native_avoid_teammember_left)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.x = 4.0;
    input.motionSetpoint.position.y = 0.1;
    robotState_t r;
    r.position = MRA::Geometry::Pose(2.0, 0.0);
    input.teamRobotState.push_back(r);

    // Act
    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
    EXPECT_NEAR(output.robotPositionSetpoint.x, 2.0, 0.5);
    EXPECT_NEAR(output.robotPositionSetpoint.y, params.obstacleAvoidance.subTargetDistance, PATH_PLANNING_NUMERICAL_TOLERANCE);
    EXPECT_NEAR(output.robotPositionSetpoint.rz, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
}

TEST(RobotsportsPathPlanningTest, native_avoid_teammember_right)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.x = 4.0;
    input.motionSetpoint.position.y = -0.1;
    robotState_t r;
    r.position = MRA::Geometry::Pose(2.0, 0.0, 0.0);
    input.teamRobotState.push_back(r);

    // Act
    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
    EXPECT_NEAR(output.robotPositionSetpoint.x, 2.0, 0.5);
    EXPECT_NEAR(output.robotPositionSetpoint.y, -params.obstacleAvoidance.subTargetDistance, PATH_PLANNING_NUMERICAL_TOLERANCE);
    EXPECT_NEAR(output.robotPositionSetpoint.rz, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
}

TEST(RobotsportsPathPlanningTest, native_avoid_moving_teammember)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.x = 4.0;
    input.motionSetpoint.position.y = -0.3;
    // WAS, but this is robot, not a moving teammate
    // input.motionSetpoint.velocity = MRA::Geometry::Pose();
    // input.motionSetpoint.velocity.y = 2.0;

    robotState_t r;
    r.position = MRA::Geometry::Pose(2.0, 0.0, 0.0);
    r.velocity = MRA::Geometry::Pose(0.0, 2.0, 0.0);
    input.teamRobotState.push_back(r);

    // Act
    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
    EXPECT_NEAR(output.robotPositionSetpoint.x, 2.0, 0.5);
    EXPECT_NEAR(output.robotPositionSetpoint.y, -params.obstacleAvoidance.subTargetDistance, PATH_PLANNING_NUMERICAL_TOLERANCE);
    EXPECT_NEAR(output.robotPositionSetpoint.rz, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
}

TEST(RobotsportsPathPlanningTest, native_avoid_obstacle)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.x = 4.0;

    obstacleResult_t obst;
    obst.position = MRA::Geometry::Point(2.0, -0.1);
    input.obstacles.push_back(obst);

    // Act
    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
    EXPECT_NEAR(output.robotPositionSetpoint.x, 2.0, 0.5);
    EXPECT_NEAR(output.robotPositionSetpoint.y, obst.position.y + params.obstacleAvoidance.subTargetDistance, 0.1);
    EXPECT_NEAR(output.robotPositionSetpoint.rz, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
}

TEST(RobotsportsPathPlanningTest, native_avoid_obstacle_cluster_left)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.x = 4.0;
    
    obstacleResult_t obst;
    obst.position = MRA::Geometry::Point(1.5, -0.5); input.obstacles.push_back(obst);
    obst.position = MRA::Geometry::Point(1.5, -0.1); input.obstacles.push_back(obst);
    obst.position = MRA::Geometry::Point(1.5,  0.3); input.obstacles.push_back(obst);
    obst.position = MRA::Geometry::Point(2.0, -0.5); input.obstacles.push_back(obst);
    obst.position = MRA::Geometry::Point(2.0, -0.1); input.obstacles.push_back(obst);
    obst.position = MRA::Geometry::Point(2.0,  0.3); input.obstacles.push_back(obst);
    obst.position = MRA::Geometry::Point(2.5, -0.5); input.obstacles.push_back(obst);
    obst.position = MRA::Geometry::Point(2.5, -0.1); input.obstacles.push_back(obst);
    obst.position = MRA::Geometry::Point(2.5,  0.3); input.obstacles.push_back(obst);

    // Act
    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
    EXPECT_NEAR(output.robotPositionSetpoint.x, 1.5, 0.5);
    EXPECT_NEAR(output.robotPositionSetpoint.y, 1.0, 0.5);
    EXPECT_NEAR(output.robotPositionSetpoint.rz, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
}

TEST(RobotsportsPathPlanningTest, native_avoid_obstacle_cluster_right)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.x = 4.0;
    input.motionSetpoint.position.y = -1.0;

    obstacleResult_t obst;
    obst.position = MRA::Geometry::Point(1.5, -0.5); input.obstacles.push_back(obst);
    obst.position = MRA::Geometry::Point(1.5, -0.1); input.obstacles.push_back(obst);
    obst.position = MRA::Geometry::Point(1.5,  0.3); input.obstacles.push_back(obst);
    obst.position = MRA::Geometry::Point(2.0, -0.5); input.obstacles.push_back(obst);
    obst.position = MRA::Geometry::Point(2.0, -0.1); input.obstacles.push_back(obst);
    obst.position = MRA::Geometry::Point(2.0,  0.3); input.obstacles.push_back(obst);
    obst.position = MRA::Geometry::Point(2.5, -0.5); input.obstacles.push_back(obst);
    obst.position = MRA::Geometry::Point(2.5, -0.1); input.obstacles.push_back(obst);
    obst.position = MRA::Geometry::Point(2.5,  0.3); input.obstacles.push_back(obst);

    // Act
    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
    EXPECT_NEAR(output.robotPositionSetpoint.x, 1.5, 0.5);
    EXPECT_NEAR(output.robotPositionSetpoint.y, -1.0, 0.5);
    EXPECT_NEAR(output.robotPositionSetpoint.rz, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
}

/*
TEST(RobotsportsPathPlanningTest, native_avoid_obstacles_jailed) // TODO: disabled for now; undefined behavior.. robot should perhaps just stop?
{
    // Arrange
    auto pp = defaultPathPlanningSetup();
    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.x = 4.0;
    obstacleResult obst;
    obst.position = MRA::Geometry::Point(-0.4, -0.4); input.obbstacles.push_back(obst);
    obst.position = MRA::Geometry::Point(-0.4,  0.4); input.obbstacles.push_back(obst);
    obst.position = MRA::Geometry::Point( 0.4,  0.4); input.obbstacles.push_back(obst);
    obst.position = MRA::Geometry::Point( 0.4, -0.4); input.obbstacles.push_back(obst);

    // Iteration 1: robot exactly at center, move towards target
    pp.data.robot.position = MRA::Geometry::Pose(0.0, 0.0, 0.0);
    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
    auto subtarget = pp.data.path[0];
    EXPECT_NEAR(output.robotPositionSetpoint.x, 0.4, 0.1);
    EXPECT_NEAR(output.robotPositionSetpoint.y, 0.4, 0.1); // ! robot actually moves towards one of the obstacles ... TODO?
    EXPECT_NEAR(output.robotPositionSetpoint.rz, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);

    // Iteration 2: robot on its way
    pp.data.robot.position = MRA::Geometry::Pose(0.2, 0.0, 0.0);
    result = pp.calculate();
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
    subtarget = pp.data.path[0];
    EXPECT_NEAR(output.robotPositionSetpoint.x, 0.6, 0.2);
    EXPECT_NEAR(output.robotPositionSetpoint.y, 0.0, 0.2);
    EXPECT_NEAR(output.robotPositionSetpoint.rz, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
}
*/

// // Tests to check yaml configurations: can robot find a way between two perfect still obstacles
// // as is required for TCrun2 (where noise comes into play as well)
// // First parameter: yaml file
// // Second parameter: distance between obstacles
// // Third parameter: y offset of obstacles
// class ObstacleDriveThroughTest : public testing::TestWithParam<std::tuple<std::string, double, double>>
// {
// public:
//     virtual void SetUp(){}
//     virtual void TearDown(){}
// };

// INSTANTIATE_TEST_CASE_P(pathPlanningTest,
//                         ObstacleDriveThroughTest,
//                         ::testing::Values(
//                             std::make_tuple("PathPlanningSim.yaml", 1.8,  0.0), // perfect placement, no noise
//                             std::make_tuple("PathPlanningSim.yaml", 1.4,  0.0), // worst case placement, no noise
//                             std::make_tuple("PathPlanningSim.yaml", 1.2,  0.0), // worst case placement, allow a bit of vision/worldModel noise
//                             std::make_tuple("PathPlanning.yaml",    1.8,  0.0), // perfect placement, no noise
//                             std::make_tuple("PathPlanning.yaml",    1.4,  0.0), // worst case placement, no noise
//                             // repeat all tests, but with offsets, to stress-test angle calculations (left/right orientation)
//                             std::make_tuple("PathPlanningSim.yaml", 1.8,  0.3),
//                             std::make_tuple("PathPlanningSim.yaml", 1.4,  0.3),
//                             std::make_tuple("PathPlanningSim.yaml", 1.2,  0.3),
//                             std::make_tuple("PathPlanning.yaml",    1.8,  0.3),
//                             std::make_tuple("PathPlanning.yaml",    1.4,  0.3),
//                             std::make_tuple("PathPlanningSim.yaml", 1.8, -0.3),
//                             std::make_tuple("PathPlanningSim.yaml", 1.4, -0.3),
//                             std::make_tuple("PathPlanningSim.yaml", 1.2, -0.3),
//                             std::make_tuple("PathPlanning.yaml",    1.8, -0.3),
//                             std::make_tuple("PathPlanning.yaml",    1.4, -0.3)
//                             )
//                         );

// // NOTE: the 1.2m distance tests using PathPlanning.yaml have been removed as they were sensitive to tuning

// TEST_P(ObstacleDriveThroughTest, ObstacleDriveThroughTests)
// {
//     const ::testing::TestInfo* const test_info = ::testing::UnitTest::GetInstance()->current_test_info();

//     // Parameters
//     std::string yamlfile = std::get<0>(GetParam());
//     double distanceBetweenObstacles = std::get<1>(GetParam());
//     double offsetY = std::get<2>(GetParam());
//     MRA_LOG_DEBUG("test parameters: yamlfile=%s distanceBetweenObstacles=%6.2f offsetY=%6.2f", yamlfile.c_str(), distanceBetweenObstacles, offsetY);

//     // Arrange
//     auto pp = yamlPathPlanningSetup(yamlfile);
//     params.obstacleAvoidance.enabled = true; // make sure it is enabled, yaml might temporarily use false
//     params.forwardDriving.withoutBall.enabled = false; // prevent extra sub-targets
//     EXPECT_GT(params.obstacleAvoidance.subTargetDistance, 0.1); // sanity check if YAML was properly loaded
//     pp.data.robot.position.y = offsetY;
    // input.motionSetpoint.position = MRA::Geometry::Pose();
    // input.motionSetpoint.position.x = 4.0;
    // input.motionSetpoint.position.y = offsetY;
//     obstacleResult obst;
//     obst.position = MRA::Geometry::Point(2.0, -0.5 * distanceBetweenObstacles); input.obbstacles.push_back(obst);
//     obst.position = MRA::Geometry::Point(2.0,  0.5 * distanceBetweenObstacles); input.obbstacles.push_back(obst);

//     // Act
    // auto path_planning = PathPlanning();
    // path_planning.calculate(ts, input, params, state, output, diagnostics);

//     // Assert
//     EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
//     //     // either no extra subtarget (= no obstacle avoidance)
//     // or a subtarget inbetween both obstacles
//     if (output.robotPositionSetpoint.x > 3.0)
//     {
//         EXPECT_NEAR(output.robotPositionSetpoint.x, 4.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
//         EXPECT_NEAR(output.robotPositionSetpoint.y, offsetY, PATH_PLANNING_NUMERICAL_TOLERANCE);
//     }
//     else
//     {
//         EXPECT_NEAR(output.robotPositionSetpoint.x, 2.0, 0.2);
//         EXPECT_NEAR(output.robotPositionSetpoint.y, 0.0, 0.25 * distanceBetweenObstacles);
//     }
// }

TEST(RobotsportsPathPlanningTest, native_avoid_forbidden_area_left)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.x = 4.0;
    input.motionSetpoint.position.y = 0.3;

    forbiddenArea_t f; // construct a forbidden area in between robot and target
    f.points.push_back(MRA::Geometry::Point(1.0, -1.0));
    f.points.push_back(MRA::Geometry::Point(1.0,  1.0));
    f.points.push_back(MRA::Geometry::Point(3.0,  1.0));
    f.points.push_back(MRA::Geometry::Point(3.0, -1.0));
    input.forbiddenAreas.push_back(f);

    // Act
    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
    EXPECT_NEAR(output.robotPositionSetpoint.x, 1.0, 0.8);
    EXPECT_NEAR(output.robotPositionSetpoint.y, 1.0, 0.8);
    EXPECT_NEAR(output.robotPositionSetpoint.rz, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
}

TEST(RobotsportsPathPlanningTest, native_avoid_forbidden_area_right)
{
    // Arrange
    double ts = 0.0;
    path_planner_input_t input  = getDefaultInput();
    path_planner_parameters_t params = getDefaultParameters();
    path_planner_state_t state;
    path_planner_output_t output;
    path_planner_diagnostics_t diagnostics;

    input.motionSetpoint.position = MRA::Geometry::Pose();
    input.motionSetpoint.position.x = 4.0;
    input.motionSetpoint.position.y = -0.3;

    forbiddenArea_t f; // construct a forbidden area in between robot and target
    f.points.push_back(MRA::Geometry::Point(1.0, -1.0));
    f.points.push_back(MRA::Geometry::Point(1.0,  1.0));
    f.points.push_back(MRA::Geometry::Point(3.0,  1.0));
    f.points.push_back(MRA::Geometry::Point(3.0, -1.0));
    input.forbiddenAreas.push_back(f);

    // Act
    auto path_planning = PathPlanning();
    path_planning.calculate(ts, input, params, state, output, diagnostics);

    // Assert
    EXPECT_EQ(output.status, MRA::Datatypes::ActionResult::RUNNING);
    EXPECT_NEAR(output.robotPositionSetpoint.x, 1.0, 0.8);
    EXPECT_NEAR(output.robotPositionSetpoint.y, -1.0, 0.8);
    EXPECT_NEAR(output.robotPositionSetpoint.rz, 0.0, PATH_PLANNING_NUMERICAL_TOLERANCE);
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

