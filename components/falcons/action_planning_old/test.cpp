// Include testframework
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_factory.hpp"
#include <google/protobuf/util/message_differencer.h>
using namespace ::testing;

// System under test:
#include "FalconsActionPlanning.hpp"
using namespace MRA;

// Custom matcher for protobuf comparison
MATCHER_P(EqualsProto, expected, "")
{
    return google::protobuf::util::MessageDifferencer::ApproximatelyEquivalent(arg, expected);
}

MATCHER_P2(EqualsProtoWithTolerance, expected, tolerance, "")
{
    google::protobuf::util::MessageDifferencer differencer;
    google::protobuf::util::DefaultFieldComparator comparator;

    // configure the comparator to use the tolerance for floating point fields
    comparator.set_float_comparison(google::protobuf::util::DefaultFieldComparator::APPROXIMATE);
    comparator.SetDefaultFractionAndMargin(tolerance, tolerance);

    // set the custom comparator on the differencer
    differencer.set_field_comparator(&comparator);
    return differencer.Compare(arg, expected);
}

// Test class
class TestActionPlanner : public ::testing::Test
{
protected:
    FalconsActionPlanning::FalconsActionPlanning planner;
    google::protobuf::Timestamp timestamp = google::protobuf::util::TimeUtil::GetCurrentTime();
    FalconsActionPlanning::Input input;
    FalconsActionPlanning::Params params;
    FalconsActionPlanning::State state;
    FalconsActionPlanning::Output output;
    FalconsActionPlanning::Diagnostics diagnostics;
    int lastTickResult = 0;

    virtual void SetUp()
    {
        params = planner.defaultParams();
    }

    virtual void TearDown()
    {
    }

    void feedTick()
    {
        lastTickResult = planner.tick(timestamp, input, params, state, output, diagnostics);
        advanceTime(0.10); // Advance time by 0.10 seconds for each tick
    }

    void setWorldState(const Datatypes::WorldState &worldState)
    {
        *input.mutable_worldstate() = worldState;
    }

    void setParams(const FalconsActionPlanning::Params &p)
    {
        params.MergeFrom(p);
    }

    void setActionInputs(const FalconsActionPlanning::ActionInputs &ActionInputs)
    {
        *input.mutable_action() = ActionInputs;
    }

    FalconsActionPlanning::Setpoints getLastSetpoints()
    {
        return output.setpoints();
    }

    Datatypes::ActionResult getLastActionResult()
    {
        EXPECT_EQ(lastTickResult, 0);
        return output.actionresult();
    }

    FalconsActionPlanning::Diagnostics getDiagnostics()
    {
        return diagnostics;
    }

    void advanceTime(double seconds)
    {
        int64_t nanos = static_cast<int64_t>(seconds * 1e9);
        timestamp.set_seconds(timestamp.seconds() + nanos / 1000000000);
        timestamp.set_nanos(timestamp.nanos() + nanos % 1000000000);
    }

};

// Basic tick shall run OK and return error_value 0.
TEST(FalconsActionPlanningTest, basicTick)
{
    // Arrange
    auto m = FalconsActionPlanning::FalconsActionPlanning();

    // Act
    int error_value = m.tick();

    // Assert
    EXPECT_EQ(error_value, 1); // return code 1 signals no action specified on input (ACTION_INVALID)
}

TEST_F(TestActionPlanner, TickStopBhDisabled)
{
    MRA_TRACE_TEST_FUNCTION();

    // Setup inputs
    FalconsActionPlanning::ActionInputs testActionInputs;
    testActionInputs.set_type(Datatypes::ActionType::ACTION_STOP);
    setActionInputs(testActionInputs);

    // Tick
    feedTick();

    // Setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_move()->set_stop(true);
    expectedSetpoints.mutable_bh()->set_enabled(false);
    Datatypes::ActionResult expectedActionResult = Datatypes::ActionResult::PASSED;

    // Check
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
}

TEST_F(TestActionPlanner, TickStopBhEnabled)
{
    MRA_TRACE_TEST_FUNCTION();

    // Setup inputs
    FalconsActionPlanning::ActionInputs testActionInputs;
    testActionInputs.set_type(Datatypes::ActionType::ACTION_STOP);
    testActionInputs.mutable_stop()->set_ballhandlersenabled(true);
    setActionInputs(testActionInputs);

    // Tick
    feedTick();

    // Setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_move()->set_stop(true);
    expectedSetpoints.mutable_bh()->set_enabled(true);
    Datatypes::ActionResult expectedActionResult = Datatypes::ActionResult::PASSED;

    // Check
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
}

TEST_F(TestActionPlanner, TickTestMoveActionAtTarget)
{
    MRA_TRACE_TEST_FUNCTION();

    // setup inputs
    Datatypes::WorldState testWorldState;
    testWorldState.mutable_robot()->mutable_position()->set_x(1.0);
    testWorldState.mutable_robot()->mutable_position()->set_y(1.0);
    testWorldState.mutable_robot()->mutable_position()->set_rz(0.0);

    FalconsActionPlanning::ActionInputs testActionInput;
    testActionInput.set_type(Datatypes::ActionType::ACTION_MOVE);
    testActionInput.mutable_move()->mutable_motiontarget()->mutable_position()->set_x(1.0);
    testActionInput.mutable_move()->mutable_motiontarget()->mutable_position()->set_y(1.0);
    testActionInput.mutable_move()->mutable_motiontarget()->mutable_position()->set_rz(0.0);
    testActionInput.mutable_move()->set_ballhandlersenabled(true);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInput);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints; // No motion setpoint
    expectedSetpoints.mutable_bh()->set_enabled(true);
    expectedSetpoints.mutable_move()->set_stop(true);
    Datatypes::ActionResult expectedActionResult = Datatypes::ActionResult::PASSED;

    // check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestMoveActionNotAtTarget)
{
    MRA_TRACE_TEST_FUNCTION();

    // setup inputs
    Datatypes::WorldState testWorldState;
    testWorldState.mutable_robot()->mutable_position()->set_x(1.0);
    testWorldState.mutable_robot()->mutable_position()->set_y(1.0);
    testWorldState.mutable_robot()->mutable_position()->set_rz(0.0);

    FalconsActionPlanning::ActionInputs testActionInput;
    testActionInput.set_type(Datatypes::ActionType::ACTION_MOVE);
    testActionInput.mutable_move()->mutable_motiontarget()->mutable_position()->set_x(2.0);
    testActionInput.mutable_move()->mutable_motiontarget()->mutable_position()->set_y(2.0);
    testActionInput.mutable_move()->mutable_motiontarget()->mutable_position()->set_rz(0.5);
    testActionInput.mutable_move()->set_ballhandlersenabled(false);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInput);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_x(2.0);
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_y(2.0);
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_rz(0.5);
    expectedSetpoints.mutable_bh()->set_enabled(false);
    Datatypes::ActionResult expectedActionResult = Datatypes::ActionResult::RUNNING;

    // check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestMoveActionDribble)
{
    MRA_TRACE_TEST_FUNCTION();

    // setup inputs
    Datatypes::WorldState testWorldState;
    testWorldState.mutable_robot()->mutable_position()->set_x(1.0);
    testWorldState.mutable_robot()->set_hasball(true);

    FalconsActionPlanning::ActionInputs testActionInput;
    testActionInput.set_type(Datatypes::ActionType::ACTION_MOVE);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInput);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_move()->set_motiontype(1);
    Datatypes::ActionResult expectedActionResult = Datatypes::ActionResult::RUNNING;

    // check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestGetBallActionHasBall)
{
    MRA_TRACE_TEST_FUNCTION();

    // setup inputs
    Datatypes::WorldState testWorldState;
    testWorldState.mutable_robot()->set_hasball(true);

    FalconsActionPlanning::ActionInputs testActionInput;
    testActionInput.set_type(Datatypes::ActionType::ACTION_GETBALL);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInput);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_bh()->set_enabled(true);
    Datatypes::ActionResult expectedActionResult = Datatypes::ActionResult::PASSED;

    // check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestGetBallActionInactiveRobot)
{
    MRA_TRACE_TEST_FUNCTION();

    // setup inputs
    Datatypes::WorldState testWorldState;
    testWorldState.mutable_robot()->set_active(false);

    FalconsActionPlanning::ActionInputs testActionInput;
    testActionInput.set_type(Datatypes::ActionType::ACTION_GETBALL);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInput);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_bh()->set_enabled(true); // TODO: this does not make much sense and likely gets overruled at lower levels in Falcons SW
    Datatypes::ActionResult expectedActionResult = Datatypes::ActionResult::FAILED;

    // check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestGetBallActionNoBall)
{
    MRA_TRACE_TEST_FUNCTION();

    // setup inputs
    Datatypes::WorldState testWorldState;
    testWorldState.clear_ball();

    FalconsActionPlanning::ActionInputs testActionInput;
    testActionInput.set_type(Datatypes::ActionType::ACTION_GETBALL);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInput);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_bh()->set_enabled(true);
    Datatypes::ActionResult expectedActionResult = Datatypes::ActionResult::FAILED;

    // check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestGetBallActionTeammateHasBall)
{
    MRA_TRACE_TEST_FUNCTION();

    // setup inputs
    Datatypes::WorldState testWorldState;
    auto teammate = testWorldState.add_teammates();
    teammate->set_hasball(true);

    FalconsActionPlanning::ActionInputs testActionInput;
    testActionInput.set_type(Datatypes::ActionType::ACTION_GETBALL);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInput);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_bh()->set_enabled(true); // TODO: when accidentally scrumming with teammate, maybe better to disable?
    Datatypes::ActionResult expectedActionResult = Datatypes::ActionResult::FAILED;

    // check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestGetBallActionRunning)
{
    MRA_TRACE_TEST_FUNCTION();

    // setup inputs
    Datatypes::WorldState testWorldState;
    testWorldState.mutable_robot()->set_active(true);
    testWorldState.mutable_ball()->mutable_position()->set_x(2.0);
    testWorldState.mutable_ball()->mutable_position()->set_y(2.0);
    testWorldState.mutable_robot()->mutable_position()->set_x(1.0);
    testWorldState.mutable_robot()->mutable_position()->set_y(1.0);

    FalconsActionPlanning::ActionInputs testActionInput;
    testActionInput.set_type(Datatypes::ActionType::ACTION_GETBALL);
    // not specifying radius should lead to any ball

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInput);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_bh()->set_enabled(true);
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_x(2.0);
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_y(2.0);
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_rz(-0.78539816339744828);
    Datatypes::ActionResult expectedActionResult = Datatypes::ActionResult::RUNNING;

    // check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestGetBallFarNoRadiusFail)
{
    MRA_TRACE_TEST_FUNCTION();

    // setup inputs
    Datatypes::WorldState testWorldState;
    testWorldState.mutable_robot()->set_active(true);
    testWorldState.mutable_ball()->mutable_position()->set_x(2.0);
    testWorldState.mutable_ball()->mutable_position()->set_y(2.0);
    testWorldState.mutable_robot()->mutable_position()->set_x(-10.0);
    testWorldState.mutable_robot()->mutable_position()->set_y(-10.0);

    FalconsActionPlanning::ActionInputs testActionInput;
    testActionInput.set_type(Datatypes::ActionType::ACTION_GETBALL);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInput);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_bh()->set_enabled(true);
    Datatypes::ActionResult expectedActionResult = Datatypes::ActionResult::FAILED;

    // check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestGetBallFarRadiusRunning)
{
    MRA_TRACE_TEST_FUNCTION();

    // setup inputs
    Datatypes::WorldState testWorldState;
    testWorldState.mutable_robot()->set_active(true);
    testWorldState.mutable_ball()->mutable_position()->set_x(2.0);
    testWorldState.mutable_ball()->mutable_position()->set_y(2.0);
    testWorldState.mutable_robot()->mutable_position()->set_x(-10.0);
    testWorldState.mutable_robot()->mutable_position()->set_y(-10.0);

    FalconsActionPlanning::ActionInputs testActionInput;
    testActionInput.set_type(Datatypes::ActionType::ACTION_GETBALL);
    testActionInput.mutable_getball()->set_radius(20.0);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInput);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_bh()->set_enabled(true);
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_x(2.0);
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_y(2.0);
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_rz(-0.78539816339744828);
    Datatypes::ActionResult expectedActionResult = Datatypes::ActionResult::RUNNING;

    // check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestGetBallRadiusClipping)
{
    MRA_TRACE_TEST_FUNCTION();

    // setup inputs
    Datatypes::WorldState testWorldState;
    testWorldState.mutable_robot()->set_active(true);
    testWorldState.mutable_ball()->mutable_position()->set_x(2.0);
    testWorldState.mutable_ball()->mutable_position()->set_y(2.0);
    testWorldState.mutable_robot()->mutable_position()->set_x(-10.0);
    testWorldState.mutable_robot()->mutable_position()->set_y(-10.0);

    FalconsActionPlanning::ActionInputs testActionInput;
    testActionInput.set_type(Datatypes::ActionType::ACTION_GETBALL);
    testActionInput.mutable_getball()->set_radius(10.0);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInput);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_bh()->set_enabled(true);
    Datatypes::ActionResult expectedActionResult = Datatypes::ActionResult::FAILED;

    // check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestPassActionNoBall)
{
    MRA_TRACE_TEST_FUNCTION();

    // setup inputs
    Datatypes::WorldState testWorldState;
    testWorldState.mutable_robot()->set_hasball(false);

    FalconsActionPlanning::ActionInputs testActionInput;
    testActionInput.set_type(Datatypes::ActionType::ACTION_PASS);
    testActionInput.mutable_pass()->mutable_target()->mutable_position()->set_x(5.0);
    testActionInput.mutable_pass()->mutable_target()->mutable_position()->set_y(5.0);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInput);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_shoot()->set_phase(FalconsActionAimedKick::SHOOT_PHASE_INVALID);
    Datatypes::ActionResult expectedActionResult = Datatypes::ActionResult::FAILED;

    // check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestPassActionRunning)
{
    MRA_TRACE_TEST_FUNCTION();

    // setup inputs
    Datatypes::WorldState testWorldState;
    testWorldState.mutable_robot()->set_hasball(true);
    testWorldState.mutable_robot()->mutable_position()->set_x(1.0);
    testWorldState.mutable_robot()->mutable_position()->set_y(2.0);
    testWorldState.mutable_robot()->mutable_position()->set_rz(0.0);
    testWorldState.mutable_ball()->mutable_position()->set_x(1.0);
    testWorldState.mutable_ball()->mutable_position()->set_y(2.1);

    FalconsActionPlanning::ActionInputs testActionInput;
    testActionInput.set_type(Datatypes::ActionType::ACTION_PASS);
    testActionInput.mutable_pass()->mutable_target()->mutable_position()->set_x(5.0);
    testActionInput.mutable_pass()->mutable_target()->mutable_position()->set_y(6.0);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInput);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_x(1.0);
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_y(2.0);
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_rz(-0.78539816339744828);
    expectedSetpoints.mutable_bh()->set_enabled(true);
    expectedSetpoints.mutable_shoot()->set_type(FalconsActionPlanning::SHOOT_TYPE_PASS);
    expectedSetpoints.mutable_shoot()->set_phase(FalconsActionAimedKick::SHOOT_PHASE_PREPARE);
    expectedSetpoints.mutable_shoot()->set_pos_x(5.0);
    expectedSetpoints.mutable_shoot()->set_pos_y(6.0);
    Datatypes::ActionResult expectedActionResult = Datatypes::ActionResult::RUNNING;

    // check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestPassActionStateTransitions)
{
    MRA_TRACE_TEST_FUNCTION();

    // Setup initial inputs
    Datatypes::WorldState testWorldState;
    testWorldState.mutable_robot()->set_hasball(true);
    testWorldState.mutable_robot()->mutable_position()->set_x(1.0);
    testWorldState.mutable_robot()->mutable_position()->set_y(2.0);
    testWorldState.mutable_robot()->mutable_position()->set_rz(0.0);
    testWorldState.mutable_ball()->mutable_position()->set_x(1.0);
    testWorldState.mutable_ball()->mutable_position()->set_y(2.1);

    FalconsActionPlanning::ActionInputs testActionInputs;
    testActionInputs.set_type(Datatypes::ActionType::ACTION_PASS);
    testActionInputs.mutable_pass()->mutable_target()->mutable_position()->set_x(5.0);
    testActionInputs.mutable_pass()->mutable_target()->mutable_position()->set_y(6.0);

    // Set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInputs);

    // Initial tick, robot has ball, go into PREPARE phase, aiming/rotating towards target
    feedTick();

    // Setup expected outputs for the first tick
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_x(1.0);
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_y(2.0);
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_rz(-0.78539816339744828);
    expectedSetpoints.mutable_bh()->set_enabled(true);
    expectedSetpoints.mutable_shoot()->set_type(FalconsActionPlanning::SHOOT_TYPE_PASS);
    expectedSetpoints.mutable_shoot()->set_phase(FalconsActionAimedKick::SHOOT_PHASE_PREPARE);
    expectedSetpoints.mutable_shoot()->set_pos_x(5.0);
    expectedSetpoints.mutable_shoot()->set_pos_y(6.0);
    Datatypes::ActionResult expectedActionResult = Datatypes::ActionResult::RUNNING;

    // Check the outputs after the first tick
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);

    // For second tick, update inputs to finish the robot rotation, so the action will advance to DISCHARGE phase and kick the ball
    testWorldState.mutable_robot()->mutable_position()->set_x(1.0);
    testWorldState.mutable_robot()->mutable_position()->set_y(2.0);
    testWorldState.mutable_robot()->mutable_position()->set_rz(-0.78539816339744828);
    setWorldState(testWorldState);
    feedTick();

    // Setup expected outputs for the second tick
    expectedSetpoints.mutable_move()->Clear();
    expectedSetpoints.mutable_shoot()->Clear();
    expectedSetpoints.mutable_shoot()->set_phase(FalconsActionAimedKick::SHOOT_PHASE_DISCHARGE);

    // Check the outputs after the second tick
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);

    // Tick: just check that the action transitions to COOLDOWN phase
    feedTick();

    // Check the outputs
    expectedSetpoints.mutable_shoot()->set_phase(FalconsActionAimedKick::SHOOT_PHASE_COOLDOWN);
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);

    // Tick: simulate that the ball has left the robot, action should still be RUNNING
    testWorldState.mutable_ball()->mutable_position()->set_x(3.0);
    testWorldState.mutable_ball()->mutable_position()->set_y(3.0);
    testWorldState.mutable_robot()->set_hasball(false);
    setWorldState(testWorldState);
    feedTick();

    // Check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);

    // Tick: simulate that the ball has arrived at target, action should finish PASSED
    testWorldState.mutable_ball()->mutable_position()->set_x(5.0);
    testWorldState.mutable_ball()->mutable_position()->set_y(6.0);
    setWorldState(testWorldState);
    feedTick();

    // Check the outputs
    FalconsActionPlanning::Diagnostics expectedDiagnostics;
    expectedDiagnostics.mutable_action()->set_type(Datatypes::ActionType::ACTION_PASS);
    expectedDiagnostics.mutable_action()->mutable_pass()->set_aimerror(0.0);
    expectedActionResult = Datatypes::ActionResult::PASSED;
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
    EXPECT_THAT(getDiagnostics(), EqualsProto(expectedDiagnostics));

    // Alternate ending 1
    // Tick: simulate that the ball slightly missed target, check reported aim error
    testWorldState.mutable_ball()->mutable_position()->set_x(5.0);
    testWorldState.mutable_ball()->mutable_position()->set_y(6.5);
    setWorldState(testWorldState);
    feedTick();

    // Check the outputs
    expectedDiagnostics.mutable_action()->set_type(Datatypes::ActionType::ACTION_PASS);
    expectedDiagnostics.mutable_action()->mutable_pass()->set_aimerror(0.058755815);
    expectedActionResult = Datatypes::ActionResult::PASSED;
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
    EXPECT_THAT(getDiagnostics(), EqualsProtoWithTolerance(expectedDiagnostics, 1e-9));

    // Alternate ending 2
    // Tick: simulate that the ball did not arrive at target and the cooldown expired, action should finish FAILED
    testWorldState.mutable_ball()->mutable_position()->set_x(3.0);
    testWorldState.mutable_ball()->mutable_position()->set_y(3.0);
    setWorldState(testWorldState);
    advanceTime(5.0);
    feedTick();

    // Check the outputs
    expectedActionResult = Datatypes::ActionResult::FAILED;
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestActionParkSuccess)
{
    MRA_TRACE_TEST_FUNCTION();

    // setup inputs
    Datatypes::WorldState testWorldState;
    testWorldState.mutable_robot()->set_hasball(false);
    testWorldState.mutable_robot()->mutable_position()->set_x(4.0);
    testWorldState.mutable_robot()->mutable_position()->set_y(3.0);
    testWorldState.mutable_robot()->mutable_position()->set_rz(0.0);

    // set no obstacles in the TechnicalTeamArea
    // TechnicalTeamArea configuration
    FalconsActionPlanning::Params params;
    auto tta = params.mutable_action()->mutable_park()->mutable_tta();
    tta->mutable_center()->set_x(7.0);
    tta->mutable_center()->set_y(3.0);
    tta->mutable_size()->set_x(1.0);
    tta->mutable_size()->set_y(6.0);
    setParams(params);

    // set action
    FalconsActionPlanning::ActionInputs testActionInputs;
    testActionInputs.set_type(Datatypes::ActionType::ACTION_PARK);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInputs);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_x(7.0); // center.x of TTA
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_y(3.0); // center.y of TTA
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_rz(0.5 * M_PI); // facing the field
    expectedSetpoints.mutable_bh()->set_enabled(false);

    MRA::Datatypes::ActionResult expectedActionResult = MRA::Datatypes::ActionResult::RUNNING;

    // check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);

    // Simulate robot arriving at the target position
    testWorldState.mutable_robot()->mutable_position()->set_x(7.0);
    testWorldState.mutable_robot()->mutable_position()->set_y(3.0);
    testWorldState.mutable_robot()->mutable_position()->set_rz(0.5 * M_PI);

    setWorldState(testWorldState);
    feedTick();

    // Expected result after arriving at the position
    expectedActionResult = MRA::Datatypes::ActionResult::PASSED;

    // check the outputs
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestActionParkFailDueToObstacles)
{
    MRA_TRACE_TEST_FUNCTION();

    // setup inputs
    Datatypes::WorldState testWorldState;
    testWorldState.mutable_robot()->set_hasball(false);
    testWorldState.mutable_robot()->mutable_position()->set_x(0.0);
    testWorldState.mutable_robot()->mutable_position()->set_y(0.0);
    testWorldState.mutable_robot()->mutable_position()->set_rz(0.0);

    // add obstacles in the TechnicalTeamArea
    auto obstacle1 = testWorldState.add_obstacles();
    obstacle1->mutable_position()->set_x(7.0);
    obstacle1->mutable_position()->set_y(3.0);

    auto obstacle2 = testWorldState.add_obstacles();
    obstacle2->mutable_position()->set_x(7.0);
    obstacle2->mutable_position()->set_y(2.5);

    // TechnicalTeamArea configuration
    FalconsActionPlanning::Params params;
    auto tta = params.mutable_action()->mutable_park()->mutable_tta();
    tta->mutable_center()->set_x(7.0);
    tta->mutable_center()->set_y(3.0);
    tta->mutable_size()->set_x(1.0);
    tta->mutable_size()->set_y(6.0);
    setParams(params);

    // set action
    FalconsActionPlanning::ActionInputs testActionInputs;
    testActionInputs.set_type(Datatypes::ActionType::ACTION_PARK);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInputs);

    // run tick, expect to move towards TTA
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_x(5.0);
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_y(3.0);
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_rz(0.5 * M_PI);
    expectedSetpoints.mutable_bh()->set_enabled(false);

    MRA::Datatypes::ActionResult expectedActionResult = MRA::Datatypes::ActionResult::RUNNING;

    // check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);

    // simulate that robot is nearby TTA, now check for blocked position
    testWorldState.mutable_robot()->mutable_position()->set_x(5.0);
    testWorldState.mutable_robot()->mutable_position()->set_y(3.0);
    testWorldState.mutable_robot()->mutable_position()->set_rz(0.0);
    setWorldState(testWorldState);

    // run tick
    feedTick();

    // setup expected outputs
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_x(0.0); // No movement
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_y(0.0); // No movement
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_rz(0.0); // No movement
    expectedSetpoints.mutable_bh()->set_enabled(false);

    expectedActionResult = MRA::Datatypes::ActionResult::FAILED;

    // check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestActionCatchRobotHasBall)
{
    MRA_TRACE_TEST_FUNCTION();

    // Setup inputs: Robot already has the ball
    Datatypes::WorldState testWorldState;
    testWorldState.mutable_robot()->set_hasball(true);

    // Set inputs in the planner
    FalconsActionPlanning::ActionInputs testActionInputs;
    testActionInputs.set_type(Datatypes::ActionType::ACTION_CATCHBALL);
    setWorldState(testWorldState);
    setActionInputs(testActionInputs);

    // Run tick
    feedTick();

    // Setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_bh()->set_enabled(true);
    MRA::Datatypes::ActionResult expectedActionResult = MRA::Datatypes::ActionResult::PASSED;

    // Check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestActionCatchGoodWeather)
{
    MRA_TRACE_TEST_FUNCTION();

    // Setup inputs: Ball is moving towards the robot directly
    Datatypes::WorldState testWorldState;
    testWorldState.mutable_robot()->set_active(true);
    testWorldState.mutable_robot()->set_hasball(false);
    testWorldState.mutable_robot()->mutable_position()->set_x(0.0);
    testWorldState.mutable_robot()->mutable_position()->set_y(0.0);
    testWorldState.mutable_robot()->mutable_position()->set_rz(-0.5 * M_PI);

    // Ball position and velocity
    auto ball = testWorldState.mutable_ball();
    ball->mutable_position()->set_x(5.0); // Ball is 5 meters in front of robot
    ball->mutable_position()->set_y(0.5); // Small y offset w.r.t. robot
    ball->mutable_velocity()->set_x(-3.0); // Ball is moving towards the robot at 3 m/s
    ball->mutable_velocity()->set_y(0.0);

    // Params: Capture radius and ball speed threshold
    FalconsActionPlanning::Params params;
    params.mutable_action()->mutable_catchball()->set_captureradius(10.0); // 10 meters
    params.mutable_action()->mutable_catchball()->set_ballspeedthreshold(2.0); // Ball speed threshold = 2 m/s
    setParams(params);

    // Set inputs in the planner
    FalconsActionPlanning::ActionInputs testActionInputs;
    testActionInputs.set_type(Datatypes::ActionType::ACTION_CATCHBALL);
    setWorldState(testWorldState);
    setActionInputs(testActionInputs);

    // Run tick
    feedTick();

    // Setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_x(0.0); // Move in front of ball (within capture radius)
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_y(0.5); // Align to ball's y-axis
    expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_rz(-0.5 * M_PI); // Face the ball, ball is coming from positive x-axis
    expectedSetpoints.mutable_bh()->set_enabled(true); // Ball handlers enabled

    MRA::Datatypes::ActionResult expectedActionResult = MRA::Datatypes::ActionResult::RUNNING;

    // Check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestActionCatchEightDirections)
{
    MRA_TRACE_TEST_FUNCTION();

    // Ball will be dy=2m in front of robot, and dx=0.5m aside, moving in opposite direction (generally towards the robot)
    // Robot is expected to move 0.5 to the right
    double dy = 2.0;
    double dx = 0.5;

    // Test all 8 wind directions, to ensure coordinate transformations are ok
    for (int direction = 0; direction < 8; direction++)
    {
        MRA_LOG_DEBUG("direction=%d", direction);

        // Robot position, facing direction
        double a = direction * M_PI / 4;
        a += 0.01; // add a small offset to avoid exact 0 or 90 degrees
        double c = cos(a);
        double s = sin(a);
        double rz = a - 0.5 * M_PI; // correct for MSL coordinate system
        if (rz > M_PI) // wrap to [-pi, pi)
        {
            rz -= 2 * M_PI;
        }
        Datatypes::WorldState testWorldState;
        testWorldState.mutable_robot()->set_active(true);
        testWorldState.mutable_robot()->set_hasball(false);
        testWorldState.mutable_robot()->mutable_position()->set_x(0.0);
        testWorldState.mutable_robot()->mutable_position()->set_y(0.0);
        testWorldState.mutable_robot()->mutable_position()->set_rz(rz);

        // Ball position and velocity
        auto ball = testWorldState.mutable_ball();
        ball->mutable_position()->set_x(dy * c - dx * s);
        ball->mutable_position()->set_y(dx * c + dy * s);
        ball->mutable_velocity()->set_x(-dy * c);
        ball->mutable_velocity()->set_y(-dy * s);

        // Set inputs in the planner
        FalconsActionPlanning::ActionInputs testActionInputs;
        testActionInputs.set_type(Datatypes::ActionType::ACTION_CATCHBALL);
        setWorldState(testWorldState);
        setActionInputs(testActionInputs);

        // Run tick
        feedTick();

        // Setup expected outputs
        FalconsActionPlanning::Setpoints expectedSetpoints;
        expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_x(-dx * s);
        expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_y(dx * c);
        expectedSetpoints.mutable_move()->mutable_target()->mutable_position()->set_rz(rz);
        expectedSetpoints.mutable_bh()->set_enabled(true);

        MRA::Datatypes::ActionResult expectedActionResult = MRA::Datatypes::ActionResult::RUNNING;

        // Check the outputs
        EXPECT_THAT(getLastSetpoints(), EqualsProtoWithTolerance(expectedSetpoints, 1e-6));
        EXPECT_EQ(getLastActionResult(), expectedActionResult);
    }
}

TEST_F(TestActionPlanner, TickTestActionCatchBallMovingAway)
{
    MRA_TRACE_TEST_FUNCTION();

    // Setup inputs: Ball is moving away from the robot
    Datatypes::WorldState testWorldState;
    testWorldState.mutable_robot()->set_active(true);
    testWorldState.mutable_robot()->set_hasball(false);
    testWorldState.mutable_robot()->mutable_position()->set_x(0.0);
    testWorldState.mutable_robot()->mutable_position()->set_y(0.0);
    testWorldState.mutable_robot()->mutable_position()->set_rz(0.0);
    auto ball = testWorldState.mutable_ball();
    ball->mutable_position()->set_x(5.0); // Ball is 5 meters in front of robot
    ball->mutable_position()->set_y(0.0);
    ball->mutable_velocity()->set_x(3.0); // Ball is moving away from the robot at 3 m/s
    ball->mutable_velocity()->set_y(0.0);

    // Set inputs in the planner
    FalconsActionPlanning::ActionInputs testActionInputs;
    testActionInputs.set_type(Datatypes::ActionType::ACTION_CATCHBALL);
    setWorldState(testWorldState);
    setActionInputs(testActionInputs);

    // Run tick
    feedTick();

    // Setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_bh()->set_enabled(true); // Ball handlers enabled
    MRA::Datatypes::ActionResult expectedActionResult = MRA::Datatypes::ActionResult::FAILED;

    // Check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

TEST_F(TestActionPlanner, TickTestActionCatchBallStationary)
{
    MRA_TRACE_TEST_FUNCTION();

    // Setup inputs: Ball is stationary
    Datatypes::WorldState testWorldState;
    testWorldState.mutable_robot()->set_active(true);
    testWorldState.mutable_robot()->set_hasball(false);
    testWorldState.mutable_robot()->mutable_position()->set_x(0.0);
    testWorldState.mutable_robot()->mutable_position()->set_y(0.0);
    testWorldState.mutable_robot()->mutable_position()->set_rz(0.0);
    auto ball = testWorldState.mutable_ball();
    ball->mutable_position()->set_x(3.0);
    ball->mutable_position()->set_y(0.0);
    ball->mutable_velocity()->set_x(0.0);
    ball->mutable_velocity()->set_y(0.0);

    // Set inputs in the planner
    FalconsActionPlanning::ActionInputs testActionInputs;
    testActionInputs.set_type(Datatypes::ActionType::ACTION_CATCHBALL);
    setWorldState(testWorldState);
    setActionInputs(testActionInputs);

    // Run tick
    feedTick();

    // Setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_bh()->set_enabled(true); // Ball handlers enabled
    MRA::Datatypes::ActionResult expectedActionResult = MRA::Datatypes::ActionResult::RUNNING;

    // Check the outputs
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

