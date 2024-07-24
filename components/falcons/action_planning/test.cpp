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
    return google::protobuf::util::MessageDifferencer::Equivalent(arg, expected);
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
    FalconsActionPlanning::Local local;

    virtual void SetUp()
    {
        params = planner.defaultParams();
    }

    virtual void TearDown()
    {
    }

    void feedTick()
    {
        planner.tick(timestamp, input, params, state, output, local);
        advanceTime(0.10); // Advance time by 0.10 seconds for each tick
    }

    void setWorldState(const Datatypes::WorldState &worldState)
    {
        *input.mutable_worldstate() = worldState;
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
        return output.actionresult();
    }

    FalconsActionPlanning::Local getDiagnostics()
    {
        return local;
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
    testActionInputs.set_type(Datatypes::ACTION_STOP);
    setActionInputs(testActionInputs);

    // Tick
    feedTick();

    // Setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_move()->set_stop(true);
    expectedSetpoints.mutable_bh()->set_enabled(false);
    Datatypes::ActionResult expectedActionResult = Datatypes::PASSED;

    // Check
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
}

TEST_F(TestActionPlanner, TickStopBhEnabled)
{
    MRA_TRACE_TEST_FUNCTION();

    // Setup inputs
    FalconsActionPlanning::ActionInputs testActionInputs;
    testActionInputs.set_type(Datatypes::ACTION_STOP);
    testActionInputs.mutable_stop()->set_ballhandlersenabled(true);
    setActionInputs(testActionInputs);

    // Tick
    feedTick();

    // Setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_move()->set_stop(true);
    expectedSetpoints.mutable_bh()->set_enabled(true);
    Datatypes::ActionResult expectedActionResult = Datatypes::PASSED;

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
    testActionInput.set_type(Datatypes::ACTION_MOVE);
    auto moveInputs = new FalconsActionPlanning::ActionMoveInputs();
    moveInputs->mutable_target()->mutable_position()->set_x(1.0);
    moveInputs->mutable_target()->mutable_position()->set_y(1.0);
    moveInputs->mutable_target()->mutable_position()->set_rz(0.0);
    moveInputs->set_ballhandlersenabled(true);
    testActionInput.set_allocated_move(moveInputs);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInput);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints; // No motion setpoint
    expectedSetpoints.mutable_bh()->set_enabled(true);
    expectedSetpoints.mutable_move()->set_stop(true);
    Datatypes::ActionResult expectedActionResult = Datatypes::PASSED;

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
    testActionInput.set_type(Datatypes::ACTION_MOVE);
    auto moveInputs = new FalconsActionPlanning::ActionMoveInputs();
    moveInputs->mutable_target()->mutable_position()->set_x(2.0);
    moveInputs->mutable_target()->mutable_position()->set_y(2.0);
    moveInputs->mutable_target()->mutable_position()->set_rz(0.5);
    moveInputs->set_ballhandlersenabled(false);
    testActionInput.set_allocated_move(moveInputs);

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
    Datatypes::ActionResult expectedActionResult = Datatypes::RUNNING;

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
    testActionInput.set_type(Datatypes::ACTION_GETBALL);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInput);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_bh()->set_enabled(true);
    Datatypes::ActionResult expectedActionResult = Datatypes::PASSED;

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
    testActionInput.set_type(Datatypes::ACTION_GETBALL);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInput);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_bh()->set_enabled(true); // TODO: this does not make much sense and likely gets overruled at lower levels in Falcons SW
    Datatypes::ActionResult expectedActionResult = Datatypes::FAILED;

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
    testActionInput.set_type(Datatypes::ACTION_GETBALL);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInput);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_bh()->set_enabled(true);
    Datatypes::ActionResult expectedActionResult = Datatypes::FAILED;

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
    testActionInput.set_type(Datatypes::ACTION_GETBALL);

    // set inputs in the planner
    setWorldState(testWorldState);
    setActionInputs(testActionInput);

    // run tick
    feedTick();

    // setup expected outputs
    FalconsActionPlanning::Setpoints expectedSetpoints;
    expectedSetpoints.mutable_bh()->set_enabled(true); // TODO: when accidentally scrumming with teammate, maybe better to disable?
    Datatypes::ActionResult expectedActionResult = Datatypes::FAILED;

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
    testActionInput.set_type(Datatypes::ACTION_GETBALL);

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
    Datatypes::ActionResult expectedActionResult = Datatypes::RUNNING;

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
    testActionInput.set_type(Datatypes::ACTION_PASS);
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
    Datatypes::ActionResult expectedActionResult = Datatypes::FAILED;

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
    testActionInput.set_type(Datatypes::ACTION_PASS);
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
    Datatypes::ActionResult expectedActionResult = Datatypes::RUNNING;

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
    testActionInputs.set_type(Datatypes::ACTION_PASS);
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
    Datatypes::ActionResult expectedActionResult = Datatypes::RUNNING;

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
    FalconsActionPlanning::Local expectedDiagnostics;
    expectedDiagnostics.mutable_action()->set_type(Datatypes::ACTION_PASS);
    expectedDiagnostics.mutable_action()->mutable_pass()->set_aimerror(0.0);
    expectedActionResult = Datatypes::PASSED;
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
    expectedDiagnostics.mutable_action()->set_type(Datatypes::ACTION_PASS);
    expectedDiagnostics.mutable_action()->mutable_pass()->set_aimerror(0.058755815);
    expectedActionResult = Datatypes::PASSED;
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
    EXPECT_THAT(getDiagnostics(), EqualsProto(expectedDiagnostics));

    // Alternate ending 2
    // Tick: simulate that the ball did not arrive at target and the cooldown expired, action should finish FAILED
    testWorldState.mutable_ball()->mutable_position()->set_x(3.0);
    testWorldState.mutable_ball()->mutable_position()->set_y(3.0);
    setWorldState(testWorldState);
    advanceTime(5.0);
    feedTick();

    // Check the outputs
    expectedActionResult = Datatypes::FAILED;
    EXPECT_THAT(getLastSetpoints(), EqualsProto(expectedSetpoints));
    EXPECT_EQ(getLastActionResult(), expectedActionResult);
}

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

