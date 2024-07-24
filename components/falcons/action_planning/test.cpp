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

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

