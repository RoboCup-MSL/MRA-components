// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "RobotsportsLocalBall.hpp"

using namespace MRA;

// custom includes, if any
#include "RobotsportsLocalBallPreprocessor.hpp"
#include "RobotsportsLocalBallTracking.hpp"

int RobotsportsLocalBall::RobotsportsLocalBall::tick
(
    google::protobuf::Timestamp timestamp,   // absolute timestamp
    InputType  const           &input,       // input data, type generated from Input.proto
    ParamsType const           &params,      // configuration parameters, type generated from Params.proto
    StateType                  &state,       // state data, type generated from State.proto
    OutputType                 &output,      // output data, type generated from Output.proto
    LocalType                  &local        // local/diagnostics data, type generated from Local.proto
)
{
    int error_value = 0;
    MRA_LOG_TICK();

    // user implementation goes here

    auto preproccessor = RobotsportsLocalBallPreprocessor::RobotsportsLocalBallPreprocessor();
    auto preproccessor_input = RobotsportsLocalBallPreprocessor::Input();
    auto preproccessor_output = RobotsportsLocalBallPreprocessor::Output();
    auto preproccessor_local = RobotsportsLocalBallPreprocessor::LocalType();
    auto preproccessor_state = RobotsportsLocalBallPreprocessor::StateType();
    auto preproccessor_params = preproccessor.defaultParams();

    preproccessor_input.mutable_frontcamera_balls()->Clear();
    for (auto idx = 0; idx < input.frontcamera_balls().size(); idx++) {
        preproccessor_input.mutable_frontcamera_balls()->Add()->CopyFrom(input.frontcamera_balls(idx));
    }
    preproccessor_input.mutable_omnivision_balls()->Clear();
    for (auto idx = 0; idx < input.omnivision_balls().size(); idx++) {
        preproccessor_input.mutable_omnivision_balls()->Add()->CopyFrom(input.omnivision_balls(idx));
    }
    error_value = preproccessor.tick(timestamp, preproccessor_input, preproccessor_params, preproccessor_state, preproccessor_output, preproccessor_local);

    if (error_value == 0) {
        auto ball_tracker = RobotsportsLocalBallTracking::RobotsportsLocalBallTracking();
        auto ball_tracker_input = RobotsportsLocalBallTracking::Input();
        auto ball_tracker_output = RobotsportsLocalBallTracking::Output();
        auto ball_tracker_local = RobotsportsLocalBallTracking::LocalType();
        auto ball_tracker_state = RobotsportsLocalBallTracking::StateType();
        auto ball_tracker_params = ball_tracker.defaultParams();

        ball_tracker_input.mutable_candidates()->Clear();
        for (auto idx = 0; idx < preproccessor_output.candidates().size(); idx++) {
            ball_tracker_input.mutable_candidates()->Add()->CopyFrom(preproccessor_output.candidates(idx));
        }
        ball_tracker_state.set_is_initialized(state.is_initialized());

        error_value = ball_tracker.tick(timestamp, ball_tracker_input, ball_tracker_params, ball_tracker_state, ball_tracker_output, ball_tracker_local);
        state.set_is_initialized(ball_tracker_state.is_initialized());
        output.mutable_ball()->CopyFrom(ball_tracker_output.ball());

    }

    return error_value;
}

