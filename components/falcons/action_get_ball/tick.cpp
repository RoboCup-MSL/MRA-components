// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "FalconsActionGetBall.hpp"

// dependent (generated) component headers:
#include "FalconsActionFetchBall.hpp"
#include "FalconsActionCatchBall.hpp"

using namespace MRA;

// custom includes, if any
#include <cmath>
#include "geometry.hpp"


// template to call subcomponent FalconsActionFetchBall or FalconsActionCatchBall
template <typename SubcomponentType>
int callSubcomponent(
    google::protobuf::Timestamp timestamp,
    FalconsActionGetBall::InputType const &input,
    typename SubcomponentType::ParamsType const &subcomponent_params,
    typename SubcomponentType::StateType *state,
    FalconsActionGetBall::OutputType &output,
    FalconsActionGetBall::DiagnosticsType &diagnostics
)
{
    SubcomponentType subcomponent;
    typename SubcomponentType::InputType subcomponent_input;
    std::string tmpdata;
    input.SerializeToString(&tmpdata);
    subcomponent_input.ParseFromString(tmpdata);
    typename SubcomponentType::OutputType subcomponent_output;
    typename SubcomponentType::StateType subcomponent_state = *state;
    typename SubcomponentType::DiagnosticsType subcomponent_diagnostics;
    int result = subcomponent.tick(
        timestamp,
        subcomponent_input,
        subcomponent_params,
        subcomponent_state,
        subcomponent_output,
        subcomponent_diagnostics
    );
    output.set_actionresult(subcomponent_output.actionresult());
    diagnostics.set_failurereason(subcomponent_diagnostics.failurereason());
    *state = subcomponent_state;
    if (subcomponent_output.has_motiontarget())
    {
        *output.mutable_motiontarget() = subcomponent_output.motiontarget();
    }
    output.set_bhenabled(subcomponent_output.bhenabled());
    return result;
}

int FalconsActionGetBall::FalconsActionGetBall::tick
(
    google::protobuf::Timestamp timestamp,   // absolute timestamp
    InputType  const           &input,       // input data, type generated from Input.proto
    ParamsType const           &params,      // configuration parameters, type generated from Params.proto
    StateType                  &state,       // state data, type generated from State.proto
    OutputType                 &output,      // output data, type generated from Output.proto
    DiagnosticsType            &diagnostics  // diagnostics data, type generated from Diagnostics.proto
)
{
    int error_value = 0;
    MRA_LOG_TICK();

    float vx = input.worldstate().ball().velocity().x();
    float vy = input.worldstate().ball().velocity().y();
    float ball_speed = sqrt(vx * vx + vy * vy);

    if (ball_speed < params.ballspeedfetchthreshold())
    {
        // use a template to call component: FalconsActionFetchBall
        return callSubcomponent<FalconsActionFetchBall::FalconsActionFetchBall>(
            timestamp,
            input,
            params.fetchball(),
            state.mutable_fetchball(),
            output,
            diagnostics
        );
    }

    // check if the ball is moving towards the robot
    MRA::Geometry::Position robot_position(input.worldstate().robot().position());
    MRA::Geometry::Velocity ball_velocity(input.worldstate().ball().velocity());
    MRA::Geometry::Velocity ball_velocity_rcs = ball_velocity;
    ball_velocity_rcs.transformFcsToRcs(robot_position);
    bool ballmovingtowardsrobot = ball_velocity_rcs.y < 0;

    if (ballmovingtowardsrobot)
    {
        // use a template to call component: FalconsActionCatchBall
        return callSubcomponent<FalconsActionCatchBall::FalconsActionCatchBall>(
            timestamp,
            input,
            params.catchball(),
            state.mutable_catchball(),
            output,
            diagnostics
        );
    }

    // TODO: maybe chase/swerve after the ball?

    return error_value;
}

