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

    // user implementation goes here

    Geometry::Position bpos = Geometry::Position(input.worldstate().ball().position()) - Geometry::Position(input.worldstate().robot().position());
    if (input.radius() > 0.0 && bpos.size() > input.radius()) {
        output.set_actionresult(Datatypes::ActionResult::FAILED);
    }
    else {

    float vx = input.worldstate().ball().velocity().x();
    float vy = input.worldstate().ball().velocity().y();
    float ball_speed = sqrt(vx * vx + vy * vy);

    if (ball_speed < params.ballspeedstationarythreshold())
    {
        // call component: FalconsActionFetchBall
        FalconsActionFetchBall::InputType subcomponent_input;
        //subcomponent_input.MergeFrom(input); // same type
        std::string tmpdata;
        input.SerializeToString(&tmpdata);
        subcomponent_input.ParseFromString(tmpdata);
        FalconsActionFetchBall::OutputType subcomponent_output;
        FalconsActionFetchBall::StateType subcomponent_state;
        FalconsActionFetchBall::DiagnosticsType subcomponent_diagnostics;
        error_value = FalconsActionFetchBall::FalconsActionFetchBall().tick(
            timestamp,
            subcomponent_input,
            params.fetchball(),
            subcomponent_state,
            subcomponent_output,
            subcomponent_diagnostics
        );
        output.set_actionresult(subcomponent_output.actionresult());
        if (subcomponent_output.has_target())
        {
            *output.mutable_target() = subcomponent_output.target();
        }
    }

    }
/*    else
    {
        // TODO: determine direction relative to robot
        error_value = FalconsActionGetBallIntercept().tick(
            timestamp,
            input,
            params.fetch,
            state,
            output,
            diagnostics
        );
    }
*/

    return error_value;
}

