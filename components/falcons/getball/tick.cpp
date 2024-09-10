// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "FalconsGetball.hpp"

// dependent (generated) component headers:
#include "FalconsGetballFetch.hpp"
#include "FalconsGetballIntercept.hpp"

using namespace MRA;

// custom includes, if any
#include <cmath>
#include "geometry.hpp"

// globals
FalconsGetballFetch::StateType g_fetch_state;
FalconsGetballIntercept::StateType g_intercept_state;


int FalconsGetball::FalconsGetball::tick
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
        // call component: FalconsGetballFetch
        FalconsGetballFetch::InputType subcomponent_input;
        //subcomponent_input.MergeFrom(input); // same type
        std::string tmpdata;
        input.SerializeToString(&tmpdata);
        subcomponent_input.ParseFromString(tmpdata);
        FalconsGetballFetch::OutputType subcomponent_output;
        FalconsGetballFetch::LocalType subcomponent_local;
        error_value = FalconsGetballFetch::FalconsGetballFetch().tick(
            timestamp,
            subcomponent_input,
            params.fetch(),
            g_fetch_state,
            subcomponent_output,
            diagnostics
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
        error_value = FalconsGetballIntercept().tick(
            timestamp,
            input,
            params.fetch,
            state,
            output,
            local
        );
    }
*/

    return error_value;
}

