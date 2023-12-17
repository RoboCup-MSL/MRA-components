// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "RobotsportsRoleAssigner.hpp"

using namespace MRA;

// custom includes, if any
// ...
#include "TeamPlay.hpp"

int RobotsportsRoleAssigner::RobotsportsRoleAssigner::tick
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

    // user implementation goes here
    trs::TeamPlannerInput tp_input = trs::TeamPlannerInput();
    trs::TeamPlannerState tp_state;
    trs::TeamPlannerOutput tp__output;
    trs::PlannerOptions tp_params;


    trs::TeamPlay team_play = trs::TeamPlay();
    team_play.assign(tp_input, tp_state, tp__output, tp_params);

    return error_value;
}

