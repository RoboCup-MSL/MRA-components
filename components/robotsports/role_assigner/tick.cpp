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
    trs::FieldConfig fieldConfig = trs::FillDefaultFieldConfig();
    trs::TeamPlannerData teamplannerData;
    vector<dynamic_role_e> teamFormation;

    trs::TeamPlay team_play = trs::TeamPlay();
    team_play.assign(teamFormation, fieldConfig, teamplannerData);

    return error_value;
}

