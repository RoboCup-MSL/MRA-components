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
    MRA::TeamPlannerInput tp_input = MRA::TeamPlannerInput();
    MRA::TeamPlannerState tp_state;
    MRA::TeamPlannerOutput tp__output;
    MRA::TeamPlannerParameters tp_params;

    if (tp_input.gamestate == MRA::game_state_e::NORMAL) {
        bool playerControlBall = false;
        bool playerPassedBall = false;
        for (unsigned r_idx = 0; r_idx < tp_input.team.size(); r_idx++) {
            if (tp_input.team[r_idx].controlBall) {
                playerControlBall = true;
            }
            if (tp_input.team[r_idx].passBall) {
                tp_input.playerPassedBall = true;
            }
        }
        tp_input.teamControlBall = playerPassedBall || playerControlBall;
        if (tp_input.teamControlBall) {
            tp_input.gamestate = MRA::game_state_e::NORMAL_ATTACK;
        }
        else{
            tp_input.gamestate = MRA::game_state_e::NORMAL_DEFEND;
        }
    }

    MRA::TeamPlay team_play = MRA::TeamPlay();
    team_play.assign(tp_input, tp_state, tp__output, tp_params);

    return error_value;
}

