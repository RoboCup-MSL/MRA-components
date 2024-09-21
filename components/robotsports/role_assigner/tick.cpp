// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "RobotsportsRoleAssigner.hpp"

using namespace MRA;

// custom includes, if any
// ...
#include "internals/RoleAssigner.hpp"

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
//    MRA::RoleAssignerInput tp_input = MRA::RoleAssignerInput();
//    MRA::RoleAssignerState tp_state;
//    MRA::RoleAssignerOutput tp_output;
//    MRA::RoleAssignerParameters tp_params;
//
//    if (tp_input.gamestate == MRA::game_state_e::NORMAL) {
//        bool playerControlBall = false;
//        bool playerPassedBall = false;
//        for (unsigned r_idx = 0; r_idx < tp_input.team.size(); r_idx++) {
//            if (tp_input.team[r_idx].controlBall) {
//                playerControlBall = true;
//            }
//        }
//        tp_input.teamControlBall = playerPassedBall || playerControlBall;
//        if (tp_input.teamControlBall) {
//            tp_input.gamestate = MRA::game_state_e::NORMAL_ATTACK;
//        }
//        else{
//            tp_input.gamestate = MRA::game_state_e::NORMAL_DEFEND;
//        }
//    }
//
//    MRA::TeamPlay team_play = MRA::TeamPlay();
//    void TeamPlay::assign(const RoleAssignerInput& input,
//                RoleAssignerState& r_state,
//                RoleAssignerOutput& r_output,
//                const RoleAssignerParameters& parameters) {
//
//    team_play.assign(tp_input, tp_state, tp_output, tp_params);

    return error_value;
}

