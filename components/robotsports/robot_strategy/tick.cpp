// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "RobotsportsRobotStrategy.hpp"
using namespace MRA;
using namespace MRA::RobotsportsRobotStrategy;

// custom includes, if any
// ...

//---------------------------------------------------------------------------------------------------------------------

static void getFormation013(OutputType &output,
                            Input_GameState game_state,
                            Input_BallStatus ball_status)
{
    switch (game_state)
    {
    case Input_GameState_FREEKICK: // intentional fall through
    case Input_GameState_GOALKICK: // intentional fall through
    case Input_GameState_THROWIN: // intentional fall through
    case Input_GameState_CORNER: // intentional fall through
    case Input_GameState_KICKOFF:  // intentional fall through
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_ASSIST);
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
        output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
        output.add_dynamic_roles(MRA::Datatypes::DEFENDER_GENERIC);
        break;
    case Input_GameState_PENALTY: // intentional fall through
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
        output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
        output.add_dynamic_roles(MRA::Datatypes::DEFENDER_GENERIC);
        output.add_dynamic_roles(MRA::Datatypes::DEFENDER_GENERIC);
        break;
    case Input_GameState_NONE:   // intentional fall through
    case Input_GameState_NORMAL: // intentional fall through
        // different role assignment when ball is passed
        switch (ball_status)
        {
            case Input_BallStatus_OWNED_BY_PLAYER:
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_ASSIST);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_GENERIC);
                break;
            case Input_BallStatus_OWNED_BY_TEAM:
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_ASSIST);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_GENERIC);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_GENERIC);
                break;
            case Input_BallStatus_FREE:
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
                break;
            default:
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
        }
        break;
    case Input_GameState_PARKING: // intentional fall through
    case Input_GameState_BEGIN_POSITION: // intentional fall through
    case Input_GameState_KICKOFF_AGAINST: // intentional fall through
    case Input_GameState_FREEKICK_AGAINST:  // intentional fall through
    case Input_GameState_GOALKICK_AGAINST: // intentional fall through
    case Input_GameState_THROWIN_AGAINST: // intentional fall through
    case Input_GameState_CORNER_AGAINST: // intentional fall through
    case Input_GameState_PENALTY_AGAINST: // intentional fall through
    case Input_GameState_DROPPED_BALL: // intentional fall through
    case Input_GameState_PENALTY_SHOOTOUT_AGAINST: // should be handled at higher level
    case Input_GameState_PENALTY_SHOOTOUT: // should be handled at higher level
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
        output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
        output.add_dynamic_roles(MRA::Datatypes::DEFENDER_GENERIC);
        output.add_dynamic_roles(MRA::Datatypes::DEFENDER_GENERIC);
        break;
    default:
        ;// empty
    };
}


static void getFormation112(OutputType &output,
        Input_GameState game_state,
        Input_BallStatus ball_status,
        bool no_defender_main_during_setplay)
{
    switch (game_state)
    {
    case Input_GameState_FREEKICK: // intentional fall through
    case Input_GameState_GOALKICK: // intentional fall through
    case Input_GameState_THROWIN: // intentional fall through
    case Input_GameState_CORNER: // intentional fall through
    case Input_GameState_KICKOFF:  // intentional fall through
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_ASSIST);
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
        if (no_defender_main_during_setplay) {
            output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
        }
        else {
            output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
        }
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
        break;
    case Input_GameState_PENALTY: // intentional fall through
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
        output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
        output.add_dynamic_roles(MRA::Datatypes::DEFENDER_GENERIC);
        break;
    case Input_GameState_NONE:   // intentional fall through
    case Input_GameState_NORMAL: // intentional fall through
    case Input_GameState_PARKING: // intentional fall through
    case Input_GameState_BEGIN_POSITION: // intentional fall through
    case Input_GameState_KICKOFF_AGAINST: // intentional fall through
    case Input_GameState_FREEKICK_AGAINST:  // intentional fall through
    case Input_GameState_GOALKICK_AGAINST: // intentional fall through
    case Input_GameState_THROWIN_AGAINST: // intentional fall through
    case Input_GameState_CORNER_AGAINST: // intentional fall through
    case Input_GameState_PENALTY_AGAINST: // intentional fall through
    case Input_GameState_DROPPED_BALL: // intentional fall through
    case Input_GameState_PENALTY_SHOOTOUT_AGAINST: // should be handled at higher level
    case Input_GameState_PENALTY_SHOOTOUT: // should be handled at higher level
        switch (ball_status)
        {
            case Input_BallStatus_OWNED_BY_PLAYER:
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_GENERIC);
                break;
            case Input_BallStatus_OWNED_BY_TEAM:
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_GENERIC);
                break;
            case Input_BallStatus_FREE:
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_GENERIC);
                break;
            default:
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_GENERIC);
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
        }
        break;
    default:
        ;// empty
    };
}

static void getFormation211(OutputType &output,
        Input_GameState game_state,
        Input_BallStatus ball_status,
        bool no_defender_main_during_setplay)
{
    switch (game_state) {
    case Input_GameState_FREEKICK: // intentional fall through
    case Input_GameState_GOALKICK: // intentional fall through
    case Input_GameState_THROWIN: // intentional fall through
    case Input_GameState_CORNER: // intentional fall through
    case Input_GameState_KICKOFF:  // intentional fall through
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_ASSIST);
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
        if (no_defender_main_during_setplay) {
            output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
        }
        else {
            output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
        }
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
        break;
    case Input_GameState_PENALTY: // intentional fall through
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
        output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
        break;
    case Input_GameState_NONE:   // intentional fall through
    case Input_GameState_NORMAL: // intentional fall through
    case Input_GameState_PARKING: // intentional fall through
    case Input_GameState_BEGIN_POSITION: // intentional fall through
    case Input_GameState_KICKOFF_AGAINST: // intentional fall through
    case Input_GameState_FREEKICK_AGAINST:  // intentional fall through
    case Input_GameState_GOALKICK_AGAINST: // intentional fall through
    case Input_GameState_THROWIN_AGAINST: // intentional fall through
    case Input_GameState_CORNER_AGAINST: // intentional fall through
    case Input_GameState_PENALTY_AGAINST: // intentional fall through
    case Input_GameState_DROPPED_BALL: // intentional fall through
    case Input_GameState_PENALTY_SHOOTOUT_AGAINST: // should be handled at higher level
    case Input_GameState_PENALTY_SHOOTOUT: // should be handled at higher level
        switch (ball_status)
        {
            // create pass possibility if two players are left, only when we control the ball (see issue #218)
            case Input_BallStatus_OWNED_BY_PLAYER:
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
                break;
            case Input_BallStatus_OWNED_BY_TEAM:
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
                break;
            case Input_BallStatus_FREE:
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
                break;
            default:
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::DEFENDER_MAIN);
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
        }
        break;
    default:
        ;// empty
    };
}


static void getFormation310(OutputType &output,
        Input_GameState game_state,
        Input_BallStatus ball_status)
{
switch (game_state) {
    case Input_GameState_FREEKICK: // intentional fall through
    case Input_GameState_GOALKICK: // intentional fall through
    case Input_GameState_THROWIN: // intentional fall through
    case Input_GameState_CORNER: // intentional fall through
    case Input_GameState_KICKOFF:  // intentional fall through
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_ASSIST);
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
        break;
    case Input_GameState_PENALTY: // intentional fall through
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
        break;
    case Input_GameState_NONE:   // intentional fall through
    case Input_GameState_NORMAL: // intentional fall through
    case Input_GameState_PARKING: // intentional fall through
    case Input_GameState_BEGIN_POSITION: // intentional fall through
    case Input_GameState_KICKOFF_AGAINST: // intentional fall through
    case Input_GameState_FREEKICK_AGAINST:  // intentional fall through
    case Input_GameState_GOALKICK_AGAINST: // intentional fall through
    case Input_GameState_THROWIN_AGAINST: // intentional fall through
    case Input_GameState_CORNER_AGAINST: // intentional fall through
    case Input_GameState_PENALTY_AGAINST: // intentional fall through
    case Input_GameState_DROPPED_BALL: // intentional fall through
    case Input_GameState_PENALTY_SHOOTOUT_AGAINST: // should be handled at higher level
    case Input_GameState_PENALTY_SHOOTOUT: // should be handled at higher level
        switch (ball_status)
        {
            case Input_BallStatus_OWNED_BY_PLAYER:
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
                break;
            case Input_BallStatus_OWNED_BY_TEAM:
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
                break;
            default:
                output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN);
        }
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_GENERIC);
        break;
    default:
        ;// empty
    };
}

static void roleOnlyFormation(OutputType &output,
                              MRA::Datatypes::DynamicRole dynamic_role) {
    for (unsigned idx = 0; idx < 4; ++idx) {
        output.add_dynamic_roles(dynamic_role);
    }
}


int RobotsportsRobotStrategy::RobotsportsRobotStrategy::tick
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

    // user implementation goes here

    if (input.game_state() == Input_GameState_PENALTY_SHOOTOUT_AGAINST) {
        // formation strategy independent formation
        // all must be defender
        roleOnlyFormation(output, MRA::Datatypes::DEFENDER_GENERIC);
    }
    else if (input.game_state() == Input_GameState_PENALTY_SHOOTOUT) {
        // formation strategy independent formation
        output.add_dynamic_roles(MRA::Datatypes::ATTACKER_MAIN );
        output.add_dynamic_roles(MRA::Datatypes::DEFENDER_GENERIC );
        output.add_dynamic_roles(MRA::Datatypes::DEFENDER_GENERIC );
        output.add_dynamic_roles(MRA::Datatypes::DEFENDER_GENERIC );
    }
    else {
        Params_TeamFormation team_formation = params.attack_formation();
        std::cout << "set params.attack_formation()\n";

        if ( (input.game_state() == Input_GameState_NORMAL and
              (input.ball_status() == Input_BallStatus_FREE or input.ball_status() == Input_BallStatus_OWNED_BY_OPPONENT))
            || (input.game_state() == Input_GameState_KICKOFF_AGAINST)
            || (input.game_state() == Input_GameState_FREEKICK_AGAINST)
            || (input.game_state() == Input_GameState_GOALKICK_AGAINST)
            || (input.game_state() == Input_GameState_CORNER_AGAINST)
            || (input.game_state() == Input_GameState_PENALTY_AGAINST)
            || (input.game_state() == Input_GameState_PENALTY_SHOOTOUT_AGAINST)) {
            std::cout << "SELECTION params.defense_formation()\n";
            team_formation = params.defense_formation();
        }

        switch (team_formation)
        {
            case Params_TeamFormation_FORMATION_ONLY_GOALKEEPER:
                roleOnlyFormation(output, MRA::Datatypes::GOALKEEPER);
                break;
            case Params_TeamFormation_FORMATION_ONLY_ATTACKER_MAIN:
                roleOnlyFormation(output, MRA::Datatypes::ATTACKER_MAIN);
                break;
            case Params_TeamFormation_FORMATION_ONLY_ATTACKER_ASSIST:
                roleOnlyFormation(output, MRA::Datatypes::ATTACKER_ASSIST);
                break;
            case Params_TeamFormation_FORMATION_ONLY_ATTACKER_GENERIC:
                roleOnlyFormation(output, MRA::Datatypes::ATTACKER_GENERIC);
                break;
            case Params_TeamFormation_FORMATION_ONLY_DEFENDER_MAIN:
                roleOnlyFormation(output, MRA::Datatypes::DEFENDER_MAIN);
                break;
            case Params_TeamFormation_FORMATION_ONLY_DEFENDER_GENERIC:
                roleOnlyFormation(output, MRA::Datatypes::DEFENDER_GENERIC);
                break;

            case Params_TeamFormation_FORMATION_ONLY_DISABLED_OUT:
                roleOnlyFormation(output, MRA::Datatypes::DISABLED_OUT);
                break;
            case Params_TeamFormation_FORMATION_ONLY_DISABLED_IN:
                roleOnlyFormation(output, MRA::Datatypes::DISABLED_IN);
                break;
            break;
        case Params_TeamFormation_FORMATION_112:
            getFormation112(output, input.game_state(), input.ball_status(), params.no_defender_main_during_setplay() );
            break;
        case Params_TeamFormation_FORMATION_211:
            getFormation211(output, input.game_state(), input.ball_status(), params.no_defender_main_during_setplay());
            break;
        case Params_TeamFormation_FORMATION_013:
            getFormation013(output, input.game_state(),input.ball_status());
            break;
        case Params_TeamFormation_FORMATION_310:
            getFormation310(output, input.game_state(), input.ball_status());
            break;
        default:
            getFormation310(output, input.game_state(), input.ball_status());
        }
    }
    return error_value;
}

