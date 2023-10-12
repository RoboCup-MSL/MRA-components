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
		                    Input_GameState gamestate,
							bool playerControlBall,
							bool playerPassedBall)
{
	switch (gamestate)
	{
	case Input_GameState_FREEKICK: // intentional fall through
	case Input_GameState_GOALKICK: // intentional fall through
	case Input_GameState_THROWIN: // intentional fall through
	case Input_GameState_CORNER: // intentional fall through
	case Input_GameState_KICKOFF:  // intentional fall through
		output.add_dynamic_roles(Output_DynamicRole_SETPLAY_RECEIVER);
		output.add_dynamic_roles(Output_DynamicRole_SETPLAY_KICKER);
		output.add_dynamic_roles(Output_DynamicRole_SWEEPER);
		output.add_dynamic_roles(Output_DynamicRole_DEFENDER);
		break;
	case Input_GameState_PENALTY: // intentional fall through
		output.add_dynamic_roles(Output_DynamicRole_PENALTY_KICKER);
		output.add_dynamic_roles(Output_DynamicRole_SWEEPER);
		output.add_dynamic_roles(Output_DynamicRole_DEFENDER);
		output.add_dynamic_roles(Output_DynamicRole_DEFENDER);
		break;
	case Input_GameState_NONE:   // intentional fall through
	case Input_GameState_NORMAL: // intentional fall through
	case Input_GameState_NORMAL_ATTACK: // intentional fall through
	case Input_GameState_NORMAL_DEFEND: // intentional fall through
	case Input_GameState_YELLOW_CARD_AGAINST: // intentional fall through
	case Input_GameState_RED_CARD_AGAINST: // intentional fall through
	case Input_GameState_GOAL: // intentional fall through
	case Input_GameState_GOAL_AGAINST: // intentional fall through
		// different role assignment when ball is passed
		if (playerControlBall) {
			output.add_dynamic_roles(Output_DynamicRole_BALLPLAYER);
		}
		else if (playerPassedBall) { // ball is being passed
			output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		}
		else {
			output.add_dynamic_roles(Output_DynamicRole_INTERCEPTOR);
		}
		output.add_dynamic_roles(Output_DynamicRole_SWEEPER);
		output.add_dynamic_roles(Output_DynamicRole_DEFENDER);
		output.add_dynamic_roles(Output_DynamicRole_DEFENDER);
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
		if (playerControlBall) {
			output.add_dynamic_roles(Output_DynamicRole_BALLPLAYER);
		}
		else {
			output.add_dynamic_roles(Output_DynamicRole_INTERCEPTOR);
		}
		output.add_dynamic_roles(Output_DynamicRole_SWEEPER);
		output.add_dynamic_roles(Output_DynamicRole_DEFENDER);
		output.add_dynamic_roles(Output_DynamicRole_DEFENDER);
		break;
	default:
		;// empty
	};
}


static void getFormation112(OutputType &output,
        Input_GameState gamestate,
		bool playerControlBall,
		bool playerPassedBall,
		bool no_sweeper_during_setplay)
{
	switch (gamestate)
	{
	case Input_GameState_FREEKICK: // intentional fall through
	case Input_GameState_GOALKICK: // intentional fall through
	case Input_GameState_THROWIN: // intentional fall through
	case Input_GameState_CORNER: // intentional fall through
	case Input_GameState_KICKOFF:  // intentional fall through
		output.add_dynamic_roles(Output_DynamicRole_SETPLAY_RECEIVER);
		output.add_dynamic_roles(Output_DynamicRole_SETPLAY_KICKER);
		if (no_sweeper_during_setplay) {
			output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		}
		else {
			output.add_dynamic_roles(Output_DynamicRole_SWEEPER);
		}
		output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		break;
	case Input_GameState_PENALTY: // intentional fall through
		output.add_dynamic_roles(Output_DynamicRole_PENALTY_KICKER);
		output.add_dynamic_roles(Output_DynamicRole_SWEEPER);
		output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		output.add_dynamic_roles(Output_DynamicRole_DEFENDER);
		break;
	case Input_GameState_NONE:   // intentional fall through
	case Input_GameState_NORMAL: // intentional fall through
	case Input_GameState_NORMAL_ATTACK: // intentional fall through
	case Input_GameState_NORMAL_DEFEND: // intentional fall through
	case Input_GameState_PARKING: // intentional fall through
	case Input_GameState_BEGIN_POSITION: // intentional fall through
	case Input_GameState_KICKOFF_AGAINST: // intentional fall through
	case Input_GameState_FREEKICK_AGAINST:  // intentional fall through
	case Input_GameState_GOALKICK_AGAINST: // intentional fall through
	case Input_GameState_THROWIN_AGAINST: // intentional fall through
	case Input_GameState_CORNER_AGAINST: // intentional fall through
	case Input_GameState_PENALTY_AGAINST: // intentional fall through
	case Input_GameState_DROPPED_BALL: // intentional fall through
	case Input_GameState_YELLOW_CARD_AGAINST: // intentional fall through
	case Input_GameState_RED_CARD_AGAINST: // intentional fall through
	case Input_GameState_GOAL: // intentional fall through
	case Input_GameState_GOAL_AGAINST: // intentional fall through
	case Input_GameState_PENALTY_SHOOTOUT_AGAINST: // should be handled at higher level
	case Input_GameState_PENALTY_SHOOTOUT: // should be handled at higher level
		if (playerControlBall || playerPassedBall) {
			if (playerControlBall) {
				output.add_dynamic_roles(Output_DynamicRole_BALLPLAYER);
			}
			else if (playerPassedBall) { // ball is being passed
				output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
			}
			else {
				output.add_dynamic_roles(Output_DynamicRole_INTERCEPTOR);
			}
			// create pass possibility if two players are left, only when we control the ball (see issue #218)
			output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
			output.add_dynamic_roles(Output_DynamicRole_SWEEPER);
			output.add_dynamic_roles(Output_DynamicRole_DEFENDER);
		}
		else {
			// defending
			output.add_dynamic_roles(Output_DynamicRole_INTERCEPTOR);
			output.add_dynamic_roles(Output_DynamicRole_SWEEPER);
			output.add_dynamic_roles(Output_DynamicRole_DEFENDER);
			output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		}
		break;
	default:
		;// empty
	};
}

static void getFormation211(OutputType &output,
        Input_GameState gamestate,
		bool playerControlBall,
		bool playerPassedBall,
		bool no_sweeper_during_setplay)
{
	switch (gamestate) {
	case Input_GameState_FREEKICK: // intentional fall through
	case Input_GameState_GOALKICK: // intentional fall through
	case Input_GameState_THROWIN: // intentional fall through
	case Input_GameState_CORNER: // intentional fall through
	case Input_GameState_KICKOFF:  // intentional fall through
		output.add_dynamic_roles(Output_DynamicRole_SETPLAY_RECEIVER);
		output.add_dynamic_roles(Output_DynamicRole_SETPLAY_KICKER);
		if (no_sweeper_during_setplay) {
			output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		}
		else {
			output.add_dynamic_roles(Output_DynamicRole_SWEEPER);
		}
		output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		break;
	case Input_GameState_PENALTY: // intentional fall through
		output.add_dynamic_roles(Output_DynamicRole_PENALTY_KICKER);
		output.add_dynamic_roles(Output_DynamicRole_SWEEPER);
		output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		break;
	case Input_GameState_NONE:   // intentional fall through
	case Input_GameState_NORMAL: // intentional fall through
	case Input_GameState_NORMAL_ATTACK: // intentional fall through
	case Input_GameState_NORMAL_DEFEND: // intentional fall through
	case Input_GameState_PARKING: // intentional fall through
	case Input_GameState_BEGIN_POSITION: // intentional fall through
	case Input_GameState_KICKOFF_AGAINST: // intentional fall through
	case Input_GameState_FREEKICK_AGAINST:  // intentional fall through
	case Input_GameState_GOALKICK_AGAINST: // intentional fall through
	case Input_GameState_THROWIN_AGAINST: // intentional fall through
	case Input_GameState_CORNER_AGAINST: // intentional fall through
	case Input_GameState_PENALTY_AGAINST: // intentional fall through
	case Input_GameState_DROPPED_BALL: // intentional fall through
	case Input_GameState_YELLOW_CARD_AGAINST: // intentional fall through
	case Input_GameState_RED_CARD_AGAINST: // intentional fall through
	case Input_GameState_GOAL: // intentional fall through
	case Input_GameState_GOAL_AGAINST: // intentional fall through
	case Input_GameState_PENALTY_SHOOTOUT_AGAINST: // should be handled at higher level
	case Input_GameState_PENALTY_SHOOTOUT: // should be handled at higher level
		if (playerControlBall || playerPassedBall) {
			if (playerControlBall) {
				output.add_dynamic_roles(Output_DynamicRole_BALLPLAYER);
			}
			else if (playerPassedBall) { // ball is being passed
				output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
			}
			else {
				output.add_dynamic_roles(Output_DynamicRole_INTERCEPTOR);
			}
			// create pass possibility if two players are left, only when we control the ball (see issue #218)
			output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
			output.add_dynamic_roles(Output_DynamicRole_SWEEPER);
		}
		else {
			output.add_dynamic_roles(Output_DynamicRole_INTERCEPTOR);
			// create pass possibility if two players are left, only when we control the ball (see issue #218)
			output.add_dynamic_roles(Output_DynamicRole_SWEEPER);
			output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		}
		output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		break;
	default:
		;// empty
	};
}


static void getFormation310(OutputType &output,
        Input_GameState gamestate,
		bool playerControlBall,
		bool playerPassedBall)
{
switch (gamestate) {
	case Input_GameState_FREEKICK: // intentional fall through
	case Input_GameState_GOALKICK: // intentional fall through
	case Input_GameState_THROWIN: // intentional fall through
	case Input_GameState_CORNER: // intentional fall through
	case Input_GameState_KICKOFF:  // intentional fall through
		output.add_dynamic_roles(Output_DynamicRole_SETPLAY_RECEIVER);
		output.add_dynamic_roles(Output_DynamicRole_SETPLAY_KICKER);
		output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		break;
	case Input_GameState_PENALTY: // intentional fall through
		output.add_dynamic_roles(Output_DynamicRole_PENALTY_KICKER);
		output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		break;
	case Input_GameState_NONE:   // intentional fall through
	case Input_GameState_NORMAL: // intentional fall through
	case Input_GameState_NORMAL_ATTACK: // intentional fall through
	case Input_GameState_NORMAL_DEFEND: // intentional fall through
	case Input_GameState_PARKING: // intentional fall through
	case Input_GameState_BEGIN_POSITION: // intentional fall through
	case Input_GameState_KICKOFF_AGAINST: // intentional fall through
	case Input_GameState_FREEKICK_AGAINST:  // intentional fall through
	case Input_GameState_GOALKICK_AGAINST: // intentional fall through
	case Input_GameState_THROWIN_AGAINST: // intentional fall through
	case Input_GameState_CORNER_AGAINST: // intentional fall through
	case Input_GameState_PENALTY_AGAINST: // intentional fall through
	case Input_GameState_DROPPED_BALL: // intentional fall through
	case Input_GameState_YELLOW_CARD_AGAINST: // intentional fall through
	case Input_GameState_RED_CARD_AGAINST: // intentional fall through
	case Input_GameState_GOAL: // intentional fall through
	case Input_GameState_GOAL_AGAINST: // intentional fall through
	case Input_GameState_PENALTY_SHOOTOUT_AGAINST: // should be handled at higher level
	case Input_GameState_PENALTY_SHOOTOUT: // should be handled at higher level
		if (playerControlBall) {
			output.add_dynamic_roles(Output_DynamicRole_BALLPLAYER);
		}
		else if (playerPassedBall) { // ball is being passed
			output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		}
		else {
			output.add_dynamic_roles(Output_DynamicRole_INTERCEPTOR);
		}
		output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		output.add_dynamic_roles(Output_DynamicRole_ATTACKSUPPORTER);
		break;
	default:
		;// empty
	};
}



//void TeamPlay::assign(team_planner_result_t* player_paths, game_state_e gamestate,
//		const MovingObject& globalBall, const MovingObject& localBall, const previous_used_ball_by_planner_t& previous_global_ball,
//		std::vector<TeamPlannerRobot>& Team, std::vector<TeamPlannerOpponent>& Opponents,
//		const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig, const std::vector<Vector2D>& parking_positions,
//		const ball_pickup_position_t& ball_pickup_position, bool passIsRequired, const pass_data_t& pass_data)
//{
//	game_state_e org_gamestate = gamestate;
//	if (gamestate != game_state_e::NONE) {
//	}
//	if ((gamestate == game_state_e::NONE) ||
//			(gamestate == game_state_e::YELLOW_CARD_AGAINST) ||
//			(gamestate == game_state_e::RED_CARD_AGAINST) ||
//			(gamestate == game_state_e::GOAL) ||
//			(gamestate == game_state_e::GOAL_AGAINST)) {
//		// unhandled game situations: cards and goals are not passed to the team planner via the game state-machine
//		return; // no path will be planned if game state is NONE
//	}
//	bool playerControlBall = false;
//	bool playerPassedBall = false;
//	for (unsigned r_idx = 0; r_idx < Team.size(); r_idx++) {
//		if (Team[r_idx].controlBall) {
//			playerControlBall = true;
//		}
//		if (Team[r_idx].passBall) {
//			playerPassedBall = true;
//		}
//	}
//	bool teamControlBall = playerPassedBall || playerControlBall;
//
//	if (gamestate == game_state_e::NORMAL) {
//		if (teamControlBall) {
//			gamestate = game_state_e::NORMAL_ATTACK;
//		}
//		else{
//			gamestate = game_state_e::NORMAL_DEFEND;
//		}
//	}
//
//	team_formation_e formationToUse = plannerOptions.attack_formation;
//
//	if ((gamestate == NORMAL_DEFEND)
//			|| (gamestate == KICKOFF_AGAINST) || (gamestate == FREEKICK_AGAINST) || (gamestate == GOALKICK_AGAINST)
//			|| (gamestate == CORNER_AGAINST) || (gamestate == PENALTY_AGAINST) || (gamestate == PENALTY_SHOOTOUT_AGAINST)) {
//		formationToUse = plannerOptions.defense_formation;
//	}
//	vector<dynamic_role_e> teamFormation = TeamFormation::selectTeamFormation(formationToUse, gamestate,
//			playerControlBall, playerPassedBall, plannerOptions)

static void roleOnlyFormation(OutputType &output,
		                      Output_DynamicRole dynamic_role) {
	for (unsigned idx = 0; idx < 4; ++idx) {
		output.add_dynamic_roles(dynamic_role);
	}
}



int RobotsportsRobotStrategy::RobotsportsRobotStrategy::tick
(
	google::protobuf::Timestamp            timestamp,   // simulation timestamp, seconds since start of simulation
    InputType  const &input,       // input data, type generated from Input.proto
    ParamsType const &params,      // configuration parameters, type generated from Params.proto
    StateType        &state,       // state data, type generated from State.proto
    OutputType       &output,      // output data, type generated from Output.proto
    LocalType        &local        // local/diagnostics data, type generated from Local.proto
)
{
    int error_value = 0;

    // user implementation goes here

    //vector<dynamic_role_e> TeamFormation::selectTeamFormation(team_formation_e team_formation, game_state_e gamestate,
    //		bool playerControlBall, bool playerPassedBall, const PlannerOptions& plannerOptions) {
    //	vector<dynamic_role_e> formation = vector<dynamic_role_e>();
    //	// inputs which can be used:
    //	//	m_plannerOptions;
    //	//	m_ballControlledByTeam
    //	//	m_ball_pickup_position;
    //	// 	//	Game-stats (score)   [Future]
	bool playerControlBall = false;
	bool playerPassedBall = false;


    if (input.gamestate() == Input_GameState_PENALTY_SHOOTOUT_AGAINST) {
    	// formation strategy independent formation
    	// all must be defender
		roleOnlyFormation(output, Output_DynamicRole_PENALTY_DEFENDER);
    }
    else if (input.gamestate() == Input_GameState_PENALTY_SHOOTOUT) {
    	// formation strategy independent formation
    	output.add_dynamic_roles(Output_DynamicRole_PENALTY_KICKER );
    	output.add_dynamic_roles(Output_DynamicRole_PENALTY_DEFENDER );
    	output.add_dynamic_roles(Output_DynamicRole_PENALTY_DEFENDER );
    	output.add_dynamic_roles(Output_DynamicRole_PENALTY_DEFENDER );
    }
    else {
    	Params_TeamFormation team_formation = params.attack_formation();

    	if ((input.gamestate() == Input_GameState_NORMAL_DEFEND)
   			|| (input.gamestate() == Input_GameState_KICKOFF_AGAINST)
			|| (input.gamestate() == Input_GameState_FREEKICK_AGAINST)
			|| (input.gamestate() == Input_GameState_GOALKICK_AGAINST)
			|| (input.gamestate() == Input_GameState_CORNER_AGAINST)
			|| (input.gamestate() == Input_GameState_PENALTY_AGAINST)
			|| (input.gamestate() == Input_GameState_PENALTY_SHOOTOUT_AGAINST)) {
    		team_formation = params.defense_formation();
    	}

    	switch (team_formation)
    	{
    	case Params_TeamFormation_FORMATION_ATTACK_SUPPORT_ONLY:
    		roleOnlyFormation(output, Output_DynamicRole_ATTACKSUPPORTER);
    		break;
    	case Params_TeamFormation_FORMATION_DEFENDER_ONLY:
    		roleOnlyFormation(output, Output_DynamicRole_DEFENDER);
    		break;
    	case Params_TeamFormation_FORMATION_INTERCEPTOR_ONLY:
    		roleOnlyFormation(output, Output_DynamicRole_INTERCEPTOR);
    		break;
    	case Params_TeamFormation_FORMATION_SWEEPER_ONLY:
    		roleOnlyFormation(output, Output_DynamicRole_SWEEPER);
    		break;
    	case Params_TeamFormation_FORMATION_SETPLAY_RECEIVER_ONLY:
    		roleOnlyFormation(output, Output_DynamicRole_SETPLAY_RECEIVER);
    		break;
    	case Params_TeamFormation_FORMATION_SETPLAY_KICKER_ONLY:
    		roleOnlyFormation(output, Output_DynamicRole_SETPLAY_KICKER);
    		break;
    	case Params_TeamFormation_FORMATION_BALLPLAYER_ONLY:
    		roleOnlyFormation(output, Output_DynamicRole_BALLPLAYER);
    		break;
    	case Params_TeamFormation_FORMATION_SEARCHFORBALL_ONLY:
    		roleOnlyFormation(output, Output_DynamicRole_SEARCH_FOR_BALL);
    		break;
    	case Params_TeamFormation_FORMATION_BEGINPOSITION_ONLY:
    		roleOnlyFormation(output, Output_DynamicRole_BEGIN_POSITION);
    		break;
    	case Params_TeamFormation_FORMATION_PARKING_ONLY:
    		roleOnlyFormation(output, Output_DynamicRole_PARKING);
    		break;
    	case Params_TeamFormation_FORMATION_PENALTYKICKER_ONLY:
    		roleOnlyFormation(output, Output_DynamicRole_PENALTY_KICKER);
    		break;
    	case Params_TeamFormation_FORMATION_PENALTY_SHOOTOUT:
    		roleOnlyFormation(output, Output_DynamicRole_PENALTY_DEFENDER);
    		break;
    	case Params_TeamFormation_FORMATION_112:
    		getFormation112(output, input.gamestate(), playerControlBall, playerPassedBall, params.no_sweeper_during_setplay() );
    		break;
    	case Params_TeamFormation_FORMATION_211:
    		getFormation211(output, input.gamestate(), playerControlBall, playerPassedBall, params.no_sweeper_during_setplay());
    		break;
    	case Params_TeamFormation_FORMATION_013:
    		getFormation013(output, input.gamestate(), playerControlBall, playerPassedBall);
    		break;
    	case Params_TeamFormation_FORMATION_310:
    		getFormation310(output, input.gamestate(), playerControlBall, playerPassedBall);
    	default:
    		getFormation310(output, input.gamestate(), playerControlBall, playerPassedBall);
    	}
    }
    return error_value;
}

