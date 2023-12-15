/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */
#include "TeamPlay.hpp"

#include <iomanip>
#include <iostream>
#include <limits>
#include <cmath>
#include <climits>
#include <algorithm>
#include "GlobalPathDynamicPlanner.hpp"
#include "RolePosition.hpp"
#include "SvgUtils.hpp"
#include "TeamFormation.hpp"
#include "TeamPlannerExport.hpp"

using namespace std;
using namespace trs;

//---------------------------------------------------------------------------------------------------------------------
TeamPlay::TeamPlay() : m_gridFileNumber(0) {

}

//---------------------------------------------------------------------------------------------------------------------
/*
 * Main method for dynamic role assignment.
 * Assign roles to team members based on game-situation and location of the players and the ball on the field.
 *
 * Internally the method will based on the game-situation call an other internal method which handles the specific game situation.
 */

void TeamPlay::assign(const FieldConfig& fieldConfig, TeamPlannerData& teamplannerData)
{
	team_planner_result_t* player_paths = teamplannerData.player_paths;
	game_state_e gamestate = teamplannerData.gamestate;
	MovingObject globalBall = teamplannerData.globalBall;
	MovingObject localBall  = teamplannerData.localBall;
	previous_used_ball_by_planner_t previous_global_ball = teamplannerData.previous_global_ball;
	std::vector<TeamPlannerRobot> Team =  teamplannerData.Team;
	std::vector<TeamPlannerOpponent> Opponents = teamplannerData.Opponents;
	PlannerOptions plannerOptions = teamplannerData.plannerOptions;
	std::vector<Vector2D> parking_positions = teamplannerData.parking_positions;
	ball_pickup_position_t ball_pickup_position = teamplannerData.ball_pickup_position;
	bool passIsRequired = teamplannerData.passIsRequired;
	pass_data_t pass_data = teamplannerData.pass_data;
	assign(player_paths, gamestate, globalBall, localBall, previous_global_ball,
		Team, Opponents, plannerOptions, fieldConfig, parking_positions, ball_pickup_position, passIsRequired, pass_data);
}

void TeamPlay::assign(team_planner_result_t* player_paths, game_state_e gamestate,
		const MovingObject& globalBall, const MovingObject& localBall, const previous_used_ball_by_planner_t& previous_global_ball,
		std::vector<TeamPlannerRobot>& Team, std::vector<TeamPlannerOpponent>& Opponents,
		const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig, const std::vector<Vector2D>& parking_positions,
		const ball_pickup_position_t& ball_pickup_position, bool passIsRequired, const pass_data_t& pass_data)
{
	game_state_e org_gamestate = gamestate;
	if (gamestate != game_state_e::NONE) {
		// printAssignInputs(gamestate, globalBall, localBall, Team, Opponents, plannerOptions,  parking_positions, ball_pickup_position, passIsRequired, pass_data);
	}
	if ((gamestate == game_state_e::NONE) ||
			(gamestate == game_state_e::YELLOW_CARD_AGAINST) ||
			(gamestate == game_state_e::RED_CARD_AGAINST) ||
			(gamestate == game_state_e::GOAL) ||
			(gamestate == game_state_e::GOAL_AGAINST)) {
		// unhandled game situations: cards and goals are not passed to the team planner via the game state-machine
		for (unsigned idx = 0; idx < Team.size(); idx++) {
			(*player_paths)[idx].path = std::vector<planner_piece_t>();
			(*player_paths)[idx].gamestate = gamestate;
			(*player_paths)[idx].dynamic_role = dynamic_role_e::dr_NONE;
			(*player_paths)[idx].defend_info.valid = false;
		}
		return; // no path will be planned if game state is NONE
	}
	bool playerControlBall = false;
	bool playerPassedBall = false;
	for (unsigned r_idx = 0; r_idx < Team.size(); r_idx++) {
		if (Team[r_idx].controlBall) {
			playerControlBall = true;
		}
		if (Team[r_idx].passBall) {
			playerPassedBall = true;
		}
	}
	bool teamControlBall = playerPassedBall || playerControlBall;

	if (gamestate == game_state_e::NORMAL) {
		if (teamControlBall) {
			gamestate = game_state_e::NORMAL_ATTACK;
		}
		else{
			gamestate = game_state_e::NORMAL_DEFEND;
		}
	}

	for (unsigned r_idx = 0; r_idx < Team.size(); r_idx++) {
		Team[r_idx].result.gamestate = gamestate;
		Team[r_idx].result.dynamic_role = dynamic_role_e::dr_NONE;
		Team[r_idx].result.defend_info.valid = false;

	}


	/* set avoid ball flag : only not avoiding during normal play */
	bool ballIsObstacle = (gamestate != game_state_e::NORMAL_ATTACK) && (gamestate != game_state_e::NORMAL_DEFEND);
	team_formation_e formationToUse = plannerOptions.attack_formation;

	if ((gamestate == NORMAL_DEFEND)
			|| (gamestate == KICKOFF_AGAINST) || (gamestate == FREEKICK_AGAINST) || (gamestate == GOALKICK_AGAINST)
			|| (gamestate == CORNER_AGAINST) || (gamestate == PENALTY_AGAINST) || (gamestate == PENALTY_SHOOTOUT_AGAINST)) {
		formationToUse = plannerOptions.defense_formation;
	}
	vector<dynamic_role_e> teamFormation = TeamFormation::selectTeamFormation(formationToUse, gamestate,
			playerControlBall, playerPassedBall, plannerOptions);

	// Assign first the Goalie
	assignGoalie(gamestate, Team, ballIsObstacle, globalBall, Opponents, plannerOptions, fieldConfig, parking_positions);

	// Assign players too long in any penalty area: closest position out of the penalty area
	assignTooLongInPenaltyAreaPlayers(gamestate, Team, ballIsObstacle, globalBall, Opponents, plannerOptions, fieldConfig);


	bool searchForBall = searchForBallBehaviorNeeded(gamestate, globalBall, fieldConfig);
	for (unsigned dr_idx = 0; dr_idx < teamFormation.size(); dr_idx++) {
		if (dr_idx >= Team.size()) {
			// can not assign more roles than team members.
			break;
		}

		planner_target_e planner_target = determine_planner_target(teamFormation[dr_idx], gamestate);
		defend_info_t Defend_info = {0};

		bool role_position_is_end_position_of_pass = false;
		Vector2D rolePosition = RolePosition::determineDynamicRolePosition(Defend_info, planner_target, m_gridFileNumber, teamFormation[dr_idx], gamestate,
				globalBall, localBall, previous_global_ball, Team, Opponents, plannerOptions, fieldConfig, ball_pickup_position,
				passIsRequired, teamControlBall, playerPassedBall, pass_data, role_position_is_end_position_of_pass);
		if (  searchForBall || gamestate == game_state_e::BEGIN_POSITION || gamestate ==  game_state_e::PARKING
				|| gamestate ==  game_state_e::KICKOFF || gamestate ==  game_state_e::KICKOFF_AGAINST)
		{
			// Fixed position assignment
			assignToFixedPositions(dr_idx, teamFormation[dr_idx], gamestate, Team, Opponents, globalBall, ballIsObstacle,
					parking_positions, plannerOptions, fieldConfig, searchForBall, Defend_info, pass_data);
		}
		else {
			//	Determine path for assigned role
			assignAnyToPosition(static_cast<int>(dr_idx), teamFormation[dr_idx], gamestate, globalBall,
					Team, Opponents, rolePosition, ballIsObstacle,  planner_target, plannerOptions, fieldConfig, Defend_info,
					role_position_is_end_position_of_pass, pass_data);
		}
		// stop if current player has ball
		if (Team[0].assigned && plannerOptions.calculateAllPaths == false) {
			// path for this robot (player nr: 0) is found and not all paths must be calculated.
			break;
		}
		//cerr << dr_idx <<": DR: " << DynamicRoleAsString(teamFormation[dr_idx]) << " pos: " << rolePosition.toString() << endl;
	}

	// ASSIGN ALWAYS A ROLE TO THE AVAIALABLE PLAYERS
	if (Team[0].assigned == false || plannerOptions.calculateAllPaths)
	{
		// current player does not have any role or all players must be assigned.
		// assign defender role to all unassigned players
		for (unsigned ap_idx = 0; ap_idx < Team.size(); ap_idx++) {
			if (Team[ap_idx].assigned == false)  {
				//	Determine path for assigned role
				planner_target_e planner_target = determine_planner_target(dynamic_role_e::dr_DEFENDER, gamestate);
				defend_info_t Defend_info;
				bool role_position_is_end_position_of_pass = false;
				Vector2D rolePosition = RolePosition::determineDynamicRolePosition(Defend_info, planner_target, m_gridFileNumber, dynamic_role_e::dr_DEFENDER, gamestate,
						globalBall, localBall, previous_global_ball, Team, Opponents, plannerOptions, fieldConfig, ball_pickup_position, passIsRequired, teamControlBall, playerPassedBall, pass_data, role_position_is_end_position_of_pass);
				assignAnyToPosition(static_cast<int>(ap_idx), dynamic_role_e::dr_DEFENDER, gamestate, globalBall,
						Team, Opponents, rolePosition, ballIsObstacle,  planner_target, plannerOptions, fieldConfig,
						Defend_info, role_position_is_end_position_of_pass, pass_data);
			}
		}
	}

	bool avoidBallPath = pass_data.valid;
	Vector2D BallTargePos = Vector2D(pass_data.target_pos.m_x, pass_data.target_pos.m_y);
	// Calculate Robot Planner path
	for (unsigned idx = 0; idx < Team.size(); idx++) {
		if (Team[idx].assigned == true)  {
			// calculate path for robot.

			avoidBallPath = pass_data.valid; // avoid bal path only if pass made (or shot on goal)
			if (Team[idx].result.target_position_is_end_position_of_pass)
			{
				// in case of pass, don't avoid ball path if player is destination of the pass
				avoidBallPath = false;
			}

			vector<MovingObject> myTeam = getTeamMates(Team, idx, true);
			GlobalPathPlanner visibilityGraph = GlobalPathPlanner(fieldConfig); // create robot planner
			visibilityGraph.setOptions(plannerOptions);
			// create list of possible targets for robot-planner
			std::vector<trs::Vertex> targetPos = vector<trs::Vertex>();
			targetPos.push_back(Vertex(Team[idx].result.target, 0));
			visibilityGraph.createGraph(Team[idx].position, globalBall, myTeam, getOpponents(Opponents),
					targetPos, Team[idx].result.planner_target, ballIsObstacle, avoidBallPath, BallTargePos);
			Team[idx].result.path = visibilityGraph.getShortestPath();
		}
		if (plannerOptions.calculateAllPaths == false) {
			// path for this robot (player nr: 0) is found and not all paths must be calculated.
			break;
		}
	}

	if (gamestate == NORMAL_DEFEND) {
		// replan interceptor with local ball only if interceptor is not performing a priority block
		int interceptorIdx = -1;
		for (unsigned idx = 0; idx < Team.size(); idx++) {
			if (Team[idx].assigned == true)  {
				if (Team[idx].dynamic_role == dr_INTERCEPTOR) {
					interceptorIdx = idx;
				}

			}
		}

		bool replan = false;
		if ((interceptorIdx == 0) || plannerOptions.calculateAllPaths) {
			replan = (interceptorIdx != -1) && (Team[interceptorIdx].result.path.size() > 0) && (Team[interceptorIdx].result.path[0].target != PRIORITY_BLOCK);
		}
		if (replan && interceptorIdx >= 0) {
			ReplanInterceptorWithLocalBall(localBall, interceptorIdx, Team, Opponents, plannerOptions, fieldConfig, ballIsObstacle);
		}
	}

	// loop over the player path to verify if path of this player stay within the allowed boundaries (field+safety area)
	bool thisPlayerHasUnallowedPath = false;
	bool thisPlayerStartsAtUnallowedPosition = false;
	for (unsigned idx = 0; idx < Team.size(); idx++) {
		if (Team[idx].assigned == true) {
			bool pathOK = stayPathWithinBoundaries(fieldConfig, Team[idx].result);
			(*player_paths)[idx] = Team[idx].result;
			(*player_paths)[idx].gamestate = Team[idx].result.gamestate;
			(*player_paths)[idx].dynamic_role = Team[idx].dynamic_role;
			if (!pathOK && idx == 0) { // CHECK Only this robot
				thisPlayerHasUnallowedPath = true; // this robot has wrong path

				// check if player starts at position in the field
				auto currentPos = Team[idx].position.getPosition().getVector2D();
				thisPlayerStartsAtUnallowedPosition = !fieldConfig.isInReachableField(currentPos.m_x, currentPos.m_y);
			}
		}
	}

	bool dynamicRoleNoneAssigned =  Team[0].dynamic_role == dr_NONE;
	// Dump diagnostics if any robot moves outside the allowed field (safety border + field)
	if (thisPlayerHasUnallowedPath || plannerOptions.svgOutputFileName.length() > 0 || dynamicRoleNoneAssigned) {
		// Create diagnostics file (.svg with comments).
		// file can be used to diagnose with path to illegal locations are planned
		long controlBallByPlayer = -1;
		std::vector<player_type_e> teamTypes = std::vector<player_type_e>();
		std::vector<long> robotIds = std::vector<long>();

		for (unsigned r_idx = 0; r_idx < Team.size(); r_idx++) {
			if(Team[r_idx].controlBall) {
				controlBallByPlayer = static_cast<int>(r_idx);
			}
			robotIds.push_back(Team[r_idx].robotId);
			teamTypes.push_back(Team[r_idx].player_type);
		}
		PlannerOptions options = plannerOptions;
		if (thisPlayerHasUnallowedPath) {
			if (thisPlayerStartsAtUnallowedPosition)
				options.svgOutputFileName = GetTeamPlannerSVGname(gamestate, "OUTSIDE_FIELD_BEGIN_ERROR");
			else {
				options.svgOutputFileName = GetTeamPlannerSVGname(gamestate, "OUTSIDE_FIELD_END_ERROR");
			}
		}
		if (dynamicRoleNoneAssigned)
		{
			options.svgOutputFileName = GetTeamPlannerSVGname(gamestate, "DYN_ROLE_NONE");
		}

		bool hasTeamPlannerInputInfo = true;
		class TeamPlannerInputInfo  inputInfo;
		inputInfo.playerWhoIsPassing = -1;
		for (unsigned r_idx = 0; r_idx < Team.size(); r_idx++) {
			if (Team[r_idx].passBall) {
				inputInfo.playerWhoIsPassing = r_idx;
			}
			inputInfo.previous_results.push_back(Team[r_idx].previous_result);
		}
		inputInfo.ball_pickup_position = ball_pickup_position;
		inputInfo.passIsRequired = passIsRequired;
		inputInfo.pass_data = pass_data;


		SvgUtils::save_graph_as_svg(globalBall, localBall, getTeamMates(Team, 9999 /* non existing id*/, false /* no add target positions */),
				TeamPlay::getOpponents(Opponents), (*player_paths), options, std::vector<Vertex*>(),
				org_gamestate, controlBallByPlayer, teamTypes, robotIds, "red", fieldConfig, hasTeamPlannerInputInfo, inputInfo);

		// create empty path for robot with wrong path
		if (thisPlayerHasUnallowedPath) {
			std::vector<planner_piece_t> path = (*player_paths)[0].path;
			if (!path.empty()) {
				planner_piece_t last_piece = path[path.size()-1];
				if (fieldConfig.isInReachableField(last_piece.x,last_piece.y) == false) {
					(*player_paths)[0] = PlayerPlannerResult();
				}
			}
		}
	}

	//printAssignOutputs(Team, *player_paths);
	return;
}

//---------------------------------------------------------------------------------------------------------------------
// check if the given path is always within the boundaries
bool TeamPlay::stayPathWithinBoundaries(const FieldConfig& fieldConfig, const PlayerPlannerResult& player_path) {
	bool ok = true;
	// check path-pieces
	for (auto it = player_path.path.begin(); it != player_path.path.end(); ++it ) {
		// abs(x) < allowed-x
		if (!fieldConfig.isInReachableField(it->x, it->y)) {
			ok = false;
		}
	}
	return ok;
}
//---------------------------------------------------------------------------------------------------------------------
// Print the output of TeamPlanner::Assign (debug purposes)
void TeamPlay::printAssignOutputs(const std::vector<TeamPlannerRobot>& Team, team_planner_result_t&  player_paths)
{
	//Print final position
	for (unsigned int player_idx = 0; player_idx < player_paths.size(); player_idx++) {
		PlayerPlannerResult playerResult = player_paths[player_idx];
		cerr << "[" << player_idx  << "] Player-id: " << Team[player_idx].robotId << DynamicRoleAsString(Team[player_idx].dynamic_role) << endl;
		if (playerResult.path.size() > 0) {
			cerr << "playerResult [ " << player_idx << "]  = (" << playerResult.path[playerResult.path.size()-1].x << ", "<< playerResult.path[playerResult.path.size()-1].y << ")" << endl << flush;
		}
		else {
			cerr << "playerResult [ " << player_idx << "]  = (<empty>)" << endl << flush;
		}
	}
}

planner_target_e TeamPlay::determine_planner_target(dynamic_role_e dynamic_role, game_state_e gamestate) {
	planner_target_e target = planner_target_e::GOTO_TARGET_POSITION;
	switch (dynamic_role) {
	case dr_DEFENDER:
		target = planner_target_e::SUPPORT_DEFENSE;
		break;
	case dr_ATTACKSUPPORTER:
		target = planner_target_e::SUPPORT_ATTACK;
		break;
	case dr_SETPLAY_KICKER:
		target = planner_target_e::PREPARE_RESTART;
		break;
	case dr_SWEEPER:
		target = planner_target_e::SWEEPER;
		break;
	case dr_SETPLAY_RECEIVER:
		target = planner_target_e::PREPARE_RESTART;
		break;
	case dr_INTERCEPTOR:
		target = planner_target_e::GOTO_BALL;
		break;
	case dr_BALLPLAYER:
		target = planner_target_e::DRIBBLE;
		break;
	case dr_GOALKEEPER:
		target = planner_target_e::GOALIE;
		break;
	case dr_SEARCH_FOR_BALL:
		target = planner_target_e::GOTO_TARGET_POSITION;
		break;
	case dr_BEGIN_POSITION:
		target = planner_target_e::GOTO_TARGET_POSITION;
		break;
	case dr_PARKING:
		target = planner_target_e::GOTO_TARGET_POSITION_SLOW;
		break;
	case dr_PENALTY_KICKER:
		target = planner_target_e::PREPARE_RESTART;
		break;
	case dr_PENALTY_DEFENDER:
		target = planner_target_e::GOTO_TARGET_POSITION;
		break;
	case dr_LOB_CALIBRATION:
	case dr_NONE:
		// empty
		break;
	}
	return target;
}
//---------------------------------------------------------------------------------------------------------------------
vector<MovingObject> TeamPlay::getOpponents(const std::vector<TeamPlannerOpponent>&  Opponents) {
	vector<MovingObject> opponents = vector<MovingObject>();
	for (unsigned int idx = 0; idx < Opponents.size(); idx++) {
		opponents.push_back(Opponents[idx].position);
	}
	return opponents;
}

bool TeamPlay::AssignAnyRobotPreferedSetPlayer(dynamic_role_e dr_role,
		const PlannerOptions& plannerOptions, game_state_e game_state,
		const MovingObject& globalBall, const std::vector<TeamPlannerOpponent>& Opponents,
		planner_target_e planner_target, bool ballIsObstacle,
		const Vector2D& target, std::vector<TeamPlannerRobot>& Team) {
	// Teamplanner will first select the receiver during set-play.
	// Planner will look to all available (unassigned) field-players.
	// If preferred receiver is available, then it will selected. Else it will select the lowest player-id which is not the kicker.
	// If not available (only kicker on the field), then the kicker will be selected.
	//	Same algorithm will be applied for the kicker (but receiver is then already be assigned).

	int preferredKickerRobotId = plannerOptions.preferredSetplayKicker;
	int preferredReceiverRobotId = plannerOptions.preferredSetplayReceiver;
	int lowestAvailablePlayerIdx = INT_MAX;
	int preferredReceiverTeamIdx = -1;
	int preferredKickerTeamIdx = -1;

	for (unsigned int idx = 0; idx < Team.size(); idx++) {
		if (Team[idx].assigned == false) {
			// player can be assigned
			if (Team[idx].robotId == preferredReceiverRobotId) {
				preferredReceiverTeamIdx = idx;
			}
			else if (Team[idx].robotId == preferredKickerRobotId) {
				preferredKickerTeamIdx = idx;
			}
			else if (lowestAvailablePlayerIdx == INT_MAX) {
				lowestAvailablePlayerIdx = idx;
			}
			else if (Team[idx].robotId <= Team[lowestAvailablePlayerIdx].robotId) {
				lowestAvailablePlayerIdx = idx;
			}
		}
	}
	int foundPlayer = -1;
	if (dr_role == dr_SETPLAY_RECEIVER) {
		if (preferredReceiverTeamIdx >= 0) {
			foundPlayer = preferredReceiverTeamIdx;
		}
		else if (lowestAvailablePlayerIdx != INT_MAX) {
			foundPlayer = lowestAvailablePlayerIdx;
		}
		else if (preferredKickerTeamIdx >= 0) {
			foundPlayer = preferredKickerTeamIdx;
		}
	}
	if (dr_role == dr_SETPLAY_KICKER) {
		if (preferredKickerTeamIdx >= 0) {
			foundPlayer = preferredKickerTeamIdx;
		}
		else if (lowestAvailablePlayerIdx != INT_MAX) {
			foundPlayer = lowestAvailablePlayerIdx;
		}
		else if (preferredReceiverTeamIdx >= 0) {
			foundPlayer = preferredReceiverTeamIdx;
		}
	}


	if (foundPlayer != -1) {
		// robot claimed role+position and that robot has lower id than this robot.
		Team[foundPlayer].result = PlayerPlannerResult();
		Team[foundPlayer].result.gamestate = game_state;
		Team[foundPlayer].dynamic_role = dr_role;
		Team[foundPlayer].result.target = target;
		Team[foundPlayer].result.planner_target = planner_target;
		Team[foundPlayer].assigned = true;

		//					cerr << "PREFERRED ROBOT - id: " << preferredRobotId << " targetpos: " << target.toString() << endl;
		return true;
	}
	return false;
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * Find the player which is the fastest to a single target-position (using path finder),
 * assign the fast player to the position and update the unassigned player list.
 * */
bool TeamPlay::assignAnyToPosition(int role_idx, dynamic_role_e dr_role, game_state_e game_state,
		const MovingObject& globalBall, std::vector<TeamPlannerRobot>& Team,
		const std::vector<TeamPlannerOpponent>& Opponents, const Vector2D& target, bool ballIsObstacle, planner_target_e planner_target,
		const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig, const defend_info_t& Defend_info,
		bool role_position_is_end_position_of_pass, const pass_data_t& pass_data) {
	// cerr << "ROLE-NR: " <<  role_idx << " : " << DynamicRoleAsString(dr_role) << " - AssignedToAnyPosition to target: " <<   target.toString() << endl;
	vector<Vector2D> targets = vector<Vector2D>();
	targets.push_back(target);

	// ------------------------------------------------------------

	// assign Preferred set player roles if applicable
	if (AssignAnyRobotPreferedSetPlayer(dr_role, plannerOptions, game_state, globalBall, Opponents, planner_target, ballIsObstacle, target, Team)) {
		return true; // preferred set player found and assigned
	}

	if (role_position_is_end_position_of_pass && pass_data.valid) {
		/* only if role_position is end postion of pass and pass data is valid*
		 * find player with same id as target_id in pass data (that player is destination of pass) */
		// loop over all players
		for (unsigned int idx = 0; idx < Team.size(); idx++) {
			if (Team[idx].assigned) {
				continue;  // player already assigned, skip
			}
			if (Team[idx].robotId == pass_data.target_id) {
				/* this player is destination of the pass */
				Team[idx].dynamic_role = dr_role;
				Team[idx].result = PlayerPlannerResult();
				Team[idx].result.gamestate = game_state;
				Team[idx].result.target = target;
				Team[idx].result.defend_info = Defend_info;
				Team[idx].result.planner_target = planner_target;
				Team[idx].result.target_position_is_end_position_of_pass = role_position_is_end_position_of_pass;
				Team[idx].assigned = true;
				return true;
			}
		}
	}

	vector <AssignToTargetData> assignToTargetData = vector<AssignToTargetData>();
	double previous_role_threshold =  plannerOptions.previous_role_bonus_end_pos_radius;
	Vector2D currentEndPos = target;  // intended target, can be slightly different than calculated (if path is blocked);
	// do all criteria calculations for the team and store it in AssignToTargetData structs

	for (unsigned int idx = 0; idx < Team.size(); idx++) {
		if (Team[idx].assigned) {
			continue;  // player already assigned, skip
		}
		if (dr_role == dr_BALLPLAYER && Team[idx].controlBall == false) {
			continue; // dribble can only be done, by player with ball, so skip this robot
		}
		if (dr_role == dr_INTERCEPTOR && Team[idx].passBall == true) {
			continue; // ball should not be intercepted by player who passed
		}

		AssignToTargetData data;
		data.team_idx = idx;   // id/position of robot in the Team-vector
		data.robotId = Team[idx].robotId;  // robotId (robot number)

		// Calculate full stop position, in the direction of current velocity vector
		Vector2D linVelocity;
		double rotVelocity;
		Team[idx].position.getVelocity( linVelocity, rotVelocity );

		double acc = plannerOptions.maxPossibleLinearAcceleration;
		double vel = linVelocity.norm();
		double stopDistance = acc == 0 ? 0 : 1/2 * vel * vel / acc;
		Vector2D fullStopPos = Team[idx].position.getPosition().getVector2D().add( linVelocity.normalize().multiply( stopDistance ) );

		// calculate distance from robot's full stop position to target
		data.distToTarget = currentEndPos.distanceTo(fullStopPos);

		// calculate previous target position distance-threshold.
		if ((Team[idx].previous_result.previous_result_present == 1 && Team[idx].previous_result.dynamic_role == dr_role) ||
			(Team[idx].previous_result.previous_result_present == 1 && Team[idx].previous_result.dynamic_role == dr_SWEEPER && dr_role == dr_DEFENDER) ||
		    (Team[idx].previous_result.previous_result_present == 1 && Team[idx].previous_result.dynamic_role == dr_DEFENDER && dr_role == dr_SWEEPER)) {
			Vector2D previousEndPos = Vector2D(Team[idx].previous_result.end_position.x, Team[idx].previous_result.end_position.y);
			data.distToPreviousTarget = 5*max(previous_role_threshold - currentEndPos.distanceTo(previousEndPos), 0.0);
		}
		else {
			data.distToPreviousTarget = 0.0;
		}
		assignToTargetData.push_back(data);

	}


	bool found = assignToTargetData.size() > 0; // at least 1 player will be found
	int bestPlayer = -1;
	for (unsigned player_idx = 0; player_idx < assignToTargetData.size(); player_idx++) {
		if (bestPlayer == -1) {
			bestPlayer = 0; // set best player to the first player which can be assigned.
		}
		else {
			// compare this robot with the current best robot.
			// best robot will become the new best robot
			double totalCostBest    = assignToTargetData[bestPlayer].distToTarget - assignToTargetData[bestPlayer].distToPreviousTarget;
			double totalCostCurrent = assignToTargetData[player_idx].distToTarget - assignToTargetData[player_idx].distToPreviousTarget;

			if (fabs(totalCostBest - totalCostCurrent) < plannerOptions.equality_cost_threshold) {
				if (dr_role == dr_INTERCEPTOR) {
					// for intercepter prefer the player closest to our goal
					Vector2D middleGoal = Vector2D(0, -fieldConfig.getMaxFieldY()+1.0); // 1 meter before own goal
					double distIdxToGoal = middleGoal.distanceTo(Team[player_idx].position.getPosition().getVector2D());
					double distBestToGoal = middleGoal.distanceTo(Team[bestPlayer].position.getPosition().getVector2D());
					if (distIdxToGoal < distBestToGoal) {
						// current player is closer to own goal
						bestPlayer = player_idx;
					}
					if (assignToTargetData[player_idx].robotId < assignToTargetData[bestPlayer].robotId) {
						// current player has lower id.
						bestPlayer = player_idx;
					}
				}
				else {
					// equality: lowest robot id wins
					if (assignToTargetData[player_idx].robotId < assignToTargetData[bestPlayer].robotId) {
						// current player has lower id.
						bestPlayer = player_idx;
					}
				}
			}
			else if (totalCostCurrent < totalCostBest) {
				bestPlayer = player_idx;  // current player will be come the new best player.
			}
		}
	}

	if (found) {
		// fill best robot data in planner result.
		unsigned idx = assignToTargetData[bestPlayer].team_idx;
		Team[idx].dynamic_role = dr_role;
		Team[idx].result = PlayerPlannerResult();
		Team[idx].result.gamestate = game_state;
		Team[idx].result.target = target;
		Team[idx].result.defend_info = Defend_info;
		Team[idx].result.planner_target = planner_target;
		Team[idx].result.target_position_is_end_position_of_pass = role_position_is_end_position_of_pass;
		Team[idx].assigned = true;
		// cerr << "BEST ROBOT - id: " << idx  << "(id = " <<  Team[idx].robotId << ") targetpos: " << target.toString() << endl;
	}
	return found;
}

//---------------------------------------------------------------------------------------------------------------------
// return true if for the new path the costs are lower than the costs of the lowest path. If lower check apply hysteresis threshold is passed.
// update lowest_pathcost only if path is faster and newCost are lower than current lowest_pathcost. This to prevent slippery target value.
bool TeamPlay::check_better_path_found(double& lowest_pathcost, double newPathCost,
		double fastestPathCost, const PlayerPlannerResult& new_path,
		const PlayerPlannerResult& fastest_path, double equality_cost_threshold)
{
	bool set_new_path = false;
	if (newPathCost < (fastestPathCost - equality_cost_threshold)) {
		// newPathCost much better than previous
		set_new_path = true;
	}
	else if (fabs(newPathCost - fastestPathCost) < equality_cost_threshold) {
		// more or less equal path costs
		if (new_path.path.size() > 1 && fastest_path.path.size() > 1) {
			double fastest_additional_penalty = fastest_path.path[0].y * 100 + fastest_path.path[0].x;
			double path_additional_penalty = new_path.path[0].y * 100 + new_path.path[0].x;
			set_new_path = (path_additional_penalty < fastest_additional_penalty);
		}
	}
	if (set_new_path) {
		if (newPathCost < lowest_pathcost) {
			lowest_pathcost = newPathCost;
		}
	}
	return set_new_path;
}

/**
 * Assign the goalie role to the  first player with player-type: goalie Calculate its best position and plan a path to it.
 */
void TeamPlay::assignGoalie(game_state_e gamestate, std::vector<TeamPlannerRobot>& Team, bool ballIsObstacle, const MovingObject& globalBall, const std::vector<TeamPlannerOpponent>& Opponents, const PlannerOptions& plannerOptions,
		const FieldConfig& fieldConfig, const std::vector<Vector2D>& parking_positions)
{
	double goalieYPosition = -fieldConfig.getMaxFieldY()+0.5;
	Vector2D goaliePosition = Vector2D(0, goalieYPosition); // center of the goal;
	if (gamestate == game_state_e::PARKING) {
		// select position closest to default goalie position as parking position for the goalie
		Vector2D startPost = Vector2D(0, -fieldConfig.getMaxFieldY());
		goaliePosition = startPost.closestTo(parking_positions);
	}
	else {
		if (globalBall.isValid()) {
			Vector2D ballPos = globalBall.getPosition().getVector2D();
			double penalty_area_half_width = fieldConfig.getPenaltyAreaWidth() * 0.5;
			if (ballPos.m_x < -penalty_area_half_width) {
				//			if (global) ball is to the left of the field (left from outer line penalty area) position keeper to the left.
				goaliePosition = Vector2D(-fieldConfig.getGoalWidth()*0.25, goalieYPosition); // left half of the goal;
			}
			else if (ballPos.m_x > penalty_area_half_width) {
				goaliePosition = Vector2D(+fieldConfig.getGoalWidth()*0.25, goalieYPosition); // right half of the goal;
			}
		}
	}

	unsigned keeper_idx = 0;
	bool keeperFound = false;

	for (unsigned int idx = 0; keeperFound == false && idx < Team.size(); idx++) {
		if (Team[idx].player_type == GOALIE) {
			keeperFound = true;
			keeper_idx = idx;
		}
	}

	if (!keeperFound && plannerOptions.autoAssignGoalie) {
		// no dedicated goalie found and auto-assign goalie is enabled.

		//find preferred setplay kicker and receiver
		int setPlayKickerIdx = -1;
		int setPlayRecieverIdx = -1;
		int preferredKickerRobotId = plannerOptions.preferredSetplayKicker;
		int preferredReceiverRobotId = plannerOptions.preferredSetplayReceiver;
		vector<int> availablePlayers = vector<int>();

		if (preferredReceiverRobotId == preferredKickerRobotId)
		{
			preferredReceiverRobotId = 0; // avoid select conflict. prefer kicker above reciever.
		}

		for (unsigned int idx = 0; idx < Team.size(); idx++) {
				// player can be assigned
				if (Team[idx].robotId == preferredReceiverRobotId) {
					setPlayRecieverIdx = idx;
				}
				else if (Team[idx].robotId == preferredKickerRobotId) {
					setPlayKickerIdx = idx;
				}
				else {
					availablePlayers.push_back(idx);
				}
		}

		// no available robots
		if (availablePlayers.size() < 1)
		{
			if (setPlayRecieverIdx != -1) {
				availablePlayers.push_back(setPlayRecieverIdx);
			}
			else if (setPlayKickerIdx != -1) {
				availablePlayers.push_back(setPlayKickerIdx);
			}
		}
		// no goalie available. Assign field player with lowest number to goalie role.
		int lowestAvailablePlayerIdx = INT_MAX;
		for (unsigned int idx = 0; idx < availablePlayers.size(); idx++) {
			int player_idx = availablePlayers[idx];
			if (lowestAvailablePlayerIdx == INT_MAX) {
				// first found player.
				lowestAvailablePlayerIdx = player_idx;
				keeperFound = true; // indicate that goalie is found
			}
			else if (Team[player_idx].robotId <= Team[lowestAvailablePlayerIdx].robotId) {
				lowestAvailablePlayerIdx = player_idx; // better goalie == lower robot_id
			}
		}
		keeper_idx = lowestAvailablePlayerIdx;
	}

	if (keeperFound) {
		Team[keeper_idx].assigned = true;
		Team[keeper_idx].result.target = goaliePosition;
		Team[keeper_idx].result.planner_target = gamestate == game_state_e::PARKING ? planner_target_e::GOTO_TARGET_POSITION_SLOW : planner_target_e::GOALIE;
		Team[keeper_idx].dynamic_role = gamestate == game_state_e::PARKING ? dr_PARKING : dr_GOALKEEPER;
		//cerr << keeper_idx <<": DR: " << DynamicRoleAsString(dr_GOALKEEPER) << " pos: " << goaliePosition.toString() << endl;
	}

	return;
}

//----------------------------------------------------------------------------------------
void TeamPlay::assignTooLongInPenaltyAreaPlayers(game_state_e gamestate, std::vector<TeamPlannerRobot>& Team, bool ballIsObstacle, const MovingObject& globalBall, const std::vector<TeamPlannerOpponent>& Opponents, const PlannerOptions& plannerOptions,
		const FieldConfig& fieldConfig)
{
	// max allowed time in penalty area in 2019: 10 seconds (attack and defense)
	const double TIME_TOO_LONG_IN_PENALTY_AREA_THRESHOLD = 8.0;  // 2 seconds to move outside the penalty area
	if (gamestate != NONE && gamestate != PARKING) {
		double d_x = fieldConfig.getPenaltyAreaWidth() * 0.5;
		double d_y = -(fieldConfig.FIELD_LENGTH/2) + fieldConfig.PENALTY_AREA_LENGTH;
		for (unsigned int idx = 0; idx < Team.size(); idx++) {
			if (!Team[idx].assigned) {

				if (Team[idx].time_in_own_penalty_area > TIME_TOO_LONG_IN_PENALTY_AREA_THRESHOLD) {
					// player not assigned and too long in penalty area
					Vector2D pos = Team[idx].position.getXYlocation();
					if (fieldConfig.isInOwnPenaltyArea(pos.m_x, pos.m_y)) {
						double d_y = -(fieldConfig.FIELD_LENGTH/2) + fieldConfig.PENALTY_AREA_LENGTH;
						Vector2D targetPos = pos;
						if (fabs(d_y - pos.m_y) < fabs(d_x - fabs(pos.m_x))) {
							// closest to Y line
							targetPos.m_y = d_y + fieldConfig.getRobotRadius();
						}
						else {
							// closest to side of penalty area
							double safe_x_pos = d_x + fieldConfig.getRobotRadius();
							if (pos.m_x > 0) {
								targetPos.m_x = safe_x_pos;
							}
							else {
								targetPos.m_x = -safe_x_pos;
							}
						}
						Team[idx].assigned = true;
						Team[idx].result.target = targetPos;
						Team[idx].result.planner_target = SUPPORT_DEFENSE;
						Team[idx].dynamic_role = dr_DEFENDER;
					}
				}
				if (Team[idx].time_in_opponent_penalty_area > TIME_TOO_LONG_IN_PENALTY_AREA_THRESHOLD) { // TODO
					// player not assigned and too long in penalty area
					Vector2D pos = Team[idx].position.getXYlocation();
					if (fieldConfig.isInOpponentPenaltyArea(pos.m_x, pos.m_y)) {
						double d_y2 = -d_y;
						Vector2D targetPos = pos;
						if (fabs(d_y2 - pos.m_y) < fabs(d_x - fabs(pos.m_x))) {
							// closest to Y line
							targetPos.m_y = d_y2 - fieldConfig.getRobotRadius();
						}
						else {
							// closest to side of penalty area
							double safe_x_pos = d_x + fieldConfig.getRobotRadius();
							if (pos.m_x > 0) {
								targetPos.m_x = safe_x_pos;
							}
							else {
								targetPos.m_x = -safe_x_pos;
							}
						}
						Team[idx].assigned = true;
						Team[idx].result.target = targetPos;
						Team[idx].result.planner_target = SUPPORT_DEFENSE;
						Team[idx].dynamic_role = dr_DEFENDER;
					}
				}
			}
		}
	}
}

//----------------------------------------------------------------------------------------
vector<MovingObject> TeamPlay::getTeamMates(const std::vector<TeamPlannerRobot>& Team, unsigned meIdx, bool addAssignedTargetAsTeamPosition) {
	vector<MovingObject> teammates = vector<MovingObject>();
	for (unsigned int idx = 0; idx < Team.size(); idx++) {
		if (idx != meIdx) {
			teammates.push_back(Team[idx].position);
			if (addAssignedTargetAsTeamPosition && Team[idx].assigned) {
				//add target position as barrier
				if (Team[idx].result.path.size() > 0) {
					planner_piece_t last_piece = Team[idx].result.path[Team[idx].result.path.size()-1];
				    Position lastPos = Position(Vector2D(last_piece.x, last_piece.y));
				    teammates.push_back(MovingObject(lastPos, true));
				}
			}
		}
	}
 	return teammates;
}

//---------------------------------------------------------------------------------------------------------------------
bool TeamPlay::searchForBallBehaviorNeeded(game_state_e gamestate, const MovingObject& globalBall, const FieldConfig& fieldConfig) {
	bool searchForBall = false;
	if (!globalBall.isValid()) {
		// no ball seen
		if ((gamestate != game_state_e::BEGIN_POSITION) && (gamestate != game_state_e::PARKING)) {
			searchForBall = true; // state requires search for ball
		}
	}
	else {
		// ball is valid, check if ball position is with the possible area during a game.
		if (!fieldConfig.isInReachableField(globalBall.getPosition().getVector2D().m_x, globalBall.getPosition().getVector2D().m_y)	) {
			// ball is outside field (not in field and not in safety area)
			searchForBall = true; // state requires search for ball
		}
	}
	return searchForBall;
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * Assign all players to list of the given positions (order of list determines the priority, most important position is the first position in the list)
 * by using the path calculation, only the goalie-role will have a fixed path to the goal.
 */
void TeamPlay::assignToFixedPositions(unsigned playerlist_idx, dynamic_role_e dynamic_role, game_state_e gamestate, std::vector<TeamPlannerRobot>& Team, const std::vector<TeamPlannerOpponent>& Opponents,
		const MovingObject& globalBall, bool ballIsObstacle, const std::vector<Vector2D>& parking_positions,
		const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig, bool searchForBall, const defend_info_t& Defend_info, const pass_data_t& pass_data) {
	vector<Vector2D> playerPositions = vector<Vector2D>();  //TODO start position must come from configuration file
	RolePosition::GetFixedPositions(playerPositions, gamestate, globalBall, searchForBall, parking_positions, fieldConfig, plannerOptions);

	planner_target_e planner_target = planner_target_e::GOTO_TARGET_POSITION;
	if (gamestate == game_state_e::PARKING) {
		planner_target = planner_target_e::GOTO_TARGET_POSITION_SLOW;
	}

	// use overwritten dynamic role: no impact only easier to follow for humans
	dynamic_role_e overwritten_dynamic_role = dynamic_role;
	if (searchForBall) {
		overwritten_dynamic_role = dynamic_role_e::dr_SEARCH_FOR_BALL;
	}
	if (gamestate == game_state_e::PARKING) {
		overwritten_dynamic_role = dynamic_role_e::dr_PARKING;
	}
	else if (gamestate == game_state_e::BEGIN_POSITION) {
		overwritten_dynamic_role = dynamic_role_e::dr_BEGIN_POSITION;
	}

	if (playerlist_idx < playerPositions.size()) {
		Vector2D rolePosition = playerPositions[playerlist_idx];
		assignAnyToPosition(playerlist_idx, overwritten_dynamic_role, gamestate, globalBall,
				Team, Opponents, rolePosition, ballIsObstacle,  planner_target, plannerOptions, fieldConfig, Defend_info, false, pass_data);
	}

	return;
}

//---------------------------------------------------------------------------------------------------------------------
// print the inputs of TeamPlanner::Assign for debug purposes
void TeamPlay::printAssignInputs(game_state_e gamestate,
		const MovingObject& globalBall, const MovingObject& localBall,
		std::vector<TeamPlannerRobot>& Team, const std::vector<TeamPlannerOpponent>& Opponents,
		const PlannerOptions& plannerOptions, const std::vector<Vector2D>& parking_positions, const ball_pickup_position_t& ball_pickup_position,
		bool passIsRequired, const pass_data_t& pass_data)
{
	cerr << "Team_Planner::assign  inputs:" << endl << flush;
	cerr << "game_state_e: " << gamestate << " (" << GameStateAsString(gamestate) << " )"<< endl << flush;
	cerr << "global ball: " << globalBall.toString() << endl << flush;
	cerr << "local ball: " << localBall.toString() << endl << flush;
	for (unsigned idx = 0; idx < Team.size(); idx++) {
		cerr << "Robot [" << idx << "] =" << endl << Team[idx].toString() << endl;
	}
	cerr << "Opponents size: " << Opponents.size()  << endl << flush;
	for (unsigned int i = 0; i < Opponents.size(); i++) {
		cerr << "Opponents[" << i << "].position: "<< Opponents[i].position.toString()  << endl << flush;
	}
	cerr << "plannerOptions: " << plannerOptions.toString() << endl << flush;
	cerr << "parking positions size: " << parking_positions.size() << endl << flush;
	for (unsigned int idx = 0; idx < parking_positions.size(); idx++) {
		cerr << parking_positions[idx].toString() << endl << flush;
	}

	if (ball_pickup_position.valid)
		cerr << "pickup: valid, x:" << std::setprecision(2) << ball_pickup_position.x << " y: " << ball_pickup_position.y << " ts:" << ball_pickup_position.ts << endl << flush;
	else{
		cerr << "pickup: invalid " << endl << flush;
	}

	cerr << "passIsRequired: " << (passIsRequired ? "true" : "false") << endl << flush;

	if (pass_data.valid) {
		cerr << "pass_data: valid target_id: " << pass_data.target_id << endl << flush;
		//		pass_data.target_id
		//		pass_data.kicked
		//		pass_data.origin_pos
		//		pass_data.target_pos
		//		pass_data.velocity
		//		pass_data.ts
		//		pass_data.eta
		//		pass_data.angle

	}
	else {
		cerr << "pass_data: invalid " << endl << flush;
	}
}

void TeamPlay::ReplanInterceptorWithLocalBall(const MovingObject& localBall, unsigned interceptorIdx,
		std::vector<TeamPlannerRobot>& Team, const std::vector<TeamPlannerOpponent>& Opponents,
		const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig, bool ballIsObstacle)
{
	if (localBall.isValid() == false) {
		return; // invalid ball, can not calculate new path
	}
	// this player is interceptor and game state is normal.
	// Replan: using dynamic path planner and local ball.
	double shortestDistanceOpponentToBall = calculateShortestDistanceObjectsToTarget(getOpponents(Opponents), localBall);
	int nrDynamicPlannerIterations = plannerOptions.nrDynamicPlannerIterations;
	double maxSpeed = plannerOptions.maxPossibleLinearSpeed;
	double distRobotToBall = Team[0].position.getPosition().getVector2D().distanceTo(localBall.getPosition().getVector2D());
	// check if ball approach from best angle must be applied.
	if (distRobotToBall < shortestDistanceOpponentToBall + 1.0) {
		// always approach from outside when the robot is closer to the ball than any opponent + 1.0 meter
		plannerOptions.addBallApproachVertices = true;
		plannerOptions.distToapplyBallApproachVertices = fieldConfig.getFieldWidth();
	} else {
		// go straight to the ball
		plannerOptions.addBallApproachVertices = false;
	}

	vector<trs::Vertex> targetPos = vector<trs::Vertex>();
	targetPos.push_back(Vertex(localBall.getPosition().getVector2D(), 0));
	std::vector<planner_piece_t> path;

	vector<MovingObject> myTeam = getTeamMates(Team, interceptorIdx, true);

	if (nrDynamicPlannerIterations <= 0) {
		bool avoidBallPath = false; // intercept should never avoid a passing ball.
		Vector2D BallTargePos;
		// use normal planner
		// when ball is obstacle or no iterations for dynamic planner is defined.
		GlobalPathPlanner visibilityGraph = GlobalPathPlanner(fieldConfig);
		visibilityGraph.setOptions(plannerOptions);
		visibilityGraph.createGraph(Team[interceptorIdx].position, localBall, myTeam, getOpponents(Opponents),
				targetPos, planner_target_e::GOTO_BALL, ballIsObstacle, avoidBallPath, BallTargePos);
		path = visibilityGraph.getShortestPath();
	} else {
		// use dynamic robot planner for goto ball
		GlobalPathDynamicPlanner dp = GlobalPathDynamicPlanner();
		path = dp.planPath(	Team[interceptorIdx].position, localBall, myTeam, getOpponents(Opponents), targetPos,
				planner_target_e::GOTO_BALL, ballIsObstacle, maxSpeed, plannerOptions, nrDynamicPlannerIterations, fieldConfig);
	}
	Team[interceptorIdx].result.path = path;
}

////---------------------------------------------------------------------------------------------------------------------
// calculate shortest distance of the vector of objects (own or opponents) to the ball
double TeamPlay::calculateShortestDistanceObjectsToTarget(const std::vector<MovingObject>& objects, const MovingObject& targetObject) {
	double shortestDistance = std::numeric_limits<double>::infinity();
	for(unsigned int idx = 0; idx < objects.size(); idx++) {
		double distToBall = objects[idx].getPosition().getVector2D().distanceTo(targetObject.getPosition().getVector2D());
		if (distToBall < shortestDistance) {
			shortestDistance = distToBall;
		}
	}
	return shortestDistance;
}
