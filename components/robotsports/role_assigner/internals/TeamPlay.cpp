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

#include "geometry.hpp"

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

void TeamPlay::assign(const TeamPlannerInput& input, TeamPlannerState& r_state, TeamPlannerOutput& r_output, const TeamPlannerParameters& plannerOptions)
{
//	game_state_e gamestate = input.gamestate;
	std::vector<TeamPlannerRobot> Team =  input.Team;
	std::vector<TeamPlannerOpponent> Opponents = input.Opponents;
	trs::MovingObject globalBall;
	TeamPlannerData teamplanner_data; // TODO
	game_state_e org_gamestate = input.gamestate;

	if (input.gamestate != game_state_e::NONE) {
		// printAssignInputs(input.gamestate, globalBall, Team, Opponents, plannerOptions,  input.parking_positions, input.ball_pickup_position, input.passIsRequired, teamplanner_data.pass_data);
	}
	if ((input.gamestate == game_state_e::NONE) ||
			(input.gamestate == game_state_e::YELLOW_CARD_AGAINST) ||
			(input.gamestate == game_state_e::RED_CARD_AGAINST) ||
			(input.gamestate == game_state_e::GOAL) ||
			(input.gamestate == game_state_e::GOAL_AGAINST)) {
		// unhandled game situations: cards and goals are not passed to the team planner via the game state-machine
		for (unsigned idx = 0; idx < teamplanner_data.team.size(); idx++) {
			(*r_output.player_paths)[idx].path = std::vector<planner_piece_t>();
			(*r_output.player_paths)[idx].gamestate = input.gamestate;
			(*r_output.player_paths)[idx].dynamic_role = dynamic_role_e::dr_NONE;
			(*r_output.player_paths)[idx].defend_info.valid = false;
		}
		return; // no path will be planned if game state is NONE
	}

	for (unsigned r_idx = 0; r_idx < teamplanner_data.team.size(); r_idx++) {
	    teamplanner_data.team[r_idx].result.gamestate = input.gamestate;
	    teamplanner_data.team[r_idx].result.dynamic_role = dynamic_role_e::dr_NONE;
	    teamplanner_data.team[r_idx].result.defend_info.valid = false;

	}


	/* set avoid ball flag : only not avoiding during normal play */
	bool ballIsObstacle = (input.gamestate != game_state_e::NORMAL_ATTACK) && (input.gamestate != game_state_e::NORMAL_DEFEND);

	// Assign first the Goalie
	assignGoalie(teamplanner_data);

	// Assign players too long in any penalty area: closest position out of the penalty area
	assignTooLongInPenaltyAreaPlayers(teamplanner_data);


	bool searchForBall = searchForBallBehaviorNeeded(teamplanner_data);
	for (unsigned dr_idx = 0; dr_idx < input.teamFormation.size(); dr_idx++) {
		if (dr_idx >= teamplanner_data.team.size()) {
			// can not assign more roles than team members.
			break;
		}

		planner_target_e planner_target = determine_planner_target(input.teamFormation[dr_idx], input.gamestate);

		bool role_position_is_end_position_of_pass = false;
		MRA::Geometry::Point rolePosition = RolePosition::determineDynamicRolePosition(teamplanner_data.defend_info, planner_target, m_gridFileNumber, input.teamFormation[dr_idx], input.gamestate,
		        globalBall, r_state, Team, Opponents, plannerOptions, input.fieldConfig, input.ball_pickup_position,
				input.passIsRequired, input.teamControlBall, input.playerPassedBall, teamplanner_data.pass_data, role_position_is_end_position_of_pass);
		if (  searchForBall || input.gamestate == game_state_e::BEGIN_POSITION || input.gamestate ==  game_state_e::PARKING
				|| input.gamestate ==  game_state_e::KICKOFF || input.gamestate ==  game_state_e::KICKOFF_AGAINST)
		{
			// Fixed position assignment
			assignToFixedPositions(teamplanner_data, dr_idx, input.teamFormation[dr_idx]);
		}
		else {
			//	Determine path for assigned role
			assignAnyToPosition(teamplanner_data, static_cast<int>(dr_idx), input.teamFormation[dr_idx], rolePosition, planner_target, role_position_is_end_position_of_pass);
		}
		// stop if current player has ball
		if (teamplanner_data.team[0].assigned && plannerOptions.calculateAllPaths == false) {
			// path for this robot (player nr: 0) is found and not all paths must be calculated.
			break;
		}
		//cerr << dr_idx <<": DR: " << DynamicRoleAsString(teamFormation[dr_idx]) << " pos: " << rolePosition.toString() << endl;
	}

	// ASSIGN ALWAYS A ROLE TO THE AVAIALABLE PLAYERS
	if (teamplanner_data.team[0].assigned == false || plannerOptions.calculateAllPaths)
	{
		// current player does not have any role or all players must be assigned.
		// assign defender role to all unassigned players
		for (unsigned ap_idx = 0; ap_idx < teamplanner_data.team.size(); ap_idx++) {
			if (teamplanner_data.team[ap_idx].assigned == false)  {
				//	Determine path for assigned role
				planner_target_e planner_target = determine_planner_target(dynamic_role_e::dr_DEFENDER, input.gamestate);
				bool role_position_is_end_position_of_pass = false;
				MRA::Geometry::Point rolePosition = RolePosition::determineDynamicRolePosition(teamplanner_data.defend_info, planner_target, m_gridFileNumber, dynamic_role_e::dr_DEFENDER, input.gamestate,
				        globalBall, r_state, Team, Opponents, plannerOptions, input.fieldConfig, input.ball_pickup_position, input.passIsRequired, input.teamControlBall, input.playerPassedBall, teamplanner_data.pass_data, role_position_is_end_position_of_pass);
				assignAnyToPosition(teamplanner_data, static_cast<int>(ap_idx), dynamic_role_e::dr_DEFENDER,
				        rolePosition, planner_target, role_position_is_end_position_of_pass);
			}
		}
	}

	bool avoidBallPath = teamplanner_data.pass_data.valid;
	MRA::Geometry::Point BallTargePos = MRA::Geometry::Point(teamplanner_data.pass_data.target_pos.x, teamplanner_data.pass_data.target_pos.y);
	// Calculate Robot Planner path
	for (unsigned idx = 0; idx < teamplanner_data.team.size(); idx++) {
		if (teamplanner_data.team[idx].assigned == true)  {
			// calculate path for robot.

			avoidBallPath = teamplanner_data.pass_data.valid; // avoid bal path only if pass made (or shot on goal)
			if (teamplanner_data.team[idx].result.target_position_is_end_position_of_pass)
			{
				// in case of pass, don't avoid ball path if player is destination of the pass
				avoidBallPath = false;
			}

			vector<MovingObject> myTeam = getTeamMates(Team, idx, true);
			GlobalPathPlanner visibilityGraph = GlobalPathPlanner(input.fieldConfig); // create robot planner
			visibilityGraph.setOptions(plannerOptions);
			// create list of possible targets for robot-planner
			std::vector<trs::Vertex> targetPos = vector<trs::Vertex>();
			targetPos.push_back(Vertex(teamplanner_data.team[idx].result.target, 0));
			visibilityGraph.createGraph(teamplanner_data.team[idx].position, teamplanner_data,
					targetPos, teamplanner_data.team[idx].result.planner_target, ballIsObstacle, avoidBallPath, BallTargePos);
			teamplanner_data.team[idx].result.path = visibilityGraph.getShortestPath(teamplanner_data);
		}
		if (plannerOptions.calculateAllPaths == false) {
			// path for this robot (player nr: 0) is found and not all paths must be calculated.
			break;
		}
	}

	if (input.gamestate == NORMAL_DEFEND) {
		// replan interceptor with ball only if interceptor is not performing a priority block
		int interceptorIdx = -1;
		for (unsigned idx = 0; idx < teamplanner_data.team.size(); idx++) {
			if (teamplanner_data.team[idx].assigned == true)  {
				if (teamplanner_data.team[idx].dynamic_role == dr_INTERCEPTOR) {
					interceptorIdx = idx;
				}

			}
		}

		bool replan = false;
		if ((interceptorIdx == 0) || plannerOptions.calculateAllPaths) {
			replan = (interceptorIdx != -1) && (teamplanner_data.team[interceptorIdx].result.path.size() > 0) && (teamplanner_data.team[interceptorIdx].result.path[0].target != PRIORITY_BLOCK);
		}
		if (replan && interceptorIdx >= 0) {
			ReplanInterceptor(interceptorIdx, teamplanner_data);
		}
	}

	// loop over the player path to verify if path of this player stay within the allowed boundaries (field+safety area)
	bool thisPlayerHasUnallowedPath = false;
	bool thisPlayerStartsAtUnallowedPosition = false;
	for (unsigned idx = 0; idx < teamplanner_data.team.size(); idx++) {
		if (teamplanner_data.team[idx].assigned == true) {
			bool pathOK = stayPathWithinBoundaries(input.fieldConfig, teamplanner_data.team[idx].result);
			(*r_output.player_paths)[idx] = teamplanner_data.team[idx].result;
			(*r_output.player_paths)[idx].gamestate = teamplanner_data.team[idx].result.gamestate;
			(*r_output.player_paths)[idx].dynamic_role = teamplanner_data.team[idx].dynamic_role;
			if (!pathOK && idx == 0) { // CHECK Only this robot
				thisPlayerHasUnallowedPath = true; // this robot has wrong path

				// check if player starts at position in the field
				auto currentPos = teamplanner_data.team[idx].position.getPosition().getPoint();
				thisPlayerStartsAtUnallowedPosition = !input.fieldConfig.isInReachableField(currentPos.x, currentPos.y);
			}
		}
	}

	bool dynamicRoleNoneAssigned =  teamplanner_data.team[0].dynamic_role == dr_NONE;
	// Dump diagnostics if any robot moves outside the allowed field (safety border + field)
	if (thisPlayerHasUnallowedPath || plannerOptions.svgOutputFileName.length() > 0 || dynamicRoleNoneAssigned) {
		// Create diagnostics file (.svg with comments).
		// file can be used to diagnose with path to illegal locations are planned
		std::vector<player_type_e> teamTypes = std::vector<player_type_e>();
		std::vector<long> robotIds = std::vector<long>();

		for (unsigned r_idx = 0; r_idx < teamplanner_data.team.size(); r_idx++) {
			robotIds.push_back(teamplanner_data.team[r_idx].robotId);
			teamTypes.push_back(teamplanner_data.team[r_idx].player_type);
		}
		TeamPlannerParameters options = plannerOptions;
		if (thisPlayerHasUnallowedPath) {
			if (thisPlayerStartsAtUnallowedPosition)
				options.svgOutputFileName = GetTeamPlannerSVGname(input.gamestate, "OUTSIDE_FIELD_BEGIN_ERROR");
			else {
				options.svgOutputFileName = GetTeamPlannerSVGname(input.gamestate, "OUTSIDE_FIELD_END_ERROR");
			}
		}
		if (dynamicRoleNoneAssigned)
		{
			options.svgOutputFileName = GetTeamPlannerSVGname(input.gamestate, "DYN_ROLE_NONE");
		}

		class TeamPlannerInputInfo  inputInfo;
		inputInfo.playerWhoIsPassing = -1;
		for (unsigned r_idx = 0; r_idx < teamplanner_data.team.size(); r_idx++) {
			if (teamplanner_data.team[r_idx].passBall) {
				inputInfo.playerWhoIsPassing = r_idx;
			}
			inputInfo.previous_results.push_back(teamplanner_data.team[r_idx].previous_result);
		}
		inputInfo.ball_pickup_position = input.ball_pickup_position;
		inputInfo.passIsRequired = input.passIsRequired;
		inputInfo.pass_data = teamplanner_data.pass_data;



        long controlBallByPlayer = -1;
        for (unsigned r_idx = 0; r_idx < teamplanner_data.team.size(); r_idx++) {
            if(teamplanner_data.team[r_idx].controlBall) {
                controlBallByPlayer = static_cast<int>(r_idx);
            }
        }
        bool hasTeamPlannerInputInfo = true;
		SvgUtils::save_graph_as_svg(teamplanner_data, *r_output.player_paths,
		        options, std::vector<Vertex*>(), org_gamestate, controlBallByPlayer, teamTypes, robotIds, "red", hasTeamPlannerInputInfo, inputInfo);

		// create empty path for robot with wrong path
		if (thisPlayerHasUnallowedPath) {
			std::vector<planner_piece_t> path = (*r_output.player_paths)[0].path;
			if (!path.empty()) {
				planner_piece_t last_piece = path[path.size()-1];
				if (input.fieldConfig.isInReachableField(last_piece.x,last_piece.y) == false) {
					(*r_output.player_paths)[0] = PlayerPlannerResult();
				}
			}
		}
	}

	//printAssignOutputs(Team, *r_output.player_paths);
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
void TeamPlay::printAssignOutputs(const std::vector<TeamPlannerRobot>& team, team_planner_result_t&  player_paths)
{
	//Print final position
	for (unsigned int player_idx = 0; player_idx < player_paths.size(); player_idx++) {
		PlayerPlannerResult playerResult = player_paths[player_idx];
		cerr << "[" << player_idx  << "] Player-id: " << team[player_idx].robotId << DynamicRoleAsString(team[player_idx].dynamic_role) << endl;
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

bool TeamPlay::AssignAnyRobotPreferedSetPlayer(TeamPlannerData&  teamplanner_data,
        dynamic_role_e dr_role, planner_target_e planner_target, const MRA::Geometry::Point& target) {
	// Teamplanner will first select the receiver during set-play.
	// Planner will look to all available (unassigned) field-players.
	// If preferred receiver is available, then it will selected. Else it will select the lowest player-id which is not the kicker.
	// If not available (only kicker on the field), then the kicker will be selected.
	//	Same algorithm will be applied for the kicker (but receiver is then already be assigned).

	int preferredKickerRobotId = teamplanner_data.parameters.preferredSetplayKicker;
	int preferredReceiverRobotId = teamplanner_data.parameters.preferredSetplayReceiver;
	int lowestAvailablePlayerIdx = INT_MAX;
	int preferredReceiverTeamIdx = -1;
	int preferredKickerTeamIdx = -1;

	for (unsigned int idx = 0; idx < teamplanner_data.team.size(); idx++) {
		if (teamplanner_data.team[idx].assigned == false) {
			// player can be assigned
			if (teamplanner_data.team[idx].robotId == preferredReceiverRobotId) {
				preferredReceiverTeamIdx = idx;
			}
			else if (teamplanner_data.team[idx].robotId == preferredKickerRobotId) {
				preferredKickerTeamIdx = idx;
			}
			else if (lowestAvailablePlayerIdx == INT_MAX) {
				lowestAvailablePlayerIdx = idx;
			}
			else if (teamplanner_data.team[idx].robotId <= teamplanner_data.team[lowestAvailablePlayerIdx].robotId) {
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
	    teamplanner_data.team[foundPlayer].result = PlayerPlannerResult();
	    teamplanner_data.team[foundPlayer].result.gamestate = teamplanner_data.gamestate;
	    teamplanner_data.team[foundPlayer].dynamic_role = dr_role;
	    teamplanner_data.team[foundPlayer].result.target = target;
	    teamplanner_data.team[foundPlayer].result.planner_target = planner_target;
	    teamplanner_data.team[foundPlayer].assigned = true;

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
bool TeamPlay::assignAnyToPosition(TeamPlannerData&  teamplanner_data, int role_idx, dynamic_role_e dr_role,
        const MRA::Geometry::Point& target, planner_target_e planner_target, bool role_position_is_end_position_of_pass) {

	// cerr << "ROLE-NR: " <<  role_idx << " : " << DynamicRoleAsString(dr_role) << " - AssignedToAnyPosition to target: " <<   target.toString() << endl;
	vector<MRA::Geometry::Point> targets = vector<MRA::Geometry::Point>();
	targets.push_back(target);

	// ------------------------------------------------------------

	// assign Preferred set player roles if applicable
	if (AssignAnyRobotPreferedSetPlayer(teamplanner_data, dr_role, planner_target, target)) {
		return true; // preferred set player found and assigned
	}

	if (role_position_is_end_position_of_pass && teamplanner_data.pass_data.valid) {
		/* only if role_position is end postion of pass and pass data is valid*
		 * find player with same id as target_id in pass data (that player is destination of pass) */
		// loop over all players
		for (unsigned int idx = 0; idx < teamplanner_data.team.size(); idx++) {
			if (teamplanner_data.team[idx].assigned) {
				continue;  // player already assigned, skip
			}
			if (teamplanner_data.team[idx].robotId == teamplanner_data.pass_data.target_id) {
				/* this player is destination of the pass */
			    teamplanner_data.team[idx].dynamic_role = dr_role;
				teamplanner_data.team[idx].result = PlayerPlannerResult();
				teamplanner_data.team[idx].result.gamestate = teamplanner_data.gamestate;
				teamplanner_data.team[idx].result.target = target;
				teamplanner_data.team[idx].result.defend_info = teamplanner_data.defend_info;
				teamplanner_data.team[idx].result.planner_target = planner_target;
				teamplanner_data.team[idx].result.target_position_is_end_position_of_pass = role_position_is_end_position_of_pass;
				teamplanner_data.team[idx].assigned = true;
				return true;
			}
		}
	}

	vector <AssignToTargetData> assignToTargetData = vector<AssignToTargetData>();
	double previous_role_threshold =  teamplanner_data.parameters.previous_role_bonus_end_pos_radius;
	MRA::Geometry::Point currentEndPos = target;  // intended target, can be slightly different than calculated (if path is blocked);
	// do all criteria calculations for the team and store it in AssignToTargetData structs

	for (unsigned int idx = 0; idx < teamplanner_data.team.size(); idx++) {
		if (teamplanner_data.team[idx].assigned) {
			continue;  // player already assigned, skip
		}
		if (dr_role == dr_BALLPLAYER && teamplanner_data.team[idx].controlBall == false) {
			continue; // dribble can only be done, by player with ball, so skip this robot
		}
		if (dr_role == dr_INTERCEPTOR && teamplanner_data.team[idx].passBall == true) {
			continue; // ball should not be intercepted by player who passed
		}

		AssignToTargetData data;
		data.team_idx = idx;   // id/position of robot in the Team-vector
		data.robotId = teamplanner_data.team[idx].robotId;  // robotId (robot number)

		// Calculate full stop position, in the direction of current velocity vector
		MRA::Geometry::Point linVelocity;
		teamplanner_data.team[idx].position.getVelocity( linVelocity );

		double acc = teamplanner_data.parameters.maxPossibleLinearAcceleration;
		double vel = linVelocity.size();
		double stopDistance = acc == 0 ? 0 : 1/2 * vel * vel / acc;
		MRA::Geometry::Point  stopDist(linVelocity);
		stopDist.normalize();
		stopDist *= stopDistance;
		MRA::Geometry::Point fullStopPos = teamplanner_data.team[idx].position.getPosition().getPoint();
		fullStopPos += stopDist;

		// calculate distance from robot's full stop position to target
		data.distToTarget = currentEndPos.distanceTo(fullStopPos);

		// calculate previous target position distance-threshold.
		if ((teamplanner_data.team[idx].previous_result.previous_result_present == 1 && teamplanner_data.team[idx].previous_result.dynamic_role == dr_role) ||
			(teamplanner_data.team[idx].previous_result.previous_result_present == 1 && teamplanner_data.team[idx].previous_result.dynamic_role == dr_SWEEPER && dr_role == dr_DEFENDER) ||
		    (teamplanner_data.team[idx].previous_result.previous_result_present == 1 && teamplanner_data.team[idx].previous_result.dynamic_role == dr_DEFENDER && dr_role == dr_SWEEPER)) {
			MRA::Geometry::Point previousEndPos = MRA::Geometry::Point(teamplanner_data.team[idx].previous_result.end_position.x, teamplanner_data.team[idx].previous_result.end_position.y);
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

			if (fabs(totalCostBest - totalCostCurrent) < teamplanner_data.parameters.equality_cost_threshold) {
				if (dr_role == dr_INTERCEPTOR) {
					// for intercepter prefer the player closest to our goal
					MRA::Geometry::Point middleGoal = MRA::Geometry::Point(0, -teamplanner_data.fieldConfig.getMaxFieldY()+1.0); // 1 meter before own goal
					double distIdxToGoal = middleGoal.distanceTo(teamplanner_data.team[player_idx].position.getPosition().getPoint());
					double distBestToGoal = middleGoal.distanceTo(teamplanner_data.team[bestPlayer].position.getPosition().getPoint());
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
		teamplanner_data.team[idx].dynamic_role = dr_role;
		teamplanner_data.team[idx].result = PlayerPlannerResult();
		teamplanner_data.team[idx].result.gamestate = teamplanner_data.gamestate;
		teamplanner_data.team[idx].result.target = target;
		teamplanner_data.team[idx].result.defend_info = teamplanner_data.defend_info;
		teamplanner_data.team[idx].result.planner_target = planner_target;
		teamplanner_data.team[idx].result.target_position_is_end_position_of_pass = role_position_is_end_position_of_pass;
		teamplanner_data.team[idx].assigned = true;
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
void TeamPlay::assignGoalie(TeamPlannerData& teamplanner_data)
{
    auto fieldConfig = teamplanner_data.fieldConfig;
	double goalieYPosition = -fieldConfig.getMaxFieldY()+0.5;
	MRA::Geometry::Point goaliePosition = MRA::Geometry::Point(0, goalieYPosition); // center of the goal;
	if (teamplanner_data.gamestate == game_state_e::PARKING) {
		// select position closest to default goalie position as parking position for the goalie
		MRA::Geometry::Point startPost = MRA::Geometry::Point(0, -fieldConfig.getMaxFieldY());
		goaliePosition = RolePosition::closestTo(startPost, teamplanner_data.parking_positions);
	}
	else {
		if (teamplanner_data.ball_present) {
			MRA::Geometry::Point ballPos = teamplanner_data.ball.getPosition().getPoint();
			double penalty_area_half_width = fieldConfig.getPenaltyAreaWidth() * 0.5;
			if (ballPos.x < -penalty_area_half_width) {
				//			if (global) ball is to the left of the field (left from outer line penalty area) position keeper to the left.
				goaliePosition = MRA::Geometry::Point(-fieldConfig.getGoalWidth()*0.25, goalieYPosition); // left half of the goal;
			}
			else if (ballPos.x > penalty_area_half_width) {
				goaliePosition = MRA::Geometry::Point(+fieldConfig.getGoalWidth()*0.25, goalieYPosition); // right half of the goal;
			}
		}
	}

	unsigned keeper_idx = 0;
	bool keeperFound = false;

	for (unsigned int idx = 0; keeperFound == false && idx < teamplanner_data.team.size(); idx++) {
		if (teamplanner_data.team[idx].player_type == GOALIE) {
			keeperFound = true;
			keeper_idx = idx;
		}
	}

	if (!keeperFound && teamplanner_data.parameters.autoAssignGoalie) {
		// no dedicated goalie found and auto-assign goalie is enabled.

		//find preferred setplay kicker and receiver
		int setPlayKickerIdx = -1;
		int setPlayRecieverIdx = -1;
		int preferredKickerRobotId = teamplanner_data.parameters.preferredSetplayKicker;
		int preferredReceiverRobotId = teamplanner_data.parameters.preferredSetplayReceiver;
		vector<int> availablePlayers = vector<int>();

		if (preferredReceiverRobotId == preferredKickerRobotId)
		{
			preferredReceiverRobotId = 0; // avoid select conflict. prefer kicker above reciever.
		}

		for (unsigned int idx = 0; idx < teamplanner_data.team.size(); idx++) {
				// player can be assigned
				if (teamplanner_data.team[idx].robotId == preferredReceiverRobotId) {
					setPlayRecieverIdx = idx;
				}
				else if (teamplanner_data.team[idx].robotId == preferredKickerRobotId) {
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
			else if (teamplanner_data.team[player_idx].robotId <= teamplanner_data.team[lowestAvailablePlayerIdx].robotId) {
				lowestAvailablePlayerIdx = player_idx; // better goalie == lower robot_id
			}
		}
		keeper_idx = lowestAvailablePlayerIdx;
	}

	if (keeperFound) {
	    teamplanner_data.team[keeper_idx].assigned = true;
	    teamplanner_data.team[keeper_idx].result.target = goaliePosition;
	    teamplanner_data.team[keeper_idx].result.planner_target = teamplanner_data.gamestate == game_state_e::PARKING ? planner_target_e::GOTO_TARGET_POSITION_SLOW : planner_target_e::GOALIE;
	    teamplanner_data.team[keeper_idx].dynamic_role = teamplanner_data.gamestate == game_state_e::PARKING ? dr_PARKING : dr_GOALKEEPER;
		//cerr << keeper_idx <<": DR: " << DynamicRoleAsString(dr_GOALKEEPER) << " pos: " << goaliePosition.toString() << endl;
	}

	return;
}

//----------------------------------------------------------------------------------------
void TeamPlay::assignTooLongInPenaltyAreaPlayers(TeamPlannerData&  teamplanner_data)
{
	// max allowed time in penalty area in 2019: 10 seconds (attack and defense)
	const double TIME_TOO_LONG_IN_PENALTY_AREA_THRESHOLD = 8.0;  // 2 seconds to move outside the penalty area
	if (teamplanner_data.gamestate != NONE && teamplanner_data.gamestate != PARKING) {
		double d_x = teamplanner_data.fieldConfig.getPenaltyAreaWidth() * 0.5;
		double d_y = -(teamplanner_data.fieldConfig.FIELD_LENGTH/2) + teamplanner_data.fieldConfig.PENALTY_AREA_LENGTH;
		for (unsigned int idx = 0; idx < teamplanner_data.team.size(); idx++) {
			if (not teamplanner_data.team[idx].assigned) {

				if (teamplanner_data.team[idx].time_in_own_penalty_area > TIME_TOO_LONG_IN_PENALTY_AREA_THRESHOLD) {
					// player not assigned and too long in penalty area
					MRA::Geometry::Point pos = teamplanner_data.team[idx].position.getXYlocation();
					if (teamplanner_data.fieldConfig.isInOwnPenaltyArea(pos.x, pos.y)) {
						double d_y = -(teamplanner_data.fieldConfig.FIELD_LENGTH/2) + teamplanner_data.fieldConfig.PENALTY_AREA_LENGTH;
						MRA::Geometry::Point targetPos = pos;
						if (fabs(d_y - pos.y) < fabs(d_x - fabs(pos.x))) {
							// closest to Y line
							targetPos.y = d_y + teamplanner_data.fieldConfig.getRobotRadius();
						}
						else {
							// closest to side of penalty area
							double safe_x_pos = d_x + teamplanner_data.fieldConfig.getRobotRadius();
							if (pos.x > 0) {
								targetPos.x = safe_x_pos;
							}
							else {
								targetPos.x = -safe_x_pos;
							}
						}
						teamplanner_data.team[idx].assigned = true;
						teamplanner_data.team[idx].result.target = targetPos;
						teamplanner_data.team[idx].result.planner_target = SUPPORT_DEFENSE;
						teamplanner_data.team[idx].dynamic_role = dr_DEFENDER;
					}
				}
				if (teamplanner_data.team[idx].time_in_opponent_penalty_area > TIME_TOO_LONG_IN_PENALTY_AREA_THRESHOLD) { // TODO
					// player not assigned and too long in penalty area
					MRA::Geometry::Point pos = teamplanner_data.team[idx].position.getXYlocation();
					if (teamplanner_data.fieldConfig.isInOpponentPenaltyArea(pos.x, pos.y)) {
						double d_y2 = -d_y;
						MRA::Geometry::Point targetPos = pos;
						if (fabs(d_y2 - pos.y) < fabs(d_x - fabs(pos.x))) {
							// closest to Y line
							targetPos.y = d_y2 - teamplanner_data.fieldConfig.getRobotRadius();
						}
						else {
							// closest to side of penalty area
							double safe_x_pos = d_x + teamplanner_data.fieldConfig.getRobotRadius();
							if (pos.x > 0) {
								targetPos.x = safe_x_pos;
							}
							else {
								targetPos.x = -safe_x_pos;
							}
						}
						teamplanner_data.team[idx].assigned = true;
						teamplanner_data.team[idx].result.target = targetPos;
						teamplanner_data.team[idx].result.planner_target = SUPPORT_DEFENSE;
						teamplanner_data.team[idx].dynamic_role = dr_DEFENDER;
					}
				}
			}
		}
	}
}

//----------------------------------------------------------------------------------------
vector<MovingObject> TeamPlay::getTeamMates(const std::vector<TeamPlannerRobot>& team, unsigned meIdx, bool addAssignedTargetAsTeamPosition) {
	vector<MovingObject> teammates = vector<MovingObject>();
	for (unsigned int idx = 0; idx < team.size(); idx++) {
		if (idx != meIdx) {
			teammates.push_back(team[idx].position);
			if (addAssignedTargetAsTeamPosition && team[idx].assigned) {
				//add target position as barrier
				if (team[idx].result.path.size() > 0) {
					planner_piece_t last_piece = team[idx].result.path[team[idx].result.path.size()-1];
				    Position lastPos = Position(MRA::Geometry::Point(last_piece.x, last_piece.y));
				    teammates.push_back(MovingObject(lastPos));
				}
			}
		}
	}
 	return teammates;
}

//---------------------------------------------------------------------------------------------------------------------
bool TeamPlay::searchForBallBehaviorNeeded(TeamPlannerData& teamplanner_data) {
	bool searchForBall = false;
	if (not teamplanner_data.ball_present) {
		// no ball seen
		if ((teamplanner_data.gamestate != game_state_e::BEGIN_POSITION) && (teamplanner_data.gamestate != game_state_e::PARKING)) {
			searchForBall = true; // state requires search for ball
		}
	}
	else {
		// ball is valid, check if ball position is with the possible area during a game.
		if (not teamplanner_data.fieldConfig.isInReachableField(teamplanner_data.ball.getPosition().getPoint().x, teamplanner_data.ball.getPosition().getPoint().y)	) {
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
void TeamPlay::assignToFixedPositions(TeamPlannerData&  teamplanner_data, unsigned playerlist_idx, dynamic_role_e dynamic_role)
{
	vector<MRA::Geometry::Point> playerPositions = vector<MRA::Geometry::Point>();  //TODO start position must come from configuration file
	RolePosition::GetFixedPositions(playerPositions, teamplanner_data.gamestate,
	        teamplanner_data.ball, teamplanner_data.searchForBall, teamplanner_data.parking_positions, teamplanner_data.fieldConfig, teamplanner_data.parameters);

	planner_target_e planner_target = planner_target_e::GOTO_TARGET_POSITION;
	if (teamplanner_data.gamestate == game_state_e::PARKING) {
		planner_target = planner_target_e::GOTO_TARGET_POSITION_SLOW;
	}

	// use overwritten dynamic role: no impact only easier to follow for humans
	dynamic_role_e overwritten_dynamic_role = dynamic_role;
	if (teamplanner_data.searchForBall) {
		overwritten_dynamic_role = dynamic_role_e::dr_SEARCH_FOR_BALL;
	}
	if (teamplanner_data.gamestate == game_state_e::PARKING) {
		overwritten_dynamic_role = dynamic_role_e::dr_PARKING;
	}
	else if (teamplanner_data.gamestate == game_state_e::BEGIN_POSITION) {
		overwritten_dynamic_role = dynamic_role_e::dr_BEGIN_POSITION;
	}

	if (playerlist_idx < playerPositions.size()) {
		MRA::Geometry::Point rolePosition = playerPositions[playerlist_idx];
		assignAnyToPosition(teamplanner_data, playerlist_idx, overwritten_dynamic_role, rolePosition, planner_target, false);
	}

	return;
}

//---------------------------------------------------------------------------------------------------------------------
// print the inputs of TeamPlanner::Assign for debug purposes
void TeamPlay::printAssignInputs(TeamPlannerData&  teamplanner_data)
{
	cerr << "Team_Planner::assign  inputs:" << endl << flush;
	cerr << "game_state_e: " << teamplanner_data.gamestate << " (" << GameStateAsString(teamplanner_data.gamestate) << " )"<< endl << flush;
	cerr << "global ball: " << teamplanner_data.ball.toString() << endl << flush;
	for (unsigned idx = 0; idx < teamplanner_data.team.size(); idx++) {
		cerr << "Robot [" << idx << "] =" << endl << teamplanner_data.team[idx].toString() << endl;
	}
	cerr << "Opponents size: " << teamplanner_data.opponents.size()  << endl << flush;
	for (unsigned int i = 0; i < teamplanner_data.opponents.size(); i++) {
		cerr << "Opponents[" << i << "].position: "<< teamplanner_data.opponents[i].position.toString()  << endl << flush;
	}
	cerr << "plannerOptions: " << teamplanner_data.parameters.toString() << endl << flush;
	cerr << "parking positions size: " << teamplanner_data.parking_positions.size() << endl << flush;
	for (unsigned int idx = 0; idx < teamplanner_data.parking_positions.size(); idx++) {
		cerr << teamplanner_data.parking_positions[idx].toString() << endl << flush;
	}

	if (teamplanner_data.ball_pickup_position.valid)
		cerr << "pickup: valid, x:" << std::setprecision(2) << teamplanner_data.ball_pickup_position.x << " y: " << teamplanner_data.ball_pickup_position.y << " ts:" << teamplanner_data.ball_pickup_position.ts << endl << flush;
	else{
		cerr << "pickup: invalid " << endl << flush;
	}

	cerr << "passIsRequired: " << (teamplanner_data.passIsRequired ? "true" : "false") << endl << flush;

	if (teamplanner_data.pass_data.valid) {
		cerr << "pass_data: valid target_id: " << teamplanner_data.pass_data.target_id << endl << flush;
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

void TeamPlay::ReplanInterceptor(unsigned interceptorIdx,
        TeamPlannerData&  teamplanner_data)
{
	if (not teamplanner_data.ball_present) {
		return; // invalid ball, can not calculate new path
	}
	auto parameters = teamplanner_data.parameters;
	// this player is interceptor and game state is normal.
	// Replan: using dynamic path planner and ball.
	double shortestDistanceOpponentToBall = calculateShortestDistanceObjectsToTarget(getOpponents(teamplanner_data.opponents), teamplanner_data.ball);
	int nrDynamicPlannerIterations = teamplanner_data.parameters.nrDynamicPlannerIterations;
	double maxSpeed = teamplanner_data.parameters.maxPossibleLinearSpeed;
	double distRobotToBall = teamplanner_data.team[0].position.getPosition().getPoint().distanceTo(teamplanner_data.ball.getPosition().getPoint());
	// check if ball approach from best angle must be applied.
	if (distRobotToBall < shortestDistanceOpponentToBall + 1.0) {
		// always approach from outside when the robot is closer to the ball than any opponent + 1.0 meter
	    parameters.addBallApproachVertices = true;
	    parameters.distToapplyBallApproachVertices = teamplanner_data.fieldConfig.getFieldWidth();
	} else {
		// go straight to the ball
	    parameters.addBallApproachVertices = false;
	}

	vector<trs::Vertex> targetPos = vector<trs::Vertex>();
	targetPos.push_back(Vertex(teamplanner_data.ball.getPosition().getPoint(), 0));
	std::vector<planner_piece_t> path;

	vector<MovingObject> myTeam = getTeamMates(teamplanner_data.team, interceptorIdx, true);

	if (nrDynamicPlannerIterations <= 0) {
		bool avoidBallPath = false; // intercept should never avoid a passing ball.
		MRA::Geometry::Point BallTargePos;
		// use normal planner
		// when ball is obstacle or no iterations for dynamic planner is defined.
		GlobalPathPlanner visibilityGraph = GlobalPathPlanner(teamplanner_data.fieldConfig);
		visibilityGraph.setOptions(parameters);
		visibilityGraph.createGraph(teamplanner_data.team[interceptorIdx].position, teamplanner_data, targetPos, planner_target_e::GOTO_BALL, teamplanner_data.ballIsObstacle, avoidBallPath, BallTargePos);
		path = visibilityGraph.getShortestPath(teamplanner_data);
	} else {
		// use dynamic robot planner for goto ball
		GlobalPathDynamicPlanner dp = GlobalPathDynamicPlanner();
		path = dp.planPath(	teamplanner_data.team[interceptorIdx].position, teamplanner_data, targetPos,
				planner_target_e::GOTO_BALL, maxSpeed, nrDynamicPlannerIterations);

	}
	teamplanner_data.team[interceptorIdx].result.path = path;
}

////---------------------------------------------------------------------------------------------------------------------
// calculate shortest distance of the vector of objects (own or opponents) to the ball
double TeamPlay::calculateShortestDistanceObjectsToTarget(const std::vector<MovingObject>& objects, const MovingObject& targetObject) {
	double shortestDistance = std::numeric_limits<double>::infinity();
	for(unsigned int idx = 0; idx < objects.size(); idx++) {
		double distToBall = objects[idx].getPosition().getPoint().distanceTo(targetObject.getPosition().getPoint());
		if (distToBall < shortestDistance) {
			shortestDistance = distToBall;
		}
	}
	return shortestDistance;
}
