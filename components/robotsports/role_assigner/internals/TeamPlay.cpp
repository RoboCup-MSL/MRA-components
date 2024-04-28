/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */
#include "TeamPlay.hpp"
#include "TeamFormation.hpp"
#include "TeamPlannerExport.hpp"
#include "RolePosition.hpp"
#include "GlobalPathDynamicPlanner.hpp"
#include "SvgUtils.hpp"
#include "MathUtils.h"
#include <iomanip>
#include <iostream>
#include <limits>
#include <cmath>
#include <climits>
#include <algorithm>

using namespace std;
using namespace MRA;

const int THIS_PLAYER_TEAM_IDX = 0;

//---------------------------------------------------------------------------------------------------------------------
TeamPlay::TeamPlay() : m_gridFileNumber(0) {

}


void TeamPlay::assign(const TeamPlannerInput& input,
            TeamPlannerState& r_state,
            TeamPlannerOutput& r_output,
            const TeamPlannerParameters& parameters) {
	// convert to teamplannerData;
}

//---------------------------------------------------------------------------------------------------------------------
/*
 * Main method for dynamic role assignment.
 * Assign roles to team members based on game-situation and location of the players and the ball on the field.
 *
 * Internally the method will based on the game-situation call an other internal method which handles the specific game situation.
 */

std::vector<PlayerPlannerResult> TeamPlay::assign(TeamPlannerData& teamplannerData)
{
    std::vector<PlayerPlannerResult> player_paths;
	if (teamplannerData.gamestate != game_state_e::NONE) {
		// printAssignInputs(gamestate, ball, Team, Opponents, parameters,  parking_positions, ball_pickup_position, passIsRequired, pass_data);
	}
	if ((teamplannerData.gamestate == game_state_e::NONE) ||
			(teamplannerData.gamestate == game_state_e::YELLOW_CARD_AGAINST) ||
			(teamplannerData.gamestate == game_state_e::RED_CARD_AGAINST) ||
			(teamplannerData.gamestate == game_state_e::GOAL) ||
			(teamplannerData.gamestate == game_state_e::GOAL_AGAINST)) {
		// unhandled game situations: cards and goals are not passed to the team planner via the game state-machine
		for (unsigned idx = 0; idx < teamplannerData.team.size(); idx++) {
		    PlayerPlannerResult player_result = {};
		    player_result.gamestate = teamplannerData.gamestate;
		    player_result.dynamic_role = dynamic_role_e::dr_NONE;
		    player_result.defend_info.valid = false;

		    player_paths.push_back(player_result);
		}
		return player_paths; // no path will be planned if game state is NONE
	}

	bool playerPassedBall = false;
	for (unsigned r_idx = 0; r_idx < teamplannerData.team.size(); r_idx++) {
		if (teamplannerData.team[r_idx].passBall) {
			playerPassedBall = true;
			teamplannerData.playerWhoIsPassing = teamplannerData.team[r_idx].robotId;
		}
	}

	if (teamplannerData.gamestate == game_state_e::NORMAL) {
		if (teamplannerData.team_controls_ball) {
		    teamplannerData.gamestate = game_state_e::NORMAL_ATTACK;
		}
		else{
		    teamplannerData.gamestate = game_state_e::NORMAL_DEFEND;
		}
	}

	for (unsigned r_idx = 0; r_idx < teamplannerData.team.size(); r_idx++) {
	    teamplannerData.team[r_idx].result.gamestate = teamplannerData.gamestate;
	    teamplannerData.team[r_idx].result.dynamic_role = dynamic_role_e::dr_NONE;
	    teamplannerData.team[r_idx].result.defend_info.valid = false;
	}


	/* set avoid ball flag : only not avoiding during normal play */
	teamplannerData.ballIsObstacle = (teamplannerData.gamestate != game_state_e::NORMAL_ATTACK) && (teamplannerData.gamestate != game_state_e::NORMAL_DEFEND);

	// Assign first the Goalie
	assignGoalie(teamplannerData);

	// Assign players too long in any penalty area: closest position out of the penalty area
	assignTooLongInPenaltyAreaPlayers(teamplannerData);


	teamplannerData.searchForBall = searchForBallBehaviorNeeded(teamplannerData);

    if ( teamplannerData.gamestate == game_state_e::BEGIN_POSITION || teamplannerData.gamestate ==  game_state_e::PARKING )
    {
        // Fixed position assignment when there is no order in position importance
        assignToFixedPositions(teamplannerData);
	}
    else {
        vector<Geometry::Point> fixedPlayerPositions = vector<Geometry::Point>();
        RolePosition::GetFixedPositions(fixedPlayerPositions, teamplannerData);
        for (unsigned dr_idx = 0; dr_idx < teamplannerData.teamFormation.size(); dr_idx++) {
            if (dr_idx >= teamplannerData.team.size()) {
                // can not assign more roles than team members.
                break;
            }

            planner_target_e planner_target = determine_planner_target(teamplannerData.teamFormation[dr_idx], teamplannerData.gamestate);
            bool role_position_is_end_position_of_pass = false;

            Geometry::Point rolePosition;
            if (   teamplannerData.searchForBall
                   || teamplannerData.gamestate ==  game_state_e::KICKOFF
                   || teamplannerData.gamestate ==  game_state_e::KICKOFF_AGAINST) {
                    // get list with player positions
                rolePosition = fixedPlayerPositions[dr_idx];
                planner_target = planner_target_e::GOTO_TARGET_POSITION;
            }
            else {
                rolePosition = RolePosition::determineDynamicRolePosition(teamplannerData.defend_info, planner_target, m_gridFileNumber, teamplannerData.teamFormation[dr_idx],
                        teamplannerData, playerPassedBall, role_position_is_end_position_of_pass);
            }


            //  Determine path for assigned role
            assignAnyToPosition(teamplannerData, static_cast<int>(dr_idx), teamplannerData.teamFormation[dr_idx], rolePosition, planner_target, role_position_is_end_position_of_pass);

            //cerr << dr_idx <<": DR: " << DynamicRoleAsString(teamplannerData.teamFormation[dr_idx]) << " pos: " << rolePosition.toString() << endl;

            // stop if current player has ball
            if (teamplannerData.team[THIS_PLAYER_TEAM_IDX].assigned && teamplannerData.parameters.calculateAllPaths == false) {
                // path for this robot (player nr: 0) is found and not all paths must be calculated.
                break;
            }
        }
    }

	// ASSIGN ALWAYS A ROLE TO THE AVAIALABLE PLAYERS
	if (teamplannerData.team[THIS_PLAYER_TEAM_IDX].assigned == false || teamplannerData.parameters.calculateAllPaths)
	{
		// current player does not have any role or all players must be assigned.
		// assign defender role to all unassigned players
		for (unsigned ap_idx = 0; ap_idx < teamplannerData.team.size(); ap_idx++) {
			if (teamplannerData.team[ap_idx].assigned == false)  {
				//	Determine path for assigned role
				planner_target_e planner_target = determine_planner_target(dynamic_role_e::dr_DEFENDER, teamplannerData.gamestate);
				bool role_position_is_end_position_of_pass = false;
				Geometry::Point rolePosition = RolePosition::determineDynamicRolePosition(teamplannerData.defend_info, planner_target, m_gridFileNumber, dynamic_role_e::dr_DEFENDER,
				        teamplannerData, playerPassedBall, role_position_is_end_position_of_pass);

				assignAnyToPosition(teamplannerData, static_cast<int>(ap_idx), dynamic_role_e::dr_DEFENDER, rolePosition, planner_target, role_position_is_end_position_of_pass);
			}
		}
	}

	Geometry::Point BallTargetPos = Geometry::Point(teamplannerData.pass_data.target_pos.x, teamplannerData.pass_data.target_pos.y);
	// Calculate Robot Planner path
	for (unsigned idx = 0; idx < teamplannerData.team.size(); idx++) {
		if (teamplannerData.team[idx].assigned)  {
			// calculate path for robot.

			bool avoidBallPath = (teamplannerData.gamestate == game_state_e::NORMAL_ATTACK) and teamplannerData.pass_data.valid; // avoid ball path only if pass made (or shot on goal) during normal play (normal_attack)
			if (teamplannerData.team[idx].result.target_position_is_end_position_of_pass)
			{
				// in case of pass, don't avoid ball path if player is destination of the pass
				avoidBallPath = false;
			}

			vector<Geometry::Position> myTeam = getTeamMates(teamplannerData.team, idx, true);
			GlobalPathPlanner visibilityGraph = GlobalPathPlanner(teamplannerData.fieldConfig); // create robot planner
			visibilityGraph.setOptions(teamplannerData.parameters);
			// create list of possible targets for robot-planner
			std::vector<MRA::Vertex> targetPos = vector<MRA::Vertex>();

            // check if role position is in allowed, if not allowed update position to an allowed positions
			teamplannerData.team[idx].result.target = updatePositionIfNotAllowed(teamplannerData.team[idx].position, teamplannerData.team[idx].dynamic_role, teamplannerData.team[idx].result.target, teamplannerData.fieldConfig);

			targetPos.push_back(Vertex(teamplannerData.team[idx].result.target, 0));
			visibilityGraph.createGraph(teamplannerData.team[idx].position, teamplannerData.team[idx].velocity, teamplannerData,
					targetPos, teamplannerData.team[idx].result.planner_target, avoidBallPath, BallTargetPos);
			teamplannerData.team[idx].result.path = visibilityGraph.getShortestPath(teamplannerData);
		}
		if (teamplannerData.parameters.calculateAllPaths == false) {
			// path for this robot (player nr: 0) is found and not all paths must be calculated.
			break;
		}
	}

	if (teamplannerData.gamestate == NORMAL_DEFEND) {
		// replan interceptor with local ball only if interceptor is not performing a priority block
		int interceptorIdx = -1;
		for (unsigned idx = 0; idx < teamplannerData.team.size(); idx++) {
			if (teamplannerData.team[idx].assigned)  {
				if (teamplannerData.team[idx].dynamic_role == dr_INTERCEPTOR) {
					interceptorIdx = idx;
				}

			}
		}

		bool replan = false;
		if ((interceptorIdx == 0) || teamplannerData.parameters.calculateAllPaths) {
			replan = (interceptorIdx != -1) && (teamplannerData.team[interceptorIdx].result.path.size() > 0) && (teamplannerData.team[interceptorIdx].result.path[THIS_PLAYER_TEAM_IDX].target != PRIORITY_BLOCK);
		}
		if (replan && interceptorIdx >= 0) {
		    ReplanInterceptor(interceptorIdx, teamplannerData);
		}
	}

	// loop over the player path to verify if path of this player stay within the allowed boundaries (field+safety area)
	bool thisPlayerHasUnallowedPath = false;
	bool thisPlayerStartsAtUnallowedPosition = false;
	for (unsigned idx = 0; idx < teamplannerData.team.size(); idx++) {
        PlayerPlannerResult player_result = {};
		if (teamplannerData.team[idx].assigned) {
			bool pathOK = stayPathWithinBoundaries(teamplannerData.fieldConfig, teamplannerData.team[idx].result);
			player_result = teamplannerData.team[idx].result;
			player_result.gamestate = teamplannerData.team[idx].result.gamestate;
			player_result.dynamic_role = teamplannerData.team[idx].dynamic_role;
			if (not pathOK && idx == 0) { // CHECK Only this robot
				thisPlayerHasUnallowedPath = true; // this robot has wrong path

				// check if player starts at position in the field
				auto currentPos = teamplannerData.team[idx].position;
				thisPlayerStartsAtUnallowedPosition = !teamplannerData.fieldConfig.isInReachableField(currentPos.x, currentPos.y);
			}
		}
		player_paths.push_back(player_result);
	}

	bool dynamicRoleNoneAssigned =  teamplannerData.team[THIS_PLAYER_TEAM_IDX].dynamic_role == dr_NONE;
	// Dump diagnostics if any robot moves outside the allowed field (safety border + field)
	if (thisPlayerHasUnallowedPath || teamplannerData.parameters.svgOutputFileName.length() > 0 || dynamicRoleNoneAssigned) {
		// Create diagnostics file (.svg with comments).
		// file can be used to diagnose with path to illegal locations are planned
		std::vector<player_type_e> teamTypes = std::vector<player_type_e>();
		std::vector<long> robotIds = std::vector<long>();

		for (unsigned r_idx = 0; r_idx < teamplannerData.team.size(); r_idx++) {
			robotIds.push_back(teamplannerData.team[r_idx].robotId);
			teamTypes.push_back(teamplannerData.team[r_idx].player_type);
		}
        TeamPlannerParameters parameters = teamplannerData.parameters;
		std::string save_name = parameters.svgOutputFileName;
		if (thisPlayerHasUnallowedPath) {
			if (thisPlayerStartsAtUnallowedPosition) {
			    save_name = GetTeamPlannerSVGname(teamplannerData.gamestate, "OUTSIDE_FIELD_BEGIN_ERROR");
			}
			else {
			    save_name = GetTeamPlannerSVGname(teamplannerData.gamestate, "OUTSIDE_FIELD_END_ERROR");
			}
		}
		if (dynamicRoleNoneAssigned)
		{
		    save_name = GetTeamPlannerSVGname(teamplannerData.gamestate, "DYN_ROLE_NONE");
		}

		SvgUtils::plannerdata_to_svg(player_paths, teamplannerData, teamplannerData.fieldConfig, save_name);

		// create empty path for robot which path ends outside the field.
		if (thisPlayerHasUnallowedPath and not thisPlayerStartsAtUnallowedPosition) {
		    std::vector<planner_piece_t> path = player_paths[THIS_PLAYER_TEAM_IDX].path;
		    if (not path.empty()) {
                auto begin_position = path[0];
		        auto end_position = path[path.size()-1];
		        bool create_empty_result = true;

		        if (teamplannerData.fieldConfig.isInReachableField(begin_position.x, begin_position.y)
		            and not teamplannerData.fieldConfig.isInReachableField(end_position.x,end_position.y)) {
		            // end position and begin position are in reachable field, but a position in the path is unreachable
		            create_empty_result = false;
		        }

		        if (create_empty_result) {
		            player_paths[THIS_PLAYER_TEAM_IDX] = PlayerPlannerResult();
		        }
		    }
		}
	}

	//printAssignOutputs(Team, player_paths);
	return player_paths;
}

//---------------------------------------------------------------------------------------------------------------------
// check if the given path is always within the boundaries
bool TeamPlay::stayPathWithinBoundaries(const FieldConfig& fieldConfig, const PlayerPlannerResult& player_path) {
	bool ok = true;
	// check path-pieces
	for (auto it = player_path.path.begin(); it != player_path.path.end(); ++it ) {
		// abs(x) < allowed-x
		if (not fieldConfig.isInReachableField(it->x, it->y)) {
			ok = false;
		}
	}
	return ok;
}
//---------------------------------------------------------------------------------------------------------------------
// Print the output of TeamPlanner::Assign (debug purposes)
void TeamPlay::printAssignOutputs(const std::vector<TeamPlannerRobot>& Team, const std::vector<PlayerPlannerResult>&  player_paths)
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
		target = planner_target_e::GOALIE_POSITION;
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
vector<Geometry::Position> TeamPlay::getOpponents(const std::vector<TeamPlannerOpponent>&  Opponents) {
	vector<Geometry::Position> opponents = vector<Geometry::Position>();
	for (unsigned int idx = 0; idx < Opponents.size(); idx++) {
		opponents.push_back(Opponents[idx].position);
	}
	return opponents;
}

bool TeamPlay::AssignAnyRobotPreferedSetPlayer(TeamPlannerData&  teamplanner_data, dynamic_role_e dr_role, planner_target_e planner_target, const Geometry::Point& targetPos) {
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
	    teamplanner_data.team[foundPlayer].result.target = targetPos;
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
            const Geometry::Point& target, planner_target_e planner_target, bool role_position_is_end_position_of_pass)
{

    // cerr << "ROLE-NR: " <<  role_idx << " : " << DynamicRoleAsString(dr_role) << " - AssignedToAnyPosition to target: " <<   target.toString() << endl;
	vector<Geometry::Point> targets = vector<Geometry::Point>();
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
	Geometry::Point currentEndPos = target;  // intended target, can be slightly different than calculated (if path is blocked);
	// do all criteria calculations for the team and store it in AssignToTargetData structs

	for (unsigned int idx = 0; idx < teamplanner_data.team.size(); idx++) {
		if (teamplanner_data.team[idx].assigned) {
			continue;  // player already assigned, skip
		}
		if (dr_role == dr_BALLPLAYER && teamplanner_data.team[idx].controlBall == false) {
			continue; // dribble can only be done, by player with ball, so skip this robot
		}
		if (dr_role == dr_INTERCEPTOR && teamplanner_data.team[idx].passBall) {
			continue; // ball should not be intercepted by player who passed
		}

		AssignToTargetData data;
		data.team_idx = idx;   // id/position of robot in the Team-vector
		data.robotId = teamplanner_data.team[idx].robotId;  // robotId (robot number)

		// Calculate full stop position, in the direction of current velocity vector
		Geometry::Point linVelocity(teamplanner_data.team[idx].velocity.x, teamplanner_data.team[idx].velocity.y);

		double acc = teamplanner_data.parameters.maxPossibleLinearAcceleration;
		double vel = linVelocity.size();
		double stopDistance = acc == 0 ? 0 : 1/2 * vel * vel / acc;
		MRA::Geometry::Point  stopDist(linVelocity);
		stopDist.normalize();
		stopDist *= stopDistance;
		MRA::Geometry::Point fullStopPos = teamplanner_data.team[idx].position;
		fullStopPos += stopDist;

		// calculate distance from robot's full stop position to target
		data.distToTarget = currentEndPos.distanceTo(fullStopPos);

		// calculate previous target position distance-threshold.
		if ((teamplanner_data.team[idx].previous_result.previous_result_present == 1 && teamplanner_data.team[idx].previous_result.dynamic_role == dr_role) ||
			(teamplanner_data.team[idx].previous_result.previous_result_present == 1 && teamplanner_data.team[idx].previous_result.dynamic_role == dr_SWEEPER && dr_role == dr_DEFENDER) ||
		    (teamplanner_data.team[idx].previous_result.previous_result_present == 1 && teamplanner_data.team[idx].previous_result.dynamic_role == dr_DEFENDER && dr_role == dr_SWEEPER)) {
			Geometry::Point previousEndPos = Geometry::Point(teamplanner_data.team[idx].previous_result.end_position.x, teamplanner_data.team[idx].previous_result.end_position.y);
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
					Geometry::Point middleGoal = Geometry::Point(0, -teamplanner_data.fieldConfig.getMaxFieldY()+1.0); // 1 meter before own goal
					double distIdxToGoal = middleGoal.distanceTo(teamplanner_data.team[player_idx].position);
					double distBestToGoal = middleGoal.distanceTo(teamplanner_data.team[bestPlayer].position);
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
	double goalieYPosition = -teamplanner_data.fieldConfig.getMaxFieldY()+0.5;
	Geometry::Point goaliePosition = Geometry::Point(0, goalieYPosition); // center of the goal;
	if (teamplanner_data.gamestate == game_state_e::PARKING) {
		// select position closest to default goalie position as parking position for the goalie
		Geometry::Point startPost = Geometry::Point(0, -teamplanner_data.fieldConfig.getMaxFieldY());
		goaliePosition = RolePosition::closestTo(startPost, teamplanner_data.parking_positions);
	}
	else {
		if (teamplanner_data.ball.is_valid) {
			Geometry::Position ballPos = teamplanner_data.ball.position;
			double penalty_area_half_width = teamplanner_data.fieldConfig.getPenaltyAreaWidth() * 0.5;
			if (ballPos.x < -penalty_area_half_width) {
				//			if (global) ball is to the left of the field (left from outer line penalty area) position keeper to the left.
				goaliePosition = Geometry::Point(-teamplanner_data.fieldConfig.getGoalWidth()*0.25, goalieYPosition); // left half of the goal;
			}
			else if (ballPos.x > penalty_area_half_width) {
				goaliePosition = Geometry::Point(+teamplanner_data.fieldConfig.getGoalWidth()*0.25, goalieYPosition); // right half of the goal;
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

	if (not keeperFound && teamplanner_data.parameters.autoAssignGoalie) {
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
	    teamplanner_data.team[keeper_idx].result.planner_target = teamplanner_data.gamestate == game_state_e::PARKING ? planner_target_e::GOTO_TARGET_POSITION_SLOW : planner_target_e::GOALIE_POSITION;
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
		double d_y = -(teamplanner_data.fieldConfig.getFieldLength()/2) + teamplanner_data.fieldConfig.getPenaltyAreaLength();
		for (unsigned int idx = 0; idx < teamplanner_data.team.size(); idx++) {
			if (not teamplanner_data.team[idx].assigned) {

				if (teamplanner_data.team[idx].time_in_own_penalty_area > TIME_TOO_LONG_IN_PENALTY_AREA_THRESHOLD) {
					// player not assigned and too long in penalty area
					Geometry::Point pos = teamplanner_data.team[idx].position;
					if (teamplanner_data.fieldConfig.isInOwnPenaltyArea(pos.x, pos.y)) {
						double d_y = -(teamplanner_data.fieldConfig.getFieldLength()/2) + teamplanner_data.fieldConfig.getPenaltyAreaLength();
						Geometry::Point targetPos = pos;
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
					Geometry::Point pos = teamplanner_data.team[idx].position;
					if (teamplanner_data.fieldConfig.isInOpponentPenaltyArea(pos.x, pos.y)) {
						double d_y2 = -d_y;
						Geometry::Point targetPos = pos;
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
vector<Geometry::Position> TeamPlay::getTeamMates(const std::vector<TeamPlannerRobot>& Team, unsigned meIdx, bool addAssignedTargetAsTeamPosition) {
	vector<Geometry::Position> teammates = vector<Geometry::Position>();
	for (unsigned int idx = 0; idx < Team.size(); idx++) {
		if (idx != meIdx) {
			teammates.push_back(Team[idx].position);
			if (addAssignedTargetAsTeamPosition && Team[idx].assigned) {
				//add target position as barrier
				if (Team[idx].result.path.size() > 0) {
					planner_piece_t last_piece = Team[idx].result.path[Team[idx].result.path.size()-1];
					Geometry::Position lastPos = Geometry::Position(last_piece.x, last_piece.y);
				    teammates.push_back(lastPos);
				}
			}
		}
	}
 	return teammates;
}

//---------------------------------------------------------------------------------------------------------------------
bool TeamPlay::searchForBallBehaviorNeeded(TeamPlannerData& teamplanner_data) {
	bool searchForBall = false;
	if (not teamplanner_data.ball.is_valid) {
		// no ball seen
		if ((teamplanner_data.gamestate != game_state_e::BEGIN_POSITION) && (teamplanner_data.gamestate != game_state_e::PARKING)) {
			searchForBall = true; // state requires search for ball
		}
	}
	else {
		// ball is valid, check if ball position is with the possible area during a game.
		if (not teamplanner_data.fieldConfig.isInReachableField(teamplanner_data.ball.position.x, teamplanner_data.ball.position.y)	) {
			// ball is outside field (not in field and not in safety area)
			searchForBall = true; // state requires search for ball
		}
	}
	return searchForBall;
}

//---------------------------------------------------------------------------------------------------------------------
// print the inputs of TeamPlanner::Assign for debug purposes
void TeamPlay::printAssignInputs(const TeamPlannerInput& teamplanner_input, const TeamPlannerParameters& parameters)
{
	cerr << "Team_Planner::assign  inputs:" << endl << flush;
	cerr << "game_state_e: " << teamplanner_input.gamestate << " (" << GameStateAsString(teamplanner_input.gamestate) << " )"<< endl << flush;
	cerr << "global ball: " << teamplanner_input.ball.toString() << endl << flush;
	for (unsigned idx = 0; idx < teamplanner_input.team.size(); idx++) {
		cerr << "Robot [" << idx << "] =" << endl << teamplanner_input.team[idx].toString() << endl;
	}
	cerr << "Opponents size: " << teamplanner_input.opponents.size()  << endl << flush;
	for (unsigned int i = 0; i < teamplanner_input.opponents.size(); i++) {
		cerr << "Opponents[" << i << "].position: "<< teamplanner_input.opponents[i].position.toString()  << endl << flush;
	}
	cerr << "plannerParameters: " << parameters.toString() << endl << flush;
	cerr << "parking positions size: " << teamplanner_input.parking_positions.size() << endl << flush;
	for (unsigned int idx = 0; idx < teamplanner_input.parking_positions.size(); idx++) {
		cerr << teamplanner_input.parking_positions[idx].toString() << endl << flush;
	}

	if (teamplanner_input.ball_pickup_position.valid)
		cerr << "pickup: valid, x:" << std::setprecision(2) << teamplanner_input.ball_pickup_position.x << " y: " << teamplanner_input.ball_pickup_position.y << " ts:" << teamplanner_input.ball_pickup_position.ts << endl << flush;
	else{
		cerr << "pickup: invalid " << endl << flush;
	}

	cerr << "passIsRequired: " << (teamplanner_input.passIsRequired ? "true" : "false") << endl << flush;

	if (teamplanner_input.pass_data.valid) {
		cerr << "pass_data: valid target_id: " << teamplanner_input.pass_data.target_id << endl << flush;
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

void TeamPlay::ReplanInterceptor(unsigned interceptorIdx, TeamPlannerData&  teamplanner_data)
{
	if (teamplanner_data.ball.is_valid == false) {
		return; // invalid ball, can not calculate new path
	}
	// this player is interceptor and game state is normal.
	// Replan: using dynamic path planner and local ball.
	double shortestDistanceOpponentToBall = calculateShortestDistanceObjectsToTarget(getOpponents(teamplanner_data.opponents), teamplanner_data.ball.position);
	int nrDynamicPlannerIterations = teamplanner_data.parameters.nrDynamicPlannerIterations;
	double maxSpeed = teamplanner_data.parameters.maxPossibleLinearSpeed;
	double distRobotToBall = teamplanner_data.team[THIS_PLAYER_TEAM_IDX].position.distanceTo(teamplanner_data.ball.position);
	// check if ball approach from best angle must be applied.
	if (distRobotToBall < shortestDistanceOpponentToBall + 1.0) {
		// always approach from outside when the robot is closer to the ball than any opponent + 1.0 meter
	    teamplanner_data.parameters.addBallApproachVertices = true;
		teamplanner_data.parameters.distToapplyBallApproachVertices = teamplanner_data.fieldConfig.getFieldWidth();
	} else {
		// go straight to the ball
	    teamplanner_data.parameters.addBallApproachVertices = false;
	}

	vector<MRA::Vertex> targetPos = vector<MRA::Vertex>();
	targetPos.push_back(Vertex(teamplanner_data.ball.position, 0));
	std::vector<planner_piece_t> path;

	vector<Geometry::Position> myTeam = getTeamMates(teamplanner_data.team, interceptorIdx, true);

	if (nrDynamicPlannerIterations <= 0) {
		bool avoidBallPath = false; // intercept should never avoid a passing ball.
		Geometry::Point BallTargetPos;
		// use normal planner
		// when ball is obstacle or no iterations for dynamic planner is defined.
		GlobalPathPlanner visibilityGraph = GlobalPathPlanner(teamplanner_data.fieldConfig);
		visibilityGraph.setOptions(teamplanner_data.parameters);
		vector<Geometry::Position> myTeam = getTeamMates(teamplanner_data.team, interceptorIdx, false);
		visibilityGraph.createGraph(teamplanner_data.team[interceptorIdx].position, teamplanner_data.team[interceptorIdx].velocity, teamplanner_data,
				targetPos, planner_target_e::GOTO_BALL, avoidBallPath, BallTargetPos);
		path = visibilityGraph.getShortestPath(teamplanner_data);

	} else {
		// use dynamic robot planner for goto ball
		GlobalPathDynamicPlanner dp = GlobalPathDynamicPlanner();
		path = dp.planPath(	teamplanner_data.team[interceptorIdx].position, teamplanner_data.team[interceptorIdx].velocity, teamplanner_data, targetPos,
				planner_target_e::GOTO_BALL, maxSpeed, nrDynamicPlannerIterations);
	}

    if (not path.empty()) {
        Geometry::Point original_role_position(path[path.size()-1].x, path[path.size()-1].y);
        // check if role position is in allowed, if not allowed update position to an allowed positions
        Geometry::Point role_position = updatePositionIfNotAllowed(teamplanner_data.team[interceptorIdx].position,
                                                            teamplanner_data.team[interceptorIdx].dynamic_role,  original_role_position, teamplanner_data.fieldConfig);

        vector<MRA::Vertex> roleTargetPos = vector<MRA::Vertex>();
        roleTargetPos.push_back(Vertex(role_position, 0));

        bool avoidBallPath = false; // intercept should never avoid a passing ball.
        Geometry::Point BallTargetPos;
        GlobalPathPlanner visibilityGraph = GlobalPathPlanner(teamplanner_data.fieldConfig);
        visibilityGraph.setOptions(teamplanner_data.parameters);
        visibilityGraph.createGraph(teamplanner_data.team[interceptorIdx].position, teamplanner_data.team[interceptorIdx].velocity, teamplanner_data,
                roleTargetPos, planner_target_e::GOTO_BALL, avoidBallPath, BallTargetPos);
        path = visibilityGraph.getShortestPath(teamplanner_data);
    }
    teamplanner_data.team[interceptorIdx].result.path = path;
}

////---------------------------------------------------------------------------------------------------------------------
// calculate shortest distance of the vector of objects (own or opponents) to the ball
double TeamPlay::calculateShortestDistanceObjectsToTarget(const std::vector<Geometry::Position>& objects, const Geometry::Position& targetObject) {
	double shortestDistance = std::numeric_limits<double>::infinity();
	for(unsigned int idx = 0; idx < objects.size(); idx++) {
		double distToBall = Geometry::Point(objects[idx].x, objects[idx].y).distanceTo(Geometry::Point(targetObject.x, targetObject.y));
		if (distToBall < shortestDistance) {
			shortestDistance = distToBall;
		}
	}
	return shortestDistance;
}

void TeamPlay::assignToFixedPositions(TeamPlannerData& teamplanner_data) {
    // assign the players to the list of fixed positions.
    //
    // Step 1: re-order the list
    //    To avoid duplicate target positions this player will select it previous target position as
    //    first in the list in case it is not claimed by other player with lower-id. (lower-id get claim in case of duplicate claiming)
    //    positions claimed by other players will be added to the end of the list. This player will first try to move to unclaimed positions
    //    When order is not relevant for the fixed positions like parking  and begin position, then closest position to this player is preferred.
    // Step 2: loop over the re-ordered list and and closest robot to each target-position (till this player is assigned or all players are assigned (depends on option).
    //

    // Fixed position assignment for field players
    planner_target_e planner_target = planner_target_e::GOTO_TARGET_POSITION;
    if (teamplanner_data.gamestate == game_state_e::PARKING) {
        planner_target = planner_target_e::GOTO_TARGET_POSITION_SLOW;
    }
    // get list with player positions
    vector<Geometry::Point> fixedPlayerPositions = vector<Geometry::Point>();
    RolePosition::GetFixedPositions(fixedPlayerPositions, teamplanner_data);
    // first add position which is player claimed to playerPositions-vector
    // other positions are added to the notByThisPlayerClaimedPositions-vector
    vector<Geometry::Point> playerPositions = vector<Geometry::Point>();
    vector<Geometry::Point> notByThisPlayerClaimedPositions = vector<Geometry::Point>();
    vector<Geometry::Point> previousTeamEndPositions = vector<Geometry::Point>();
    vector<Geometry::Point> availablePlayersPositions = vector<Geometry::Point>();

    bool own_previous_position_added = false;

    if ((teamplanner_data.team[THIS_PLAYER_TEAM_IDX].previous_result.previous_result_present == 1)) {
        Geometry::Point end_pos = Geometry::Point(teamplanner_data.team[THIS_PLAYER_TEAM_IDX].previous_result.end_position.x,
                teamplanner_data.team[THIS_PLAYER_TEAM_IDX].previous_result.end_position.y);
        // this player has a previous end position

        const double SAME_POSITION_THRESHOLD = 0.1;
        for (unsigned dr_idx = 0; dr_idx < fixedPlayerPositions.size(); dr_idx++) {
            if (fixedPlayerPositions[dr_idx].distanceTo(end_pos) < SAME_POSITION_THRESHOLD) {
                // previous end  position from this player is very close to the fixed position

                // check if no other player goes to same position
                bool claimed_by_teammate_with_higher_prio = false;
                for (unsigned team_idx = THIS_PLAYER_TEAM_IDX + 1; team_idx < teamplanner_data.team.size(); team_idx++) {
                    if (teamplanner_data.team[team_idx].previous_result.previous_result_present == 1) {
                        Geometry::Point end_pos_teammate = Geometry::Point(teamplanner_data.team[team_idx].previous_result.end_position.x,
                                                             teamplanner_data.team[team_idx].previous_result.end_position.y);
                        if (end_pos_teammate.distanceTo(end_pos) < SAME_POSITION_THRESHOLD) {
                            // position claimed by teammate. Player with lowest id wins this position
                            claimed_by_teammate_with_higher_prio = teamplanner_data.team[team_idx].robotId < teamplanner_data.team[THIS_PLAYER_TEAM_IDX].robotId;
                        }
                    }
                }
                if (not claimed_by_teammate_with_higher_prio) {
                    playerPositions.push_back(fixedPlayerPositions[dr_idx]);
                    own_previous_position_added = true;
                }
                else {
                    previousTeamEndPositions.push_back(fixedPlayerPositions[dr_idx]);
                }
            } else {
                // previous end  position from this player is NOT very close to the fixed position
                notByThisPlayerClaimedPositions.push_back(fixedPlayerPositions[dr_idx]);
            }
        }
    } else {
        // this player did not claimed a position
        notByThisPlayerClaimedPositions = fixedPlayerPositions;
    }

    // collect positions which were previous end position of a team member.
    for (unsigned dr_idx = 0; dr_idx < notByThisPlayerClaimedPositions.size(); dr_idx++) {
        bool claimed_by_teammate = false;
        for (unsigned team_idx = THIS_PLAYER_TEAM_IDX + 1; team_idx < teamplanner_data.team.size(); team_idx++) {
            if (teamplanner_data.team[team_idx].previous_result.previous_result_present == 1) {
                Geometry::Point end_pos = Geometry::Point(teamplanner_data.team[team_idx].previous_result.end_position.x,
                        teamplanner_data.team[team_idx].previous_result.end_position.y);
                if (notByThisPlayerClaimedPositions[dr_idx].distanceTo(end_pos) < 0.1) {
                    claimed_by_teammate = true;
                    break;
                }
            }
        }
        if (claimed_by_teammate) {
            // position was previous end position of a teammate
            previousTeamEndPositions.push_back(notByThisPlayerClaimedPositions[dr_idx]);
        } else {
            availablePlayersPositions.push_back(notByThisPlayerClaimedPositions[dr_idx]);
        }
    }

    // add closest end position to playersPosition vector
    // when game-situation is BEGIN_POSITION or PARKING: all fixed postions are equal important.
    if ((availablePlayersPositions.size() > 0) && (!own_previous_position_added)
            && (teamplanner_data.gamestate == game_state_e::BEGIN_POSITION || teamplanner_data.gamestate == game_state_e::PARKING)) {
        // find closest position for current player
        double closestDistance = 0;
        int closestIdx = -1;
        Geometry::Point mePos = teamplanner_data.team[THIS_PLAYER_TEAM_IDX].position;
        for (unsigned dr_idx = 0; dr_idx < availablePlayersPositions.size(); dr_idx++) {
            auto dist = mePos.distanceTo(availablePlayersPositions[dr_idx]);
            if ((closestIdx == -1) || (dist < closestDistance)) {
                closestIdx = dr_idx;
                closestDistance = dist;
            }
        }
        if (closestIdx >= 0) {
            // add closest position to playersPosition vector
            playerPositions.push_back(availablePlayersPositions[closestIdx]);
        }
        // add other available positions to playersPosition vector
        for (int dr_idx = 0; dr_idx < (int) (availablePlayersPositions.size()); dr_idx++) {
            if (dr_idx != closestIdx) {
                playerPositions.push_back(availablePlayersPositions[dr_idx]);
            }
        }
    } else {
        // add available positions to playersPosition vector
        std::copy(availablePlayersPositions.begin(), availablePlayersPositions.end(),
                std::back_inserter(playerPositions));
    }
    // add previous end positions of team members to playersPosition vector
    std::copy(previousTeamEndPositions.begin(), previousTeamEndPositions.end(), std::back_inserter(playerPositions));

    // playersPosition vector is completely filled.
    if (fixedPlayerPositions.size() != playerPositions.size()) {
        cerr << "--> ERROR: fixedPlayerPositions.size() = " << fixedPlayerPositions.size() << endl;
        cerr << "ERROR: playerPositions.size() = " << playerPositions.size() << endl;
        cerr << "ERROR: previousTeamEndPositions.size() = " << previousTeamEndPositions.size() << endl;
        cerr << "ERROR: availablePlayersPositions.size() = " << availablePlayersPositions.size() << endl;
        cerr << "ERROR: notByThisPlayerClaimedPositions.size() = " << notByThisPlayerClaimedPositions.size() << endl;
        return;
    }


    // loop over the playersPosition vector (sorted on importance) and assign players to the positions
    for (unsigned dr_idx = 0; dr_idx < playerPositions.size(); dr_idx++) {
        if (dr_idx >= teamplanner_data.team.size()) {
            // can not assign more roles than team members.
            break;
        }
        // use overwritten dynamic role: no impact only easier to follow for humans
        dynamic_role_e dynamic_role = teamplanner_data.teamFormation[dr_idx];
        if (teamplanner_data.searchForBall) {
            dynamic_role = dynamic_role_e::dr_SEARCH_FOR_BALL;
        }
        if (teamplanner_data.gamestate == game_state_e::PARKING) {
            dynamic_role = dynamic_role_e::dr_PARKING;
        } else if (teamplanner_data.gamestate == game_state_e::BEGIN_POSITION) {
            dynamic_role = dynamic_role_e::dr_BEGIN_POSITION;
        }

        bool role_position_is_end_position_of_pass = false;
        Geometry::Point rolePosition = playerPositions[dr_idx];
        //  Determine path for assigned role
        assignAnyToPosition(teamplanner_data, static_cast<int>(dr_idx), dynamic_role, rolePosition, planner_target, role_position_is_end_position_of_pass);
        // stop if current player has ball
        if (teamplanner_data.team[THIS_PLAYER_TEAM_IDX].assigned && teamplanner_data.parameters.calculateAllPaths == false) {
            // path for this robot (player nr: 0) is found and not all paths must be calculated.
            break;
        }
        //cerr << dr_idx <<": DR (fixed): " << DynamicRoleAsString(teamplanner_data.teamFormation[dr_idx]) << " pos: " << rolePosition.toString() << endl;
    }

}

//--------------------------------------------------------------------------
Geometry::Point TeamPlay::updatePositionIfNotAllowed(const Geometry::Point& playerPosition, dynamic_role_e dr_role, const Geometry::Point& original_target_position, const FieldConfig& fieldConfig) {
    //  Updated the role position when it is unreachable into a reachable position, the role position is not updated if the original position was reachable.
    //
    //  background info: using role calculation of another role was considered: use defender position when ball was on own half
    //  and using attack support position if ball was on opponent half. The attack support position will not be close to the original position
    //  (which is typical the ball for the interceptor). Using interceptor role position during restart does also take certain distance to ball.

    auto new_position = original_target_position;
    auto robot_offset = fieldConfig.getRobotRadius();
    auto goal_area_x = 0.5 * fieldConfig.getGoalAreaWidth() + robot_offset;  // max x of goal-area
    auto goal_area_top_y = (0.5 * fieldConfig.getFieldLength()) - fieldConfig.getGoalAreaLength() - robot_offset; // top y of goal-area
    const double PLAYER_TO_BORDER_MARGIN = 0.05; // 5 cm space for turning etc.
    auto max_reach_x = fieldConfig.getMaxReachableFieldX() - PLAYER_TO_BORDER_MARGIN;
    auto max_reach_y = fieldConfig.getMaxReachableFieldY() - PLAYER_TO_BORDER_MARGIN;

    // First check if target is within reachable field. If not then update target position into position in the reachable field.
    if ((fabs(new_position.x) > max_reach_x) or fabs(new_position.y) > max_reach_y) {
        // original_target_position is in unreachable area.
        // check where a line-segment intersects with the line-segment player-target_pos.
        double intercept_x = 0.0;
        double intercept_y = 0.0;

        // check with top line of max reachable field and line-segment: player-target
        bool intersect = lineSegmentIntersection(-max_reach_x, max_reach_y, max_reach_x, max_reach_y,
                                                 new_position.x, new_position.y, playerPosition.x, playerPosition.y,
                                                 intercept_x, intercept_y);

        if (not intersect) {
            // check with bottom line of max reachable field and line-segment: player-target
            intersect = lineSegmentIntersection(-max_reach_x, -max_reach_y, max_reach_x, -max_reach_y,
                                                new_position.x, new_position.y, playerPosition.x, playerPosition.y,
                                                intercept_x, intercept_y);
        }
        if (not intersect) {
            // check with left line of max reachable field and line-segment: player-target
            intersect = lineSegmentIntersection(-max_reach_x, -max_reach_y, -max_reach_x, max_reach_y,
                                                new_position.x, new_position.y, playerPosition.x, playerPosition.y,
                                                intercept_x, intercept_y);
        }
        if (not intersect) {
            // check with right line of max reachable field and line-segment: player-target
            intersect = lineSegmentIntersection(max_reach_x, -max_reach_y, max_reach_x, max_reach_y,
                                                new_position.x, new_position.y, playerPosition.x, playerPosition.y,
                                                intercept_x, intercept_y);
        }
        if (not intersect) {
            std::cerr << "target position not in reachable area ("<< new_position.toString()
                      << ")  no intersection found, but it should be... "
                      << " set target position player-position: player is at: " << playerPosition.toString() << std:: endl;
            // set target to player position, alternative could be set target position to center of the field
            new_position.x = playerPosition.x;
            new_position.y = playerPosition.y;
        }
        else {
            new_position.x = intercept_x;
            new_position.y = intercept_y;
        }
    }

    // if new_position (can be updated by previous step) is not within a goal-area. If not then update target position into position not in a goal area.
    if (        (fabs(new_position.x) < goal_area_x)
            and (fabs(new_position.y) > goal_area_top_y)
            and (dr_role != dynamic_role_e::dr_GOALKEEPER))
    {
        // original_target_position is in own or opponent goal area.
        // check where a line-segment intersects with the line-segment player-target_pos.
        double intercept_x = 0.0;
        double intercept_y = 0.0;
        auto max_field_y = 0.5 * fieldConfig.getFieldLength();

        // To ensure that intersection positions are in the field.
        // the abs y target position must be <= max_reach_y
        // a target position far behind the goal will otherwise not have an intersection within the field.
        if (new_position.y < -max_field_y) {
            new_position.y = -max_field_y;
        }
        if (new_position.y >  max_field_y) {
            new_position.y = max_field_y;
        }

        // check with top line of opponent goal area and line-segment: player-target
        bool intersect = lineSegmentIntersection(-goal_area_x, goal_area_top_y, goal_area_x, goal_area_top_y,
                                                 new_position.x, new_position.y, playerPosition.x, playerPosition.y,
                                                 intercept_x, intercept_y);
        if (not intersect) {
            // check with top line of own goal area and line-segment: player-target
            intersect = lineSegmentIntersection(-goal_area_x, -goal_area_top_y, goal_area_x, -goal_area_top_y,
                                                             new_position.x, new_position.y, playerPosition.x, playerPosition.y,
                                                             intercept_x, intercept_y);
        }
        if (not intersect) {
            // check with left line of opponent goal area and line-segment: player-target
            intersect = lineSegmentIntersection(-goal_area_x, goal_area_top_y, -goal_area_x, max_reach_y,
                                                             new_position.x, new_position.y, playerPosition.x, playerPosition.y,
                                                             intercept_x, intercept_y);
        }
        if (not intersect) {
            // check with left line of own goal area and line-segment: player-target
            intersect = lineSegmentIntersection(-goal_area_x, -goal_area_top_y, -goal_area_x, -max_reach_y,
                                                             new_position.x, new_position.y, playerPosition.x, playerPosition.y,
                                                             intercept_x, intercept_y);
        }
        if (not intersect) {
            // check with right line of opponent goal area and line-segment: player-target
            intersect = lineSegmentIntersection(goal_area_x, goal_area_top_y, goal_area_x, max_reach_y,
                                                             new_position.x, new_position.y, playerPosition.x, playerPosition.y,
                                                             intercept_x, intercept_y);
        }
        if (not intersect) {
            // check with right line of own goal area and line-segment: player-target
            intersect = lineSegmentIntersection(goal_area_x, -goal_area_top_y, goal_area_x, -max_reach_y,
                                                             new_position.x, new_position.y, playerPosition.x, playerPosition.y,
                                                             intercept_x, intercept_y);
        }
        if (not intersect) {
            std::cerr << "target position not in reachable area ("<< new_position.toString()
                      << ")  no intersection found, but it should be... "
                      << " set target position player-position: player is at: " << playerPosition.toString() << std:: endl;
            // set target to player position, alternative could be set target position to center of the field
            new_position.x = playerPosition.x;
            new_position.y = playerPosition.y;
        }
        else {
            new_position.x = intercept_x;
            new_position.y = intercept_y;
        }
    }

    return new_position;
}
