/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */
#include "TeamPlay.hpp"

#include "TeamPlannerExport.hpp"
#include "RolePosition.hpp"
#include "GlobalPathDynamicPlanner.hpp"
#include "SvgUtils.hpp"
#include "MathUtils.hpp"

#include <iomanip>
#include <iostream>
#include <limits>
#include <cmath>
#include <climits>
#include <algorithm>

using namespace std;
using namespace MRA;

const bool SETPLAY_2024 = true;



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
    teamplannerData.original_gamestate = teamplannerData.gamestate;

    if (teamplannerData.gamestate == game_state_e::NORMAL) {
        if (teamplannerData.teamControlsBall()) {
            teamplannerData.gamestate = game_state_e::NORMAL_ATTACK;
        }
        else{
            teamplannerData.gamestate = game_state_e::NORMAL_DEFEND;
        }
    }
    teamplannerData.this_player_robotId = teamplannerData.team[0].robotId;

    // ---------------------------------------------------------------------------------------------
    // sort team of teamplanner data by robotId. Remember order to have output in the original order
    // Can be removed if data is provided by caller: in fixed order and indicator which is this robot
    // Then also putting them back in the correct order at the end of this function can be removed.
    std::vector<long> orginal_order_robotIds = std::vector<long>();
    for (auto idx = 0u; idx < teamplannerData.team.size(); idx++) {
        orginal_order_robotIds.push_back(teamplannerData.team[idx].robotId);
    }
    // first sort TeamPlannerTeam on robotId
    sort(teamplannerData.team.begin(),teamplannerData.team.end(), TeamPlannerRobot::CompareRobotId);
    for (auto idx = 0u; idx< teamplannerData.team.size(); idx++) {
        if (teamplannerData.team[idx].robotId == teamplannerData.this_player_robotId) {
            teamplannerData.this_player_idx = idx;
        }
    }
    // <<< END sort team of teamplanner data by robotId

    teamplannerData.teamFormation = getListWithRoles(teamplannerData);

    // printAssignInputs(teamplannerData);  // for debug purposes
    teamplannerData.original_opponents = teamplannerData.opponents;
    teamplannerData.opponents = {};
    for (unsigned idx = 0; idx < teamplannerData.original_opponents.size(); idx++) {
        MRA::Geometry::Point opponent_pos = teamplannerData.original_opponents[idx].position;
        bool behind_own_backline = (fabs(opponent_pos.x) < teamplannerData.fieldConfig.getMaxFieldX()-1)  // 1 meter from side
                        and opponent_pos.y < -teamplannerData.fieldConfig.getMaxFieldY(); // behind backline
        if (teamplannerData.fieldConfig.isInReachableField(opponent_pos) and not behind_own_backline) {
            teamplannerData.opponents.push_back(teamplannerData.original_opponents[idx]);
        }
    }

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
            PlayerPlannerResult player_result(teamplannerData.gamestate);
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
        if (teamplannerData.teamControlsBall()) {
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
//    MRA::Geometry::Point ballPos = teamplannerData.ball;

    if (teamplannerData.gamestate == game_state_e::BEGIN_POSITION) {
        assignBeginPositions (teamplannerData);
    }
    else if (teamplannerData.gamestate == game_state_e::PARKING) {
        assignParkingPositions (teamplannerData);
    }
    else if (isOneOf(teamplannerData.gamestate, {FREEKICK, GOALKICK, CORNER, THROWIN}) and not teamplannerData.searchForBall) {
        int assignment_nr = 0;
        for (unsigned dr_idx = 0; dr_idx < teamplannerData.teamFormation.size() and dr_idx < teamplannerData.team.size(); dr_idx++) {

            planner_target_e planner_target = determine_planner_target(teamplannerData.teamFormation[dr_idx], teamplannerData.gamestate);
            bool role_position_is_end_position_of_pass = false;

            MRA::Geometry::Point rolePosition;
            rolePosition = RolePosition::determineSetplayRolePosition_2024(++assignment_nr,
                                                                           teamplannerData.defend_info,
                                                                           planner_target,
                                                                           ++m_gridFileNumber,
                                                                           teamplannerData.teamFormation[dr_idx],
                    teamplannerData, playerPassedBall, role_position_is_end_position_of_pass);

            //  Determine path for assigned role
            assignAnyToPosition(teamplannerData, teamplannerData.teamFormation[dr_idx], rolePosition, planner_target, role_position_is_end_position_of_pass);

            // stop if this player is assigne and not all paths must be calculated.
            if (teamplannerData.team[teamplannerData.this_player_idx].assigned && teamplannerData.parameters.calculateAllPaths == false) {
                // path for this robot (player nr: teamplannerData.this_player_idx) is found and not all paths must be calculated.
                break;
            }
        }
    }
    else {
        vector<MRA::Geometry::Point> fixedPlayerPositions = {};
        RolePosition::GetFixedPositions(fixedPlayerPositions, teamplannerData);
        for (unsigned dr_idx = 0; dr_idx < teamplannerData.teamFormation.size(); dr_idx++) {
            if (dr_idx >= teamplannerData.team.size()) {
                // can not assign more roles than team members.
                break;
            }

            planner_target_e planner_target = determine_planner_target(teamplannerData.teamFormation[dr_idx], teamplannerData.gamestate);
            bool role_position_is_end_position_of_pass = false;

            MRA::Geometry::Point rolePosition;
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
            assignAnyToPosition(teamplannerData, teamplannerData.teamFormation[dr_idx], rolePosition, planner_target, role_position_is_end_position_of_pass);

            //cerr << dr_idx <<": DR: " << DynamicRoleAsString(teamplannerData.teamFormation[dr_idx]) << " pos: " << rolePosition.toString() << endl;

            // stop if current player has ball
            if (teamplannerData.team[teamplannerData.this_player_idx].assigned && teamplannerData.parameters.calculateAllPaths == false) {
                // path for this robot (player nr: teamplannerData.this_player_idx) is found and not all paths must be calculated.
                break;
            }
        }
    }

    // ASSIGN ALWAYS A ROLE TO THE AVAIALABLE PLAYERS
    if (teamplannerData.team[teamplannerData.this_player_idx].assigned == false || teamplannerData.parameters.calculateAllPaths)
    {
        // current player does not have any role or all players must be assigned.
        // assign defender role to all unassigned players
        for (unsigned ap_idx = 0; ap_idx < teamplannerData.team.size(); ap_idx++) {
            if (teamplannerData.team[ap_idx].assigned == false)  {
                //    Determine path for assigned role
                planner_target_e planner_target = determine_planner_target(dynamic_role_e::dr_DEFENDER, teamplannerData.gamestate);
                bool role_position_is_end_position_of_pass = false;
                MRA::Geometry::Point rolePosition = RolePosition::determineDynamicRolePosition(teamplannerData.defend_info, planner_target, m_gridFileNumber, dynamic_role_e::dr_DEFENDER,
                        teamplannerData, playerPassedBall, role_position_is_end_position_of_pass);

                assignAnyToPosition(teamplannerData, dynamic_role_e::dr_DEFENDER, rolePosition, planner_target, role_position_is_end_position_of_pass);
            }
        }
    }

    // calculate path for this robot
    calculatePathForRobot(teamplannerData, teamplannerData.this_player_idx);
    if (teamplannerData.parameters.calculateAllPaths) {
        // Calculate Robot Planner path for other robots
        for (unsigned idx = 0; idx < teamplannerData.team.size(); idx++) {
            // all robots except this-robot (already calculated)
            if (teamplannerData.this_player_idx != idx) {
                if (teamplannerData.team[idx].assigned )  {
                    // calculate path for robot.
                    calculatePathForRobot(teamplannerData, idx);
                }
            }
        }
    }

    if (teamplannerData.gamestate == NORMAL_DEFEND and
        (teamplannerData.ball_status == ball_status_e::FREE or teamplannerData.ball_status == ball_status_e::OWNED_BY_TEAMMATE)) {
        // replan interceptor with local ball only if interceptor is not performing a priority block
        int interceptorIdx = -1;
        for (unsigned idx = 0; idx < teamplannerData.team.size(); idx++) {
            if (teamplannerData.team[idx].assigned)  {
                if (teamplannerData.team[idx].result.dynamic_role == dr_INTERCEPTOR) {
                    interceptorIdx = idx;
                }

            }
        }

        bool replan = false;
        if ((interceptorIdx == (int) teamplannerData.this_player_idx) || teamplannerData.parameters.calculateAllPaths) {
            replan = (interceptorIdx != -1) && (teamplannerData.team[interceptorIdx].result.path.size() > 0) && (teamplannerData.team[interceptorIdx].result.path[teamplannerData.this_player_idx].target != PRIORITY_BLOCK);
        }
        if (replan && interceptorIdx >= 0) {
            ReplanInterceptor(interceptorIdx, teamplannerData);
        }
    }

    // loop over the player path to verify if path of this player stay within the allowed boundaries (field+safety area)
    bool thisPlayerHasUnallowedPath = false;
    bool thisPlayerStartsAtUnallowedPosition = false;
    for (unsigned idx = 0; idx < teamplannerData.team.size(); idx++) {
        PlayerPlannerResult player_result;
        if (teamplannerData.team[idx].assigned) {
            bool pathOK = stayPathWithinBoundaries(teamplannerData.fieldConfig, teamplannerData.team[idx].result);
            player_result = teamplannerData.team[idx].result;
            player_result.gamestate = teamplannerData.team[idx].result.gamestate;
            player_result.dynamic_role = teamplannerData.team[idx].result.dynamic_role;
            if (not pathOK && idx == teamplannerData.this_player_idx) { // CHECK Only this robot
                thisPlayerHasUnallowedPath = true; // this robot has wrong path

                // check if player starts at position in the field
                auto currentPos = teamplannerData.team[idx].position;
                thisPlayerStartsAtUnallowedPosition = !teamplannerData.fieldConfig.isInReachableField(currentPos.x, currentPos.y);
            }
        }
        player_paths.push_back(player_result);
    }

    bool dynamicRoleNoneAssigned =  teamplannerData.team[teamplannerData.this_player_idx].result.dynamic_role == dr_NONE;
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

        // create empty path for robot with a path that ends outside the field.
        std::vector<planner_piece_t> path = player_paths[teamplannerData.this_player_idx].path;
        if (not path.empty()) {
            auto end_position = path[path.size()-1];
            if (not teamplannerData.fieldConfig.isInReachableField(end_position.x,end_position.y)) {
                // end position is unreachable: create empty path
                player_paths[teamplannerData.this_player_idx] = PlayerPlannerResult();
            }
        }
    }

    //printAssignOutputs(Team, player_paths_in_correct_order);

    // ----------------------------------------------------------
    // Put the player_paths in the same order as Team was defined by the client
    // Can be remove if client provide team in correct ordere
    std::vector<TeamPlannerRobot> restored_order_team = {};
    for (auto org_idx = 0u; org_idx < orginal_order_robotIds.size(); org_idx++) {
        for (unsigned team_idx = 0; team_idx < teamplannerData.team.size(); team_idx++) {
            if (teamplannerData.team[team_idx].robotId == orginal_order_robotIds[org_idx]) {
                restored_order_team.push_back(teamplannerData.team[team_idx]);
            }
        }
    }
    teamplannerData.team = restored_order_team;

    std::vector<PlayerPlannerResult> player_paths_in_correct_order;
    for (unsigned team_idx = 0; team_idx < teamplannerData.team.size(); team_idx++) {
        player_paths_in_correct_order.push_back(teamplannerData.team[team_idx].result);
    }

    return player_paths_in_correct_order;
}

//---------------------------------------------------------------------------------------------------------------------
void
TeamPlay::calculatePathForRobot (TeamPlannerData &r_teamplannerData, unsigned idx) {

    MRA::Geometry::Point BallTargetPos = MRA::Geometry::Point (r_teamplannerData.pass_data.target_pos.x, r_teamplannerData.pass_data.target_pos.y);

    bool avoidBallPath = (r_teamplannerData.gamestate == game_state_e::NORMAL_ATTACK)
                    and r_teamplannerData.pass_data.valid; // avoid ball path only if pass made (or shot on goal) during normal play (normal_attack)
    if (r_teamplannerData.team[idx].result.target_position_is_end_position_of_pass) {
        // in case of pass, don't avoid ball path if player is destination of the pass
        avoidBallPath = false;
    }

    vector<TeamPlannerRobot> myTeam = getTeamMates (r_teamplannerData.team, idx, true);
    GlobalPathPlanner visibilityGraph = GlobalPathPlanner (r_teamplannerData.fieldConfig); // create robot planner
    visibilityGraph.setOptions (r_teamplannerData.parameters);
    // create list of possible targets for robot-planner
    std::vector<MRA::Vertex> targetPos = vector<MRA::Vertex> ();

    // check if role position is in allowed, if not allowed update position to an allowed positions
    r_teamplannerData.team[idx].result.target = updatePositionIfNotAllowed (
                    r_teamplannerData.team[idx].position, r_teamplannerData.team[idx].result.dynamic_role,
                    r_teamplannerData.team[idx].result.target, r_teamplannerData.fieldConfig);

    bool stay_in_playing_field = stayInPlayingField (r_teamplannerData.gamestate);

    targetPos.push_back (Vertex (r_teamplannerData.team[idx].result.target, 0));
    visibilityGraph.createGraph (r_teamplannerData.team[idx].position, r_teamplannerData.team[idx].velocity, r_teamplannerData.ball,
                                 myTeam, r_teamplannerData.opponents,
                                 targetPos,r_teamplannerData.team[idx].result.planner_target,
                                 r_teamplannerData.ballIsObstacle,
                                 avoidBallPath, stay_in_playing_field, BallTargetPos);

    r_teamplannerData.team[idx].result.path = visibilityGraph.getShortestPath(r_teamplannerData);
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
        cerr << "[" << player_idx  << "] Player-id: " << Team[player_idx].robotId << DynamicRoleAsString(Team[player_idx].result.dynamic_role) << endl;
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
        target = planner_target_e::GOALKEEPER;
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
    case dr_IS_ALIVE:
    case dr_LOB_CALIBRATION:
    case dr_NONE:
        // empty
        break;
    }
    return target;
}
//---------------------------------------------------------------------------------------------------------------------
vector<MRA::Geometry::Position> TeamPlay::getOpponents(const std::vector<TeamPlannerOpponent>&  Opponents) {
    vector<MRA::Geometry::Position> opponents = vector<MRA::Geometry::Position>();
    for (unsigned int idx = 0; idx < Opponents.size(); idx++) {
        opponents.push_back(Opponents[idx].position);
    }
    return opponents;
}

bool TeamPlay::AssignAnyRobotPreferedSetPlayer(TeamPlannerData&  teamplanner_data, dynamic_role_e dr_role,
                                               planner_target_e planner_target, const MRA::Geometry::Point& targetPos) {
    // Teamplanner will first select the receiver during set-play.
    // Planner will look to all available (unassigned) field-players.
    // If preferred receiver is available, then it will selected. Else it will select the lowest player-id which is not the kicker.
    // If not available (only kicker on the field), then the kicker will be selected.
    //    Same algorithm will be applied for the kicker (but receiver is then already be assigned).

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
        teamplanner_data.team[foundPlayer].result = PlayerPlannerResult(
            teamplanner_data.gamestate,
            dr_role,
            teamplanner_data.incrementAndGetRank(),
            targetPos,
            planner_target);
        teamplanner_data.team[foundPlayer].assigned = true;

        //                    cerr << "PREFERRED ROBOT - id: " << preferredRobotId << " targetpos: " << target.toString() << endl;
        return true;
    }
    return false;
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * Find the player which is the fastest to a single target-position (using path finder),
 * assign the fast player to the position and update the unassigned player list.
 * */
bool TeamPlay::assignAnyToPosition(TeamPlannerData&  teamplanner_data, dynamic_role_e dr_role,
            const MRA::Geometry::Point& target, planner_target_e planner_target, bool role_position_is_end_position_of_pass)
{
    bool found = false; // return value
    vector<MRA::Geometry::Point> targets = vector<MRA::Geometry::Point>();
    targets.push_back(target);

    // ------------------------------------------------------------

    // assign Preferred set player roles if applicable
    if (AssignAnyRobotPreferedSetPlayer(teamplanner_data, dr_role, planner_target, target)) {
        return true; // preferred set player found and assigned
    }

    if (role_position_is_end_position_of_pass && teamplanner_data.pass_data.valid) {
        /* only if role_position is end position of pass and pass data is valid*
         * find player with same id as target_id in pass data (that player is destination of pass) */
        // loop over all players
        for (unsigned int idx = 0; idx < teamplanner_data.team.size(); idx++) {
            if (teamplanner_data.team[idx].assigned) {
                continue;  // player already assigned, skip
            }
            if (teamplanner_data.team[idx].robotId == teamplanner_data.pass_data.target_id) {
                /* this player is destination of the pass */
                teamplanner_data.team[idx].result = PlayerPlannerResult(
                    teamplanner_data.gamestate,
                    dr_role,
                    teamplanner_data.incrementAndGetRank(),
                    target,
                    planner_target,
                    teamplanner_data.defend_info,
                    role_position_is_end_position_of_pass);
                teamplanner_data.team[idx].assigned = true;
                return true;
            }
        }
    }

    vector <AssignToTargetData> assignToTargetData = vector<AssignToTargetData>();
    MRA::Geometry::Point currentEndPos = target;  // intended target, can be slightly different than calculated (if path is blocked);

    const int BEST_PLAYER_UNASSIGNED = -1;
    int bestPlayerIdx = BEST_PLAYER_UNASSIGNED;
    // do all criteria calculations for the team and store it in AssignToTargetData structs for all robots (incl. assigned)
    for (unsigned int idx = 0; idx < teamplanner_data.team.size(); idx++) {
        AssignToTargetData player = {};
        player.available = not teamplanner_data.team[idx].assigned;

        if (dr_role == dr_BALLPLAYER && not teamplanner_data.team[idx].controlBall) {
            player.available = false; // dribble can only be done, by player with ball, so skip this robot
        }
        if (dr_role == dr_INTERCEPTOR && teamplanner_data.team[idx].passBall) {
            player.available = false; // ball should not be intercepted by player who passed
        }

        if (player.available) {
            if (bestPlayerIdx == BEST_PLAYER_UNASSIGNED) {
                bestPlayerIdx = idx;
            }
            // Calculate full stop position, in the direction of current velocity vector
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
            player.distToTarget = currentEndPos.distanceTo(fullStopPos);

            player.distToPreviousTarget = 0.0;
            // calculate previous target position distance-threshold.
            if (teamplanner_data.team[idx].previous_result.previous_result_present == 1 && teamplanner_data.team[idx].previous_result.dynamic_role == dr_role)
            {
                Geometry::Point previousEndPos = Geometry::Point(teamplanner_data.team[idx].previous_result.end_position.x,
                                                                 teamplanner_data.team[idx].previous_result.end_position.y);
                if (currentEndPos.distanceTo(previousEndPos) < teamplanner_data.parameters.previous_role_end_pos_threshold) {
                    player.distToPreviousTarget = teamplanner_data.parameters.previous_role_bonus_end_pos_radius;
                }
            }
            player.totalCost = player.distToTarget - player.distToPreviousTarget;
        }
        assignToTargetData.push_back(player);

    }

    if (bestPlayerIdx != BEST_PLAYER_UNASSIGNED) {

        for (unsigned int idx = 0; idx < teamplanner_data.team.size(); idx++) {
            if (not assignToTargetData[idx].available) {
                continue;
            }

            if (idx == (unsigned) bestPlayerIdx) {
                continue;
            }
            // compare this robot with the current best robot.
            // best robot will become the new best robot

            if (fabs(assignToTargetData[idx].totalCost - assignToTargetData[bestPlayerIdx].totalCost) < teamplanner_data.parameters.equality_cost_threshold) {
                // equality: lowest robot id wins
                if (teamplanner_data.team[idx].robotId < teamplanner_data.team[bestPlayerIdx].robotId) {
                    // current player has lower id.
                    bestPlayerIdx = idx;
                }
            }
            else if (fabs(assignToTargetData[idx].totalCost < assignToTargetData[bestPlayerIdx].totalCost)) {
                bestPlayerIdx = idx;  // current player will be come the new best player.
            }
        }

    }

    found = bestPlayerIdx != BEST_PLAYER_UNASSIGNED;
    if (found) {
        // fill best robot data in planner result.
        teamplanner_data.team[bestPlayerIdx].result = PlayerPlannerResult(
            teamplanner_data.gamestate,
            dr_role,
            teamplanner_data.incrementAndGetRank(),
            target,
            planner_target,
            teamplanner_data.defend_info,
            role_position_is_end_position_of_pass);
        teamplanner_data.team[bestPlayerIdx].assigned = true;
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
            MRA::Geometry::Point ballPos = teamplanner_data.ball.position;
            double penalty_area_half_width = teamplanner_data.fieldConfig.getPenaltyAreaWidth() * 0.5;
            if (ballPos.x < -penalty_area_half_width) {
                //            if (global) ball is to the left of the field (left from outer line penalty area) position keeper to the left.
                goaliePosition = MRA::Geometry::Point(-teamplanner_data.fieldConfig.getGoalWidth()*0.25, goalieYPosition); // left half of the goal;
            }
            else if (ballPos.x > penalty_area_half_width) {
                goaliePosition = MRA::Geometry::Point(+teamplanner_data.fieldConfig.getGoalWidth()*0.25, goalieYPosition); // right half of the goal;
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
        int setPlayReceiverIdx = -1;
        int preferredKickerRobotId = teamplanner_data.parameters.preferredSetplayKicker;
        int preferredReceiverRobotId = teamplanner_data.parameters.preferredSetplayReceiver;
        vector<int> availablePlayers = vector<int>();

        if (preferredReceiverRobotId == preferredKickerRobotId)
        {
            preferredReceiverRobotId = 0; // avoid select conflict. prefer kicker above receiver.
        }

        for (unsigned int idx = 0; idx < teamplanner_data.team.size(); idx++) {
                // player can be assigned
                if (teamplanner_data.team[idx].robotId == preferredReceiverRobotId) {
                    setPlayReceiverIdx = idx;
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
            if (setPlayReceiverIdx != -1) {
                availablePlayers.push_back(setPlayReceiverIdx);
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
        defend_info_t defend_info = {};
        teamplanner_data.team[keeper_idx].result = PlayerPlannerResult(
            teamplanner_data.gamestate,
            teamplanner_data.gamestate == game_state_e::PARKING ? dr_PARKING : dr_GOALKEEPER,
            teamplanner_data.incrementAndGetRank(),
            goaliePosition,
            teamplanner_data.gamestate == game_state_e::PARKING ? planner_target_e::GOTO_TARGET_POSITION_SLOW : planner_target_e::GOALKEEPER,
            defend_info); // no defend info for goalie
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
                    MRA::Geometry::Point pos = teamplanner_data.team[idx].position;
                    if (teamplanner_data.fieldConfig.isInOwnPenaltyArea(pos.x, pos.y)) {
                        double d_y = -(teamplanner_data.fieldConfig.getFieldLength()/2) + teamplanner_data.fieldConfig.getPenaltyAreaLength();
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
                        teamplanner_data.team[idx].result = PlayerPlannerResult(
                            teamplanner_data.gamestate,
                            dr_DEFENDER,
                            teamplanner_data.incrementAndGetRank(),
                            targetPos,
                            planner_target_e::SUPPORT_DEFENSE,
                            teamplanner_data.defend_info);
                    }
                }
                if (teamplanner_data.team[idx].time_in_opponent_penalty_area > TIME_TOO_LONG_IN_PENALTY_AREA_THRESHOLD) { // TODO
                    // player not assigned and too long in penalty area
                    MRA::Geometry::Point pos = teamplanner_data.team[idx].position;
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
                        teamplanner_data.team[idx].result = PlayerPlannerResult(
                                                    teamplanner_data.gamestate,
                                                    dr_DEFENDER,
                                                    teamplanner_data.incrementAndGetRank(),
                                                    targetPos,
                                                    planner_target_e::SUPPORT_DEFENSE,
                                                    teamplanner_data.defend_info);
                    }
                }
            }
        }
    }
}

//----------------------------------------------------------------------------------------
vector<TeamPlannerRobot> TeamPlay::getTeamMates(const std::vector<TeamPlannerRobot>& Team, unsigned meIdx, bool addAssignedTargetAsTeamPosition) {
    vector<TeamPlannerRobot> teammates = {};
    for (unsigned int idx = 0; idx < Team.size(); idx++) {
        if (idx != meIdx) {
            teammates.push_back(Team[idx]);
            if (addAssignedTargetAsTeamPosition && Team[idx].assigned) {
                //add target position as barrier
                if (Team[idx].result.path.size() > 0) {
                    planner_piece_t last_piece = Team[idx].result.path[Team[idx].result.path.size()-1];
                    TeamPlannerRobot barrierRobot = {};
                    barrierRobot.position = Geometry::Position(last_piece.x, last_piece.y);
                    teammates.push_back(barrierRobot);
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
        if (not teamplanner_data.fieldConfig.isInReachableField(teamplanner_data.ball.position.x, teamplanner_data.ball.position.y)    ) {
            // ball is outside field (not in field and not in safety area)
            searchForBall = true; // state requires search for ball
        }
    }
    return searchForBall;
}

//---------------------------------------------------------------------------------------------------------------------
// print the inputs of TeamPlanner::Assign for debug purposes
void TeamPlay::printAssignInputs(const TeamPlannerData& teamplanner_data)
{
    cerr << "Team_Planner::assign  inputs:" << endl << flush;
    cerr << "game_state_e: " << teamplanner_data.gamestate << " (" << GameStateAsString(teamplanner_data.gamestate) << " )"<< endl << flush;
    cerr << "global ball: " << teamplanner_data.ball.toString(true) << endl << flush;
    for (unsigned idx = 0; idx < teamplanner_data.team.size(); idx++) {
        cerr << "Robot [" << idx << "] =" << endl << teamplanner_data.team[idx].toString() << endl;
    }
    cerr << "Opponents size: " << teamplanner_data.opponents.size()  << endl << flush;
    for (unsigned int i = 0; i < teamplanner_data.opponents.size(); i++) {
        cerr << "Opponents[" << i << "].position: "<< teamplanner_data.opponents[i].position.toString()  << endl << flush;
    }
    cerr << "parameters: " << teamplanner_data.parameters.toString() << endl << flush;
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
        //        pass_data.target_id
        //        pass_data.kicked
        //        pass_data.origin_pos
        //        pass_data.target_pos
        //        pass_data.velocity
        //        pass_data.ts
        //        pass_data.eta
        //        pass_data.angle

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
    double distRobotToBall = teamplanner_data.team[teamplanner_data.this_player_idx].position.distanceTo(teamplanner_data.ball.position);
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

    vector<TeamPlannerRobot> myTeam = getTeamMates(teamplanner_data.team, interceptorIdx, true);
    bool stay_in_playing_field = stayInPlayingField (teamplanner_data.gamestate);

    if (nrDynamicPlannerIterations <= 0) {
        bool avoidBallPath = false; // intercept should never avoid a passing ball.
        MRA::Geometry::Point BallTargetPos;
        // use normal planner
        // when ball is obstacle or no iterations for dynamic planner is defined.
        GlobalPathPlanner visibilityGraph = GlobalPathPlanner(teamplanner_data.fieldConfig);
        visibilityGraph.setOptions(teamplanner_data.parameters);
        vector<TeamPlannerRobot> myTeam = getTeamMates(teamplanner_data.team, interceptorIdx, false);
        bool stay_in_playing_field = stayInPlayingField (teamplanner_data.gamestate);
        visibilityGraph.createGraph(teamplanner_data.team[interceptorIdx].position,
                                    teamplanner_data.team[interceptorIdx].velocity, teamplanner_data.ball, myTeam, teamplanner_data.opponents,
                targetPos, planner_target_e::GOTO_BALL, teamplanner_data.ballIsObstacle, avoidBallPath, stay_in_playing_field, BallTargetPos);
        path = visibilityGraph.getShortestPath(teamplanner_data);
    } else {
        // use dynamic robot planner for goto ball
        GlobalPathDynamicPlanner dp = GlobalPathDynamicPlanner();
        path = dp.planPath(    teamplanner_data.team[interceptorIdx].position, teamplanner_data.team[interceptorIdx].velocity,
                               myTeam, teamplanner_data, targetPos,
                planner_target_e::GOTO_BALL, teamplanner_data.ballIsObstacle, maxSpeed,
                nrDynamicPlannerIterations, stay_in_playing_field);
    }

    if (not path.empty()) {
        MRA::Geometry::Point original_role_position(path[path.size()-1].x, path[path.size()-1].y);
        // check if role position is in allowed, if not allowed update position to an allowed positions
        MRA::Geometry::Point role_position = updatePositionIfNotAllowed(teamplanner_data.team[interceptorIdx].position,
                                                            teamplanner_data.team[interceptorIdx].result.dynamic_role,
                                                            original_role_position, teamplanner_data.fieldConfig);

        vector<MRA::Vertex> roleTargetPos = vector<MRA::Vertex>();
        roleTargetPos.push_back(Vertex(role_position, 0));

        bool avoidBallPath = false; // intercept should never avoid a passing ball.
        MRA::Geometry::Point BallTargetPos;
        GlobalPathPlanner visibilityGraph = GlobalPathPlanner(teamplanner_data.fieldConfig);
        visibilityGraph.setOptions(teamplanner_data.parameters);
        visibilityGraph.createGraph(teamplanner_data.team[interceptorIdx].position, teamplanner_data.team[interceptorIdx].velocity,
                                    teamplanner_data.ball, myTeam, teamplanner_data.opponents,
                roleTargetPos, planner_target_e::GOTO_BALL, teamplanner_data.ballIsObstacle, avoidBallPath, stay_in_playing_field, BallTargetPos);
        path = visibilityGraph.getShortestPath(teamplanner_data);
    }
    teamplanner_data.team[interceptorIdx].result.path = path;
}

////---------------------------------------------------------------------------------------------------------------------
// calculate shortest distance of the vector of objects (own or opponents) to the ball
double TeamPlay::calculateShortestDistanceObjectsToTarget(const std::vector<MRA::Geometry::Position>& objects, const MRA::Geometry::Position& targetObject) {
    double shortestDistance = std::numeric_limits<double>::infinity();
    for(unsigned int idx = 0; idx < objects.size(); idx++) {
        double distToBall = objects[idx].distanceTo(targetObject);
        if (distToBall < shortestDistance) {
            shortestDistance = distToBall;
        }
    }
    return shortestDistance;
}

void TeamPlay::assignParkingPositions(TeamPlannerData& teamplanner_data) {
    // assignment parking positions to field players

    // get list with player positions
    vector<MRA::Geometry::Point> fixedPlayerPositions = {};
    RolePosition::GetFixedPositions(fixedPlayerPositions, teamplanner_data); // only positions for field-players

    auto fixed_role_idx = 0u;
    for (auto idx = 0u; idx < teamplanner_data.team.size(); idx++) {
        if (teamplanner_data.team[idx].player_type != player_type_e::GOALIE) {
            // fill best robot data in planner result.
            teamplanner_data.team[idx].result = PlayerPlannerResult(
                teamplanner_data.gamestate,
                dynamic_role_e::dr_PARKING,
                teamplanner_data.incrementAndGetRank(),
                fixedPlayerPositions[fixed_role_idx],
                planner_target_e::GOTO_TARGET_POSITION_SLOW,
                teamplanner_data.defend_info);
            teamplanner_data.team[idx].assigned = true;
            fixed_role_idx++;
        }
    }
}

void TeamPlay::assignBeginPositions(TeamPlannerData& teamplanner_data) {
    // Fixed position assignment for field players
    // get list with player positions
    vector<MRA::Geometry::Point> fixedPlayerPositions = {};
    RolePosition::GetFixedPositions(fixedPlayerPositions, teamplanner_data); // only positions for field-players
    bool setPlayKickerPresent = false;
    bool setPlayReceiverPresent = false;
    for (auto idx = 0u; idx < teamplanner_data.team.size (); idx++) {
        if (teamplanner_data.team[idx].robotId == teamplanner_data.parameters.preferredSetplayKicker) {
            setPlayKickerPresent = true;

        }
        if (teamplanner_data.team[idx].robotId == teamplanner_data.parameters.preferredSetplayReceiver) {
            setPlayReceiverPresent = true;
        }
    }

    unsigned firstFreeBeginPositionIdx = 0;
    unsigned setPlayReceiverIdx = 0;
    if (setPlayKickerPresent and setPlayReceiverPresent) {
        setPlayReceiverIdx = 1;
        firstFreeBeginPositionIdx = 2;
    }
    else if (setPlayReceiverPresent) {
        setPlayReceiverIdx = 0;
        firstFreeBeginPositionIdx = 1;
    }


    auto fixed_role_idx = 0u;
    for (auto idx = 0u; idx < teamplanner_data.team.size(); idx++) {
        if (teamplanner_data.team[idx].player_type != player_type_e::GOALIE) {
            MRA::Geometry::Point targetPosition = {};

            if (teamplanner_data.team[idx].robotId == teamplanner_data.parameters.preferredSetplayKicker) {
                targetPosition = fixedPlayerPositions[0];  // setplay kicker always on position: 0
            }
            else if (teamplanner_data.team[idx].robotId == teamplanner_data.parameters.preferredSetplayReceiver) {
                // setplay receiver on position: 1 if setplay kicker is present, otherwise on position-0
                targetPosition = fixedPlayerPositions[setPlayReceiverIdx];
            }
            else {
                targetPosition = fixedPlayerPositions[firstFreeBeginPositionIdx++];
            }

            // fill best robot data in planner result.
            teamplanner_data.team[idx].result = PlayerPlannerResult(
                teamplanner_data.gamestate,
                dynamic_role_e::dr_BEGIN_POSITION,
                teamplanner_data.incrementAndGetRank(),
                fixedPlayerPositions[fixed_role_idx],
                planner_target_e::GOTO_TARGET_POSITION,
                teamplanner_data.defend_info);
            teamplanner_data.team[idx].assigned = true;
            fixed_role_idx++;
        }
    }
}

//--------------------------------------------------------------------------
MRA::Geometry::Point TeamPlay::updatePositionIfNotAllowed(const MRA::Geometry::Point& playerPosition, dynamic_role_e dr_role, const MRA::Geometry::Point& original_target_position, const FieldConfig& fieldConfig) {
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

bool TeamPlay::stayInPlayingField(game_state_e gamestate) const {
    return isOneOf(gamestate , {
        NORMAL,    NORMAL_ATTACK,
        NORMAL_DEFEND ,
        BEGIN_POSITION,
        KICKOFF,  KICKOFF_AGAINST,
        GOALKICK, GOALKICK_AGAINST,
        THROWIN_AGAINST,
        CORNER_AGAINST,
        PENALTY,
        PENALTY_AGAINST,
        PENALTY_SHOOTOUT,
        PENALTY_SHOOTOUT_AGAINST,
        DROPPED_BALL,
    });
}

std::vector<dynamic_role_e> TeamPlay::getListWithRoles(TeamPlannerData& teamplannerData) {


    std::vector<dynamic_role_e> roles_to_assign = {};
    for (auto idx = 0u; idx < teamplannerData.input_formation.size(); idx++) {
        MRA::RobotsportsRobotStrategy::Output_DynamicRole odr = teamplannerData.input_formation[idx];
        dynamic_role_e dr = dr_NONE;

        switch (odr) {
            case MRA::RobotsportsRobotStrategy::Output_DynamicRole_GOALKEEPER:
                dr = dr_GOALKEEPER;
                break;
            case MRA::RobotsportsRobotStrategy::Output_DynamicRole_ATTACKER_MAIN:
                if (isOneOf(teamplannerData.gamestate, { FREEKICK, GOALKICK, CORNER, KICKOFF, THROWIN})) {
                    dr = dr_SETPLAY_KICKER;
                }
                else if (isOneOf(teamplannerData.gamestate, {PENALTY, PENALTY_SHOOTOUT})) {
                    dr = dr_PENALTY_KICKER;
                }
                else if (isOneOf(teamplannerData.ball_status, {OWNED_BY_PLAYER, OWNED_BY_TEAMMATE})) {
                    dr = dr_BALLPLAYER;
                }
                else if (isOneOf(teamplannerData.gamestate, { FREEKICK_AGAINST, GOALKICK_AGAINST,
                                             CORNER_AGAINST, KICKOFF_AGAINST,
                                             THROWIN_AGAINST, PENALTY_AGAINST,
                                             DROPPED_BALL})) {
                    dr = dr_INTERCEPTOR;
                }
                else if (teamplannerData.ball_status == OWNED_BY_TEAM) {
                    dr = dr_ATTACKSUPPORTER;
                }
                else if (isOneOf(teamplannerData.ball_status, {FREE, OWNED_BY_OPPONENT})) {
                    dr = dr_INTERCEPTOR;
                }
                break;
            case MRA::RobotsportsRobotStrategy::Output_DynamicRole_ATTACKER_ASSIST:
                if (isOneOf(teamplannerData.gamestate, { FREEKICK, GOALKICK, CORNER, KICKOFF, THROWIN})) {
                    dr = dr_SETPLAY_RECEIVER;
                }
                else {
                    dr = dr_ATTACKSUPPORTER;
                }
                break;
            case MRA::RobotsportsRobotStrategy::Output_DynamicRole_ATTACKER_GENERIC:
                dr = dr_ATTACKSUPPORTER;
                break;
            case MRA::RobotsportsRobotStrategy::Output_DynamicRole_DEFENDER_MAIN:
                if (isOneOf(teamplannerData.gamestate, { FREEKICK, GOALKICK, CORNER, KICKOFF, THROWIN})) {
                    dr = dr_ATTACKSUPPORTER;
                }
                else {
                    dr = dr_SWEEPER;
                }
                break;
            case MRA::RobotsportsRobotStrategy::Output_DynamicRole_DEFENDER_GENERIC:
                if (isOneOf(teamplannerData.gamestate, { PENALTY_SHOOTOUT, PENALTY_SHOOTOUT_AGAINST})) {
                    dr = dr_PENALTY_DEFENDER;
                }
                else {
                    dr = dr_DEFENDER;
                }
                break;
            case MRA::RobotsportsRobotStrategy::Output_DynamicRole_DISABLED_OUT:
                dr = dr_PARKING;
                break;
            case MRA::RobotsportsRobotStrategy::Output_DynamicRole_DISABLED_IN:
                dr = dr_BEGIN_POSITION;
                break;
            default:
                break;
        }
        roles_to_assign.push_back(dr);
    }

    return roles_to_assign;
}
