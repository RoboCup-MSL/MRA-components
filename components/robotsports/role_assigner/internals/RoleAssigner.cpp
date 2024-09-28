/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */
#include "RoleAssigner.hpp"

#include "RolePosition.hpp"
#include "GlobalPathDynamicPlanner.hpp"
#include "MathUtils.hpp"

#include <iomanip>
#include <iostream>
#include <limits>
#include <cmath>
#include <climits>
#include <algorithm>
#include "RoleAssignerExport.hpp"
#include "RoleAssignerSvg.hpp"
#include "Vertex.hpp"
#include "RoleAssignerData.hpp"
#include "GlobalPathPlanner.hpp"

using namespace std;
using namespace MRA;

//---------------------------------------------------------------------------------------------------------------------
RoleAssigner::RoleAssigner() : m_gridFileNumber(0) {

}

void RoleAssigner::assign(const RoleAssignerInput& input,
            RoleAssignerState& r_state,
            RoleAssignerOutput& r_output,
            const RoleAssignerParameters& parameters) {
    // convert to role_assigner_data;
    RoleAssignerData role_assigner_data = {};
    role_assigner_data.parameters = parameters;
    role_assigner_data.fieldConfig = FieldConfig(parameters.field_parameters);
    role_assigner_data.input_formation = input.input_formation;
    role_assigner_data.gamestate = input.gamestate;
    role_assigner_data.ball = input.ball;
    role_assigner_data.parking_positions = input.parking_positions;
    role_assigner_data.ball_pickup_position = input.ball_pickup_position;
    role_assigner_data.passIsRequired = input.passIsRequired;
    role_assigner_data.pass_data = input.pass_data;
    role_assigner_data.previous_ball = r_state.previous_ball;
    role_assigner_data.team = input.team;
    role_assigner_data.opponents = input.opponents;

    // inputs
    role_assigner_data.previous_ball = r_state.previous_ball;
    for (auto idx = 0u; idx < r_state.previous_result.size(); ++idx) {
        RoleAssignerAdminTeam tp_admin = {};
        tp_admin.previous_result = r_state.previous_result[idx];
        role_assigner_data.team_admin.push_back(tp_admin);
    }

    std::vector<RoleAssignerResult> assign_results = assign(role_assigner_data);
    r_state.previous_ball.present = role_assigner_data.ball.is_valid;
    r_state.previous_ball.x  = role_assigner_data.ball.position.x;
    r_state.previous_ball.y  = role_assigner_data.ball.position.y;
    for (auto idx = 0u; idx < role_assigner_data.team_admin.size(); ++idx) {
        r_output.player_paths.push_back(role_assigner_data.team_admin[idx].result);
    }

}

//---------------------------------------------------------------------------------------------------------------------
/*
 * Main method for dynamic role assignment.
 * Assign roles to team members based on game-situation and location of the players and the ball on the field.
 *
 * Internally the method will based on the game-situation call an other internal method which handles the specific game situation.
 */

std::vector<RoleAssignerResult> RoleAssigner::assign(RoleAssignerData& role_assigner_data)
{
    role_assigner_data.original_gamestate = role_assigner_data.gamestate;

    if (role_assigner_data.gamestate == game_state_e::NORMAL) {
        if (role_assigner_data.teamControlsBall()) {
            role_assigner_data.gamestate = game_state_e::NORMAL_ATTACK;
        }
        else{
            role_assigner_data.gamestate = game_state_e::NORMAL_DEFEND;
        }
    }

    role_assigner_data.this_player_robotId = role_assigner_data.team[0].robotId;

    // ---------------------------------------------------------------------------------------------
    // sort team of role_assigner data by robotId. Remember order to have output in the original order
    // Can be removed if data is provided by caller: in fixed order and indicator which is this robot
    // Then also putting them back in the correct order at the end of this function can be removed.
    std::vector<long> orginal_order_robotIds = std::vector<long>();

    for (auto idx = 0u; idx < role_assigner_data.team.size(); idx++) {
        orginal_order_robotIds.push_back(role_assigner_data.team[idx].robotId);
    }
    // first sort RoleAssignerTeam on robotId
    sort(role_assigner_data.team.begin(),role_assigner_data.team.end(), RoleAssignerRobot::CompareRobotId);
    sort(role_assigner_data.team_admin.begin(),role_assigner_data.team_admin.end(), RoleAssignerAdminTeam::CompareRobotId);
    for (auto idx = 0u; idx< role_assigner_data.team.size(); idx++) {
        if (role_assigner_data.team[idx].robotId == role_assigner_data.this_player_robotId) {
            role_assigner_data.this_player_idx = idx;
        }
    }
    // <<< END sort team of role assigner data by robotId

    role_assigner_data.teamFormation = role_assigner_data.input_formation;// getListWithRoles(role_assigner_data);

    // printAssignInputs(role_assigner_data);  // for debug purposes
    role_assigner_data.original_opponents = role_assigner_data.opponents;
    role_assigner_data.opponents = {};
    for (unsigned idx = 0; idx < role_assigner_data.original_opponents.size(); idx++) {
        MRA::Geometry::Point opponent_pos = role_assigner_data.original_opponents[idx].position;
        bool behind_own_backline = (fabs(opponent_pos.x) < role_assigner_data.fieldConfig.getMaxFieldX()-1)  // 1 meter from side
                        and opponent_pos.y < -role_assigner_data.fieldConfig.getMaxFieldY(); // behind backline
        if (role_assigner_data.fieldConfig.isInReachableField(opponent_pos) and not behind_own_backline) {
            role_assigner_data.opponents.push_back(role_assigner_data.original_opponents[idx]);
        }
    }

    std::vector<RoleAssignerResult> player_paths;
    if (role_assigner_data.gamestate != game_state_e::NONE) {
        // printAssignInputs(gamestate, ball, Team, Opponents, parameters,  parking_positions, ball_pickup_position, passIsRequired, pass_data);
    }
    if ((role_assigner_data.gamestate == game_state_e::NONE) ||
            (role_assigner_data.gamestate == game_state_e::YELLOW_CARD_AGAINST) ||
            (role_assigner_data.gamestate == game_state_e::RED_CARD_AGAINST) ||
            (role_assigner_data.gamestate == game_state_e::GOAL) ||
            (role_assigner_data.gamestate == game_state_e::GOAL_AGAINST)) {
        // unhandled game situations: cards and goals are not passed to the team planner via the game state-machine
        for (unsigned idx = 0; idx < role_assigner_data.team.size(); idx++) {
            RoleAssignerResult player_result = {};
            player_paths.push_back(player_result);
        }
        return player_paths; // no path will be planned if game state is NONE
    }

    bool playerPassedBall = false;
    for (unsigned r_idx = 0; r_idx < role_assigner_data.team.size(); r_idx++) {
        if (role_assigner_data.team[r_idx].passBall) {
            playerPassedBall = true;
        }
    }

    if (role_assigner_data.gamestate == game_state_e::NORMAL) {
        if (role_assigner_data.teamControlsBall()) {
            role_assigner_data.gamestate = game_state_e::NORMAL_ATTACK;
        }
        else{
            role_assigner_data.gamestate = game_state_e::NORMAL_DEFEND;
        }
    }

    for (unsigned r_idx = 0; r_idx < role_assigner_data.team_admin.size(); r_idx++) {
        role_assigner_data.team_admin[r_idx].result.role = role_e::role_UNDEFINED;
        role_assigner_data.team_admin[r_idx].result.defend_info.valid = false;
    }


    /* set avoid ball flag : only not avoiding during normal play */
    role_assigner_data.ballIsObstacle = (role_assigner_data.gamestate != game_state_e::NORMAL_ATTACK) && (role_assigner_data.gamestate != game_state_e::NORMAL_DEFEND);

    // Assign first the Goalie
    assignGoalie(role_assigner_data);

    // Assign players too long in any penalty area: closest position out of the penalty area
    assignTooLongInPenaltyAreaPlayers(role_assigner_data);

    role_assigner_data.searchForBall = searchForBallBehaviorNeeded(role_assigner_data);
//    MRA::Geometry::Point ballPos = role_assigner_data.ball;

    if (role_assigner_data.gamestate == game_state_e::BEGIN_POSITION) {
        assignBeginPositions (role_assigner_data);
    }
    else if (role_assigner_data.gamestate == game_state_e::PARKING) {
        assignParkingPositions (role_assigner_data);
    }
    else if (isOneOf(role_assigner_data.gamestate, {FREEKICK, GOALKICK, CORNER, THROWIN}) and not role_assigner_data.searchForBall) {
        int assignment_nr = 0;
        for (unsigned dr_idx = 0; dr_idx < role_assigner_data.teamFormation.size() and dr_idx < role_assigner_data.team.size(); dr_idx++) {

            planner_target_e planner_target = determine_planner_target(role_assigner_data.teamFormation[dr_idx], role_assigner_data);
            bool role_position_is_end_position_of_pass = false;

            MRA::Geometry::Point rolePosition;
            rolePosition = RolePosition::determineSetplayRolePosition_2024(++assignment_nr,
                                                                           role_assigner_data.defend_info,
                                                                           planner_target,
                                                                           ++m_gridFileNumber,
                                                                           role_assigner_data.teamFormation[dr_idx],
                    role_assigner_data, playerPassedBall, role_position_is_end_position_of_pass);

            //  Determine path for assigned role
            assignAnyToPosition(role_assigner_data, role_assigner_data.teamFormation[dr_idx], rolePosition,
                                planner_target, role_position_is_end_position_of_pass, role_assigner_data.input_formation[dr_idx]);

            // stop if this player is assigne and not all paths must be calculated.
            if (role_assigner_data.team_admin[role_assigner_data.this_player_idx].assigned && role_assigner_data.parameters.calculateAllPaths == false) {
                // path for this robot (player nr: role_assigner_data.this_player_idx) is found and not all paths must be calculated.
                break;
            }
        }
    }
    else {
        vector<MRA::Geometry::Point> fixedPlayerPositions = {};
        RolePosition::GetFixedPositions(fixedPlayerPositions, role_assigner_data);
        for (unsigned dr_idx = 0; dr_idx < role_assigner_data.teamFormation.size(); dr_idx++) {
            if (dr_idx >= role_assigner_data.team.size()) {
                // can not assign more roles than team members.
                break;
            }

            planner_target_e planner_target = determine_planner_target(role_assigner_data.teamFormation[dr_idx], role_assigner_data);
            bool role_position_is_end_position_of_pass = false;

            MRA::Geometry::Point rolePosition;
            if (   role_assigner_data.searchForBall
                   || role_assigner_data.gamestate ==  game_state_e::KICKOFF
                   || role_assigner_data.gamestate ==  game_state_e::KICKOFF_AGAINST) {
                    // get list with player positions
                rolePosition = fixedPlayerPositions[dr_idx];
                planner_target = planner_target_e::GOTO_TARGET_POSITION;
            }
            else {
                rolePosition = RolePosition::determineDynamicRolePosition(role_assigner_data.defend_info, planner_target, m_gridFileNumber, role_assigner_data.teamFormation[dr_idx],
                        role_assigner_data, playerPassedBall, role_position_is_end_position_of_pass);
            }


            //  Determine path for assigned role
            assignAnyToPosition(role_assigner_data, role_assigner_data.teamFormation[dr_idx], rolePosition, planner_target,
                                role_position_is_end_position_of_pass, role_assigner_data.input_formation[dr_idx]);

            //cerr << dr_idx <<": DR: " << DynamicRoleAsString(role_assigner_data.teamFormation[dr_idx]) << " pos: " << rolePosition.toString() << endl;

            // stop if current player has ball
            if (role_assigner_data.team_admin[role_assigner_data.this_player_idx].assigned && role_assigner_data.parameters.calculateAllPaths == false) {
                // path for this robot (player nr: role_assigner_data.this_player_idx) is found and not all paths must be calculated.
                break;
            }
        }
    }

    // ASSIGN ALWAYS A ROLE TO THE AVAIALABLE PLAYERS
    if (role_assigner_data.team_admin[role_assigner_data.this_player_idx].assigned == false || role_assigner_data.parameters.calculateAllPaths)
    {
        // current player does not have any role or all players must be assigned.
        // assign defender role to all unassigned players
        for (unsigned ap_idx = 0; ap_idx < role_assigner_data.team.size(); ap_idx++) {
            if (role_assigner_data.team_admin[ap_idx].assigned == false)  {
                //    Determine path for assigned role
                planner_target_e planner_target = determine_planner_target(role_e::role_DEFENDER_GENERIC, role_assigner_data);
                bool role_position_is_end_position_of_pass = false;
                MRA::Geometry::Point rolePosition = RolePosition::determineDynamicRolePosition(role_assigner_data.defend_info, planner_target, m_gridFileNumber,
                                                    role_DEFENDER_GENERIC, role_assigner_data, playerPassedBall, role_position_is_end_position_of_pass);

                assignAnyToPosition(role_assigner_data, role_DEFENDER_GENERIC, rolePosition, planner_target, role_position_is_end_position_of_pass,
                                    role_assigner_data.input_formation[ap_idx]);
            }
        }
    }

    // calculate path for this robot
    calculatePathForRobot(role_assigner_data, role_assigner_data.this_player_idx);
    if (role_assigner_data.parameters.calculateAllPaths) {
        // Calculate Robot Planner path for other robots
        for (unsigned idx = 0; idx < role_assigner_data.team_admin.size(); idx++) {
            // all robots except this-robot (already calculated)
            if (role_assigner_data.this_player_idx != idx) {
                if (role_assigner_data.team_admin[idx].assigned )  {
                    // calculate path for robot.
                    calculatePathForRobot(role_assigner_data, idx);
                }
            }
        }
    }

    if (role_assigner_data.gamestate == NORMAL_DEFEND and
        (isOneOf(role_assigner_data.ball.status, {FREE, OWNED_BY_TEAMMATE}))) {
        // replan interceptor with local ball only if interceptor is not performing a priority block
        int interceptorIdx = -1;
        for (unsigned idx = 0; idx < role_assigner_data.team.size(); idx++) {
            if (role_assigner_data.team_admin[idx].assigned)  {
                if (role_assigner_data.team_admin[idx].result.role == role_e::role_ATTACKER_MAIN) {
                    interceptorIdx = idx;
                }

            }
        }

        bool replan = false;
        if ((interceptorIdx == (int) role_assigner_data.this_player_idx) || role_assigner_data.parameters.calculateAllPaths) {
            replan = (interceptorIdx != -1)
                      and (role_assigner_data.team_admin[interceptorIdx].result.path.size() > 0)
                      and (role_assigner_data.team_admin[interceptorIdx].result.path[role_assigner_data.this_player_idx].target != PRIORITY_BLOCK);
        }
        if (replan and interceptorIdx >= 0) {
            ReplanInterceptor(interceptorIdx, role_assigner_data);
        }
    }

    // loop over the player path to verify if path of this player stay within the allowed boundaries (field+safety area)
    bool thisPlayerHasUnallowedPath = false;
    bool thisPlayerStartsAtUnallowedPosition = false;
    for (unsigned idx = 0; idx < role_assigner_data.team.size(); idx++) {
        RoleAssignerResult player_result;
        if (role_assigner_data.team_admin[idx].assigned) {
            bool pathOK = stayPathWithinBoundaries(role_assigner_data.fieldConfig, role_assigner_data.team_admin[idx].result);
            player_result = role_assigner_data.team_admin[idx].result;
            player_result.role = role_assigner_data.team_admin[idx].result.role;
            if (not pathOK && idx == role_assigner_data.this_player_idx) { // CHECK Only this robot
                thisPlayerHasUnallowedPath = true; // this robot has wrong path

                // check if player starts at position in the field
                auto currentPos = role_assigner_data.team[idx].position;
                thisPlayerStartsAtUnallowedPosition = !role_assigner_data.fieldConfig.isInReachableField(currentPos.x, currentPos.y);
            }
            player_paths.push_back(player_result);
        }
    }

    bool dynamicRoleNoneAssigned =  role_assigner_data.team_admin[role_assigner_data.this_player_idx].result.role == role_e::role_UNDEFINED;
    // Dump diagnostics if any robot moves outside the allowed field (safety border + field)
    if (thisPlayerHasUnallowedPath || role_assigner_data.parameters.svgOutputFileName.length() > 0 || dynamicRoleNoneAssigned) {
        // Create diagnostics file (.svg with comments).
        // file can be used to diagnose with path to illegal locations are planned
        std::vector<player_type_e> teamTypes = std::vector<player_type_e>();
        std::vector<long> robotIds = std::vector<long>();

        for (unsigned r_idx = 0; r_idx < role_assigner_data.team.size(); r_idx++) {
            robotIds.push_back(role_assigner_data.team[r_idx].robotId);
            teamTypes.push_back(role_assigner_data.team[r_idx].player_type);
        }
        RoleAssignerParameters parameters = role_assigner_data.parameters;
        std::string save_name = parameters.svgOutputFileName;
        if (thisPlayerHasUnallowedPath) {
            if (thisPlayerStartsAtUnallowedPosition) {
                save_name = GetRoleAssignerSVGname(role_assigner_data.gamestate, "OUTSIDE_FIELD_BEGIN_ERROR");
            }
            else {
                save_name = GetRoleAssignerSVGname(role_assigner_data.gamestate, "OUTSIDE_FIELD_END_ERROR");
            }
        }
        if (dynamicRoleNoneAssigned)
        {
            save_name = GetRoleAssignerSVGname(role_assigner_data.gamestate, "DYN_ROLE_NONE");
        }
        RoleAssignerSvg::role_assigner_data_to_svg(player_paths, role_assigner_data, role_assigner_data.fieldConfig, save_name);

        // create empty path for robot with a path that ends outside the field.
        std::vector<path_piece_t> path = player_paths[role_assigner_data.this_player_idx].path;
        if (not path.empty()) {
            auto end_position = path[path.size()-1];
            if (not role_assigner_data.fieldConfig.isInReachableField(end_position.x,end_position.y)) {
                // end position is unreachable: create empty path
                player_paths[role_assigner_data.this_player_idx] = RoleAssignerResult();
            }
        }
    }

    // ----------------------------------------------------------
    // Put the player_paths in the same order as Team was defined by the client
    // Can be remove if client provide team in correct ordere
    std::vector<RoleAssignerRobot> restored_order_team = {};
    std::vector<RoleAssignerAdminTeam> restored_order_team_admin = {};

    for (auto org_idx = 0u; org_idx < orginal_order_robotIds.size(); org_idx++) {
        for (unsigned team_idx = 0; team_idx < role_assigner_data.team.size(); team_idx++) {
            if (role_assigner_data.team[team_idx].robotId == orginal_order_robotIds[org_idx]) {
                restored_order_team.push_back(role_assigner_data.team[team_idx]);
                restored_order_team_admin.push_back(role_assigner_data.team_admin[team_idx]);
            }
        }
    }
    role_assigner_data.team = restored_order_team;
    role_assigner_data.team_admin = restored_order_team_admin;

    std::vector<RoleAssignerResult> player_paths_in_correct_order;
    for (unsigned team_idx = 0; team_idx < role_assigner_data.team_admin.size(); team_idx++) {
        player_paths_in_correct_order.push_back(role_assigner_data.team_admin[team_idx].result);
    }

    return player_paths_in_correct_order;
}

//---------------------------------------------------------------------------------------------------------------------
void
RoleAssigner::calculatePathForRobot (RoleAssignerData &r_role_assigner_data, unsigned idx) {

    MRA::Geometry::Point BallTargetPos = MRA::Geometry::Point (r_role_assigner_data.pass_data.target_pos.x, r_role_assigner_data.pass_data.target_pos.y);

    bool avoidBallPath = (r_role_assigner_data.gamestate == game_state_e::NORMAL_ATTACK)
                    and r_role_assigner_data.pass_data.valid; // avoid ball path only if pass made (or shot on goal) during normal play (normal_attack)
//    if (r_role_assigner_data.team_admin[idx].result.target_position_is_end_position_of_pass) {
//        // in case of pass, don't avoid ball path if player is destination of the pass
//        avoidBallPath = false;
//    }

    vector<RoleAssignerRobot> myTeam = getTeamMates (r_role_assigner_data, idx, true);
    GlobalPathPlanner visibilityGraph = GlobalPathPlanner (r_role_assigner_data.fieldConfig); // create robot planner
    visibilityGraph.setOptions (r_role_assigner_data.parameters);
    // create list of possible targets for robot-planner
    std::vector<MRA::Vertex> targetPos = vector<MRA::Vertex> ();

    // check if role position is in allowed, if not allowed update position to an allowed positions
    r_role_assigner_data.team_admin[idx].result.target = updatePositionIfNotAllowed (
                    r_role_assigner_data.team[idx].position, r_role_assigner_data.team_admin[idx].result.role,
                    r_role_assigner_data.team_admin[idx].result.target, r_role_assigner_data.fieldConfig);

    bool stay_in_playing_field = stayInPlayingField (r_role_assigner_data.gamestate);

    targetPos.push_back (Vertex (r_role_assigner_data.team_admin[idx].result.target, 0));
    visibilityGraph.createGraph (r_role_assigner_data.team[idx].position, r_role_assigner_data.team[idx].velocity, r_role_assigner_data.ball,
                                 myTeam, r_role_assigner_data.opponents,
                                 targetPos,r_role_assigner_data.team_admin[idx].result.planner_target,
                                 r_role_assigner_data.ballIsObstacle,
                                 avoidBallPath, stay_in_playing_field, BallTargetPos);

    r_role_assigner_data.team_admin[idx].result.path = visibilityGraph.getShortestPath(r_role_assigner_data);
}

//---------------------------------------------------------------------------------------------------------------------
// check if the given path is always within the boundaries
bool RoleAssigner::stayPathWithinBoundaries(const FieldConfig& fieldConfig, const RoleAssignerResult& player_path) {
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

planner_target_e RoleAssigner::determine_planner_target(role_e role, const RoleAssignerData& role_assigner_data) {
    planner_target_e target = planner_target_e::GOTO_TARGET_POSITION;
    if (role_assigner_data.searchForBall) {
        target = planner_target_e::GOTO_TARGET_POSITION;
    }
    else if (role == role_GOALKEEPER) {
        target = planner_target_e::GOALKEEPER;
    }
    else if (role_assigner_data.gamestate == BEGIN_POSITION) {
        target = planner_target_e::GOTO_TARGET_POSITION;
    }
    else if (role_assigner_data.gamestate == PARKING) {
        target = planner_target_e::GOTO_TARGET_POSITION_SLOW;
    }
    else if (role == role_ATTACKER_ASSIST) {
        target = planner_target_e::PREPARE_RESTART;
    }
    else if (role == role_DEFENDER_MAIN) {
        target = planner_target_e::SWEEPER;
    }
    else if (role == role_ATTACKER_GENERIC) {
        target = planner_target_e::SUPPORT_ATTACK;
    }
    else if (role == role_DEFENDER_GENERIC) {
        if (isOneOf(role_assigner_data.gamestate, {PENALTY_SHOOTOUT, PENALTY_SHOOTOUT_AGAINST})) {
            target = planner_target_e::GOTO_TARGET_POSITION;
        }
        else {
            target = planner_target_e::SUPPORT_DEFENSE;
        }
    }
    else if (role == role_ATTACKER_MAIN) {
        if (isOneOf(role_assigner_data.gamestate, {CORNER, FREEKICK, GOALKICK, KICKOFF, THROWIN})) {
            target = planner_target_e::PREPARE_RESTART;
        }
        else if (isOneOf(role_assigner_data.gamestate, {PENALTY, PENALTY_SHOOTOUT})) {
            target = planner_target_e::PREPARE_RESTART;
        }
        else if (isOneOf(role_assigner_data.ball.status, {OWNED_BY_PLAYER, OWNED_BY_TEAMMATE})) {
            target = planner_target_e::DRIBBLE;
        }
       else {
           target = planner_target_e::GOTO_BALL;
        }
    }
    return target;
}
//---------------------------------------------------------------------------------------------------------------------
vector<MRA::Geometry::Position> RoleAssigner::getOpponents(const std::vector<RoleAssignerOpponent>&  Opponents) {
    vector<MRA::Geometry::Position> opponents = vector<MRA::Geometry::Position>();
    for (unsigned int idx = 0; idx < Opponents.size(); idx++) {
        opponents.push_back(Opponents[idx].position);
    }
    return opponents;
}

bool RoleAssigner::AssignAnyRobotPreferedSetPlayer(RoleAssignerData&  role_assigner_data, role_e role,
                                               planner_target_e planner_target, const MRA::Geometry::Point& targetPos, role_e org_role) {
    // RoleAssigner will first select the receiver during set-play.
    // Planner will look to all available (unassigned) field-players.
    // If preferred receiver is available, then it will selected. Else it will select the lowest player-id which is not the kicker.
    // If not available (only kicker on the field), then the kicker will be selected.
    //    Same algorithm will be applied for the kicker (but receiver is then already be assigned).

    int preferredKickerRobotId = role_assigner_data.parameters.preferredSetplayKicker;
    int preferredReceiverRobotId = role_assigner_data.parameters.preferredSetplayReceiver;
    int lowestAvailablePlayerIdx = INT_MAX;
    int preferredReceiverTeamIdx = -1;
    int preferredKickerTeamIdx = -1;

    for (unsigned int idx = 0; idx < role_assigner_data.team.size(); idx++) {
        if (role_assigner_data.team_admin[idx].assigned == false) {
            // player can be assigned
            if (role_assigner_data.team[idx].robotId == preferredReceiverRobotId) {
                preferredReceiverTeamIdx = idx;
            }
            else if (role_assigner_data.team[idx].robotId == preferredKickerRobotId) {
                preferredKickerTeamIdx = idx;
            }
            else if (lowestAvailablePlayerIdx == INT_MAX) {
                lowestAvailablePlayerIdx = idx;
            }
            else if (role_assigner_data.team[idx].robotId <= role_assigner_data.team[lowestAvailablePlayerIdx].robotId) {
                lowestAvailablePlayerIdx = idx;
            }
        }
    }
    int foundPlayer = -1;
    if (role == role_ATTACKER_ASSIST) {
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
    if (role == role_ATTACKER_MAIN and isOneOf(role_assigner_data.gamestate, {CORNER, FREEKICK, GOALKICK, KICKOFF, THROWIN})) {
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
        role_assigner_data.team_admin[foundPlayer].result = RoleAssignerResult(
            role,
            role_assigner_data.incrementAndGetRank(),
            targetPos,
            planner_target);
        role_assigner_data.team_admin[foundPlayer].assigned = true;

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
bool RoleAssigner::assignAnyToPosition(RoleAssignerData&  role_assigner_data, role_e role,
            const MRA::Geometry::Point& target, planner_target_e planner_target, bool role_position_is_end_position_of_pass,
            role_e org_role)
{
    bool found = false; // return value
    vector<MRA::Geometry::Point> targets = vector<MRA::Geometry::Point>();
    targets.push_back(target);

    // ------------------------------------------------------------

    // assign Preferred set player roles if applicable
    if (AssignAnyRobotPreferedSetPlayer(role_assigner_data, role, planner_target, target, org_role)) {
        return true; // preferred set player found and assigned
    }

    if (role_position_is_end_position_of_pass && role_assigner_data.pass_data.valid) {
        /* only if role_position is end position of pass and pass data is valid*
         * find player with same id as target_id in pass data (that player is destination of pass) */
        // loop over all players
        for (unsigned int idx = 0; idx < role_assigner_data.team.size(); idx++) {
            if (role_assigner_data.team_admin[idx].assigned) {
                continue;  // player already assigned, skip
            }
            if (role_assigner_data.team[idx].robotId == role_assigner_data.pass_data.target_id) {
                /* this player is destination of the pass */
                role_assigner_data.team_admin[idx].result = RoleAssignerResult(
                    role,
                    role_assigner_data.incrementAndGetRank(),
                    target,
                    planner_target,
                    role_assigner_data.defend_info,
                    role_position_is_end_position_of_pass);
                role_assigner_data.team_admin[idx].assigned = true;
                return true;
            }
        }
    }

    vector <AssignToTargetData> assignToTargetData = vector<AssignToTargetData>();
    MRA::Geometry::Point currentEndPos = target;  // intended target, can be slightly different than calculated (if path is blocked);

    const int BEST_PLAYER_UNASSIGNED = -1;
    int bestPlayerIdx = BEST_PLAYER_UNASSIGNED;
    // do all criteria calculations for the team and store it in AssignToTargetData structs for all robots (incl. assigned)
    for (unsigned int idx = 0; idx < role_assigner_data.team.size(); idx++) {
        AssignToTargetData player = {};
        player.available = not role_assigner_data.team_admin[idx].assigned;

        if (role == role_ATTACKER_MAIN and isOneOf(role_assigner_data.ball.status, {OWNED_BY_PLAYER, OWNED_BY_TEAMMATE}) and not role_assigner_data.team[idx].controlBall) {
            player.available = false; // dribble can only be done, by player with ball, so skip this robot
        }
        if (role == role_ATTACKER_MAIN and not isOneOf(role_assigner_data.ball.status, {OWNED_BY_PLAYER, OWNED_BY_TEAMMATE}) and role_assigner_data.team[idx].passBall) {
            player.available = false; // ball should not be intercepted by player who passed
        }

        if (player.available) {
            if (bestPlayerIdx == BEST_PLAYER_UNASSIGNED) {
                bestPlayerIdx = idx;
            }
            // Calculate full stop position, in the direction of current velocity vector
            // Calculate full stop position, in the direction of current velocity vector
            Geometry::Point linVelocity(role_assigner_data.team[idx].velocity.x, role_assigner_data.team[idx].velocity.y);

            double acc = role_assigner_data.parameters.maxPossibleLinearAcceleration;
            double vel = linVelocity.size();
            double stopDistance = acc == 0 ? 0 : 1/2 * vel * vel / acc;
            MRA::Geometry::Point  stopDist(linVelocity);
            stopDist.normalize();
            stopDist *= stopDistance;
            MRA::Geometry::Point fullStopPos = role_assigner_data.team[idx].position;
            fullStopPos += stopDist;

            // calculate distance from robot's full stop position to target
            player.distToTarget = currentEndPos.distanceTo(fullStopPos);

            player.distToPreviousTarget = 0.0;
            // calculate previous target position distance-threshold.
            if (role_assigner_data.team_admin[idx].previous_result.present and role_assigner_data.team_admin[idx].previous_result.role == role)
            {
                Geometry::Point previousEndPos = Geometry::Point(role_assigner_data.team_admin[idx].previous_result.end_position.x,
                                                                 role_assigner_data.team_admin[idx].previous_result.end_position.y);
                if (currentEndPos.distanceTo(previousEndPos) < role_assigner_data.parameters.previous_role_end_pos_threshold) {
                    player.distToPreviousTarget = role_assigner_data.parameters.previous_role_bonus_end_pos_radius;
                }
            }
            player.totalCost = player.distToTarget - player.distToPreviousTarget;
        }
        assignToTargetData.push_back(player);

    }

    if (bestPlayerIdx != BEST_PLAYER_UNASSIGNED) {

        for (unsigned int idx = 0; idx < role_assigner_data.team.size(); idx++) {
            if (not assignToTargetData[idx].available) {
                continue;
            }

            if (idx == (unsigned) bestPlayerIdx) {
                continue;
            }
            // compare this robot with the current best robot.
            // best robot will become the new best robot

            if (fabs(assignToTargetData[idx].totalCost - assignToTargetData[bestPlayerIdx].totalCost) < role_assigner_data.parameters.equality_cost_threshold) {
                // equality: lowest robot id wins
                if (role_assigner_data.team[idx].robotId < role_assigner_data.team[bestPlayerIdx].robotId) {
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
        role_assigner_data.team_admin[bestPlayerIdx].result = RoleAssignerResult(
            role,
            role_assigner_data.incrementAndGetRank(),
            target,
            planner_target,
            role_assigner_data.defend_info,
            role_position_is_end_position_of_pass);
        role_assigner_data.team_admin[bestPlayerIdx].assigned = true;
    }
    return found;
}


//---------------------------------------------------------------------------------------------------------------------
// return true if for the new path the costs are lower than the costs of the lowest path. If lower check apply hysteresis threshold is passed.
// update lowest_pathcost only if path is faster and newCost are lower than current lowest_pathcost. This to prevent slippery target value.
bool RoleAssigner::check_better_path_found(double& lowest_pathcost, double newPathCost,
        double fastestPathCost, const RoleAssignerResult& new_path,
        const RoleAssignerResult& fastest_path, double equality_cost_threshold)
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
void RoleAssigner::assignGoalie(RoleAssignerData& role_assigner_data)
{
    double goalieYPosition = -role_assigner_data.fieldConfig.getMaxFieldY()+0.5;
    Geometry::Point goaliePosition = Geometry::Point(0, goalieYPosition); // center of the goal;
    if (role_assigner_data.gamestate == game_state_e::PARKING) {
        // select position closest to default goalie position as parking position for the goalie
        Geometry::Point startPost = Geometry::Point(0, -role_assigner_data.fieldConfig.getMaxFieldY());
        goaliePosition = RolePosition::closestTo(startPost, role_assigner_data.parking_positions);
    }
    else {
        if (role_assigner_data.ball.is_valid) {
            MRA::Geometry::Point ballPos = role_assigner_data.ball.position;
            double penalty_area_half_width = role_assigner_data.fieldConfig.getPenaltyAreaWidth() * 0.5;
            if (ballPos.x < -penalty_area_half_width) {
                //            if (global) ball is to the left of the field (left from outer line penalty area) position keeper to the left.
                goaliePosition = MRA::Geometry::Point(-role_assigner_data.fieldConfig.getGoalWidth()*0.25, goalieYPosition); // left half of the goal;
            }
            else if (ballPos.x > penalty_area_half_width) {
                goaliePosition = MRA::Geometry::Point(+role_assigner_data.fieldConfig.getGoalWidth()*0.25, goalieYPosition); // right half of the goal;
            }
        }
    }

    unsigned keeper_idx = 0;
    bool keeperFound = false;

    for (unsigned int idx = 0; keeperFound == false && idx < role_assigner_data.team.size(); idx++) {
        if (role_assigner_data.team[idx].player_type == GOALIE) {
            keeperFound = true;
            keeper_idx = idx;
        }
    }

    if (not keeperFound && role_assigner_data.parameters.autoAssignGoalie) {
        // no dedicated goalie found and auto-assign goalie is enabled.

        //find preferred setplay kicker and receiver
        int setPlayKickerIdx = -1;
        int setPlayReceiverIdx = -1;
        int preferredKickerRobotId = role_assigner_data.parameters.preferredSetplayKicker;
        int preferredReceiverRobotId = role_assigner_data.parameters.preferredSetplayReceiver;
        vector<int> availablePlayers = vector<int>();

        if (preferredReceiverRobotId == preferredKickerRobotId)
        {
            preferredReceiverRobotId = 0; // avoid select conflict. prefer kicker above receiver.
        }

        for (unsigned int idx = 0; idx < role_assigner_data.team.size(); idx++) {
                // player can be assigned
                if (role_assigner_data.team[idx].robotId == preferredReceiverRobotId) {
                    setPlayReceiverIdx = idx;
                }
                else if (role_assigner_data.team[idx].robotId == preferredKickerRobotId) {
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
            else if (role_assigner_data.team[player_idx].robotId <= role_assigner_data.team[lowestAvailablePlayerIdx].robotId) {
                lowestAvailablePlayerIdx = player_idx; // better goalie == lower robot_id
            }
        }
        keeper_idx = lowestAvailablePlayerIdx;
    }

    if (keeperFound) {
        role_assigner_data.team_admin[keeper_idx].assigned = true;
        defend_info_t defend_info = {};
        role_assigner_data.team_admin[keeper_idx].result = RoleAssignerResult(
            role_assigner_data.gamestate == game_state_e::PARKING ? DynamicRoleToRole(dr_PARKING, role_GOALKEEPER) : DynamicRoleToRole(dr_GOALKEEPER, role_GOALKEEPER),
            role_assigner_data.incrementAndGetRank(),
            goaliePosition,
            role_assigner_data.gamestate == game_state_e::PARKING ? planner_target_e::GOTO_TARGET_POSITION_SLOW : planner_target_e::GOALKEEPER,
            defend_info); // no defend info for goalie
    }

    return;
}

//----------------------------------------------------------------------------------------
void RoleAssigner::assignTooLongInPenaltyAreaPlayers(RoleAssignerData&  role_assigner_data)
{
    // max allowed time in penalty area in 2019: 10 seconds (attack and defense)
    const double TIME_TOO_LONG_IN_PENALTY_AREA_THRESHOLD = 8.0;  // 2 seconds to move outside the penalty area
    if (role_assigner_data.gamestate != NONE && role_assigner_data.gamestate != PARKING) {
        double d_x = role_assigner_data.fieldConfig.getPenaltyAreaWidth() * 0.5;
        double d_y = -(role_assigner_data.fieldConfig.getFieldLength()/2) + role_assigner_data.fieldConfig.getPenaltyAreaLength();
        for (unsigned int idx = 0; idx < role_assigner_data.team.size(); idx++) {
            if (not role_assigner_data.team_admin[idx].assigned) {

                if (role_assigner_data.team[idx].time_in_own_penalty_area > TIME_TOO_LONG_IN_PENALTY_AREA_THRESHOLD) {
                    // player not assigned and too long in penalty area
                    MRA::Geometry::Point pos = role_assigner_data.team[idx].position;
                    if (role_assigner_data.fieldConfig.isInOwnPenaltyArea(pos.x, pos.y)) {
                        double d_y = -(role_assigner_data.fieldConfig.getFieldLength()/2) + role_assigner_data.fieldConfig.getPenaltyAreaLength();
                        MRA::Geometry::Point targetPos = pos;
                        if (fabs(d_y - pos.y) < fabs(d_x - fabs(pos.x))) {
                            // closest to Y line
                            targetPos.y = d_y + role_assigner_data.fieldConfig.getRobotRadius();
                        }
                        else {
                            // closest to side of penalty area
                            double safe_x_pos = d_x + role_assigner_data.fieldConfig.getRobotRadius();
                            if (pos.x > 0) {
                                targetPos.x = safe_x_pos;
                            }
                            else {
                                targetPos.x = -safe_x_pos;
                            }
                        }
                        role_assigner_data.team_admin[idx].assigned = true;
                        role_assigner_data.team_admin[idx].result = RoleAssignerResult(
                            DynamicRoleToRole(dr_DEFENDER, role_DEFENDER_GENERIC),
                            role_assigner_data.incrementAndGetRank(),
                            targetPos,
                            planner_target_e::SUPPORT_DEFENSE,
                            role_assigner_data.defend_info);
                    }
                }
                if (role_assigner_data.team[idx].time_in_opponent_penalty_area > TIME_TOO_LONG_IN_PENALTY_AREA_THRESHOLD) { // TODO
                    // player not assigned and too long in penalty area
                    MRA::Geometry::Point pos = role_assigner_data.team[idx].position;
                    if (role_assigner_data.fieldConfig.isInOpponentPenaltyArea(pos.x, pos.y)) {
                        double d_y2 = -d_y;
                        MRA::Geometry::Point targetPos = pos;
                        if (fabs(d_y2 - pos.y) < fabs(d_x - fabs(pos.x))) {
                            // closest to Y line
                            targetPos.y = d_y2 - role_assigner_data.fieldConfig.getRobotRadius();
                        }
                        else {
                            // closest to side of penalty area
                            double safe_x_pos = d_x + role_assigner_data.fieldConfig.getRobotRadius();
                            if (pos.x > 0) {
                                targetPos.x = safe_x_pos;
                            }
                            else {
                                targetPos.x = -safe_x_pos;
                            }
                        }
                        role_assigner_data.team_admin[idx].assigned = true;
                        role_assigner_data.team_admin[idx].result = RoleAssignerResult(
                                                    DynamicRoleToRole(dr_DEFENDER, role_DEFENDER_GENERIC),
                                                    role_assigner_data.incrementAndGetRank(),
                                                    targetPos,
                                                    planner_target_e::SUPPORT_DEFENSE,
                                                    role_assigner_data.defend_info);
                    }
                }
            }
        }
    }
}

//----------------------------------------------------------------------------------------
vector<RoleAssignerRobot> RoleAssigner::getTeamMates(const RoleAssignerData& role_assigner_data, unsigned meIdx, bool addAssignedTargetAsTeamPosition) {
    vector<RoleAssignerRobot> teammates = {};
    for (unsigned int idx = 0; idx < role_assigner_data.team.size(); idx++) {
        if (idx != meIdx) {
            teammates.push_back(role_assigner_data.team[idx]);
            if (addAssignedTargetAsTeamPosition && role_assigner_data.team_admin[idx].assigned) {
                //add target position as barrier
                if (role_assigner_data.team_admin[idx].result.path.size() > 0) {
                    path_piece_t last_piece = role_assigner_data.team_admin[idx].result.path[role_assigner_data.team_admin[idx].result.path.size()-1];
                    RoleAssignerRobot barrierRobot = {};
                    barrierRobot.position = Geometry::Position(last_piece.x, last_piece.y);
                    teammates.push_back(barrierRobot);
                }
            }
        }
    }
     return teammates;
}

//---------------------------------------------------------------------------------------------------------------------
bool RoleAssigner::searchForBallBehaviorNeeded(RoleAssignerData& role_assigner_data) {
    bool searchForBall = false;
    if (not role_assigner_data.ball.is_valid) {
        // no ball seen
        if ((role_assigner_data.gamestate != game_state_e::BEGIN_POSITION) && (role_assigner_data.gamestate != game_state_e::PARKING)) {
            searchForBall = true; // state requires search for ball
        }
    }
    else {
        // ball is valid, check if ball position is with the possible area during a game.
        if (not role_assigner_data.fieldConfig.isInReachableField(role_assigner_data.ball.position.x, role_assigner_data.ball.position.y)    ) {
            // ball is outside field (not in field and not in safety area)
            searchForBall = true; // state requires search for ball
        }
    }
    return searchForBall;
}

//---------------------------------------------------------------------------------------------------------------------
// print the inputs of RoleAssigner::Assign for debug purposes
void RoleAssigner::printAssignInputs(const RoleAssignerData& role_assigner_data)
{
    cerr << "Team_Planner::assign  inputs:" << endl << flush;
    cerr << "game_state_e: " << role_assigner_data.gamestate << " (" << GameStateAsString(role_assigner_data.gamestate) << " )"<< endl << flush;
    cerr << "global ball: " << role_assigner_data.ball.toString(true) << endl << flush;
    for (unsigned idx = 0; idx < role_assigner_data.team.size(); idx++) {
        cerr << "Robot [" << idx << "] =" << endl << role_assigner_data.team[idx].toString() << endl;
    }
    cerr << "Opponents size: " << role_assigner_data.opponents.size()  << endl << flush;
    for (unsigned int i = 0; i < role_assigner_data.opponents.size(); i++) {
        cerr << "Opponents[" << i << "].position: "<< role_assigner_data.opponents[i].position.toString()  << endl << flush;
    }
    cerr << "parameters: " << role_assigner_data.parameters.toString() << endl << flush;
    cerr << "parking positions size: " << role_assigner_data.parking_positions.size() << endl << flush;
    for (unsigned int idx = 0; idx < role_assigner_data.parking_positions.size(); idx++) {
        cerr << role_assigner_data.parking_positions[idx].toString() << endl << flush;
    }

    if (role_assigner_data.ball_pickup_position.valid)
        cerr << "pickup: valid, x:" << std::setprecision(2) << role_assigner_data.ball_pickup_position.x << " y: " << role_assigner_data.ball_pickup_position.y << " ts:" << role_assigner_data.ball_pickup_position.ts << endl << flush;
    else{
        cerr << "pickup: invalid " << endl << flush;
    }

    cerr << "passIsRequired: " << (role_assigner_data.passIsRequired ? "true" : "false") << endl << flush;

    if (role_assigner_data.pass_data.valid) {
        cerr << "pass_data: valid target_id: " << role_assigner_data.pass_data.target_id << endl << flush;
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

void RoleAssigner::ReplanInterceptor(unsigned interceptorIdx, RoleAssignerData&  role_assigner_data)
{
    if (role_assigner_data.ball.is_valid == false) {
        return; // invalid ball, can not calculate new path
    }
    // this player is interceptor and game state is normal.
    // Replan: using dynamic path planner and local ball.
    double shortestDistanceOpponentToBall = calculateShortestDistanceObjectsToTarget(getOpponents(role_assigner_data.opponents), role_assigner_data.ball.position);
    int nrDynamicPlannerIterations = role_assigner_data.parameters.nrDynamicPlannerIterations;
    double maxSpeed = role_assigner_data.parameters.maxPossibleLinearSpeed;
    double distRobotToBall = role_assigner_data.team[role_assigner_data.this_player_idx].position.distanceTo(role_assigner_data.ball.position);
    // check if ball approach from best angle must be applied.
    if (distRobotToBall < shortestDistanceOpponentToBall + 1.0) {
        // always approach from outside when the robot is closer to the ball than any opponent + 1.0 meter
        role_assigner_data.parameters.addBallApproachVertices = true;
        role_assigner_data.parameters.distToapplyBallApproachVertices = role_assigner_data.fieldConfig.getFieldWidth();
    } else {
        // go straight to the ball
        role_assigner_data.parameters.addBallApproachVertices = false;
    }

    vector<MRA::Vertex> targetPos = vector<MRA::Vertex>();
    targetPos.push_back(Vertex(role_assigner_data.ball.position, 0));
    std::vector<path_piece_t> path;

    vector<RoleAssignerRobot> myTeam = getTeamMates(role_assigner_data, interceptorIdx, true);
    bool stay_in_playing_field = stayInPlayingField (role_assigner_data.gamestate);

    if (nrDynamicPlannerIterations <= 0) {
        bool avoidBallPath = false; // intercept should never avoid a passing ball.
        MRA::Geometry::Point BallTargetPos;
        // use normal planner
        // when ball is obstacle or no iterations for dynamic planner is defined.
        GlobalPathPlanner visibilityGraph = GlobalPathPlanner(role_assigner_data.fieldConfig);
        visibilityGraph.setOptions(role_assigner_data.parameters);
        vector<RoleAssignerRobot> myTeam = getTeamMates(role_assigner_data, interceptorIdx, false);
        bool stay_in_playing_field = stayInPlayingField (role_assigner_data.gamestate);
        visibilityGraph.createGraph(role_assigner_data.team[interceptorIdx].position,
                                    role_assigner_data.team[interceptorIdx].velocity, role_assigner_data.ball, myTeam, role_assigner_data.opponents,
                targetPos, planner_target_e::GOTO_BALL, role_assigner_data.ballIsObstacle, avoidBallPath, stay_in_playing_field, BallTargetPos);
        path = visibilityGraph.getShortestPath(role_assigner_data);
    } else {
        // use dynamic robot planner for goto ball
        GlobalPathDynamicPlanner dp = GlobalPathDynamicPlanner();
        path = dp.planPath(    role_assigner_data.team[interceptorIdx].position, role_assigner_data.team[interceptorIdx].velocity,
                               myTeam, role_assigner_data, targetPos,
                planner_target_e::GOTO_BALL, role_assigner_data.ballIsObstacle, maxSpeed,
                nrDynamicPlannerIterations, stay_in_playing_field);
    }

    if (not path.empty()) {
        MRA::Geometry::Point original_role_position(path[path.size()-1].x, path[path.size()-1].y);
        // check if role position is in allowed, if not allowed update position to an allowed positions
        MRA::Geometry::Point role_position = updatePositionIfNotAllowed(role_assigner_data.team[interceptorIdx].position,
                                                            role_assigner_data.team_admin[interceptorIdx].result.role,
                                                            original_role_position, role_assigner_data.fieldConfig);

        vector<MRA::Vertex> roleTargetPos = vector<MRA::Vertex>();
        roleTargetPos.push_back(Vertex(role_position, 0));

        bool avoidBallPath = false; // intercept should never avoid a passing ball.
        MRA::Geometry::Point BallTargetPos;
        GlobalPathPlanner visibilityGraph = GlobalPathPlanner(role_assigner_data.fieldConfig);
        visibilityGraph.setOptions(role_assigner_data.parameters);
        visibilityGraph.createGraph(role_assigner_data.team[interceptorIdx].position, role_assigner_data.team[interceptorIdx].velocity,
                                    role_assigner_data.ball, myTeam, role_assigner_data.opponents,
                roleTargetPos, planner_target_e::GOTO_BALL, role_assigner_data.ballIsObstacle, avoidBallPath, stay_in_playing_field, BallTargetPos);
        path = visibilityGraph.getShortestPath(role_assigner_data);
    }
    role_assigner_data.team_admin[interceptorIdx].result.path = path;
}

////---------------------------------------------------------------------------------------------------------------------
// calculate shortest distance of the vector of objects (own or opponents) to the ball
double RoleAssigner::calculateShortestDistanceObjectsToTarget(const std::vector<MRA::Geometry::Position>& objects, const MRA::Geometry::Position& targetObject) {
    double shortestDistance = std::numeric_limits<double>::infinity();
    for(unsigned int idx = 0; idx < objects.size(); idx++) {
        double distToBall = objects[idx].distanceTo(targetObject);
        if (distToBall < shortestDistance) {
            shortestDistance = distToBall;
        }
    }
    return shortestDistance;
}

void RoleAssigner::assignParkingPositions(RoleAssignerData& role_assigner_data) {
    // assignment parking positions to field players

    // get list with player positions
    vector<MRA::Geometry::Point> fixedPlayerPositions = {};
    RolePosition::GetFixedPositions(fixedPlayerPositions, role_assigner_data); // only positions for field-players

    auto fixed_role_idx = 0u;
    for (auto idx = 0u; idx < role_assigner_data.team.size(); idx++) {
        if (role_assigner_data.team[idx].player_type != player_type_e::GOALIE) {
            // fill best robot data in planner result.
            role_assigner_data.team_admin[idx].result = RoleAssignerResult(
                role_assigner_data.input_formation[idx],
                role_assigner_data.incrementAndGetRank(),
                fixedPlayerPositions[fixed_role_idx],
                planner_target_e::GOTO_TARGET_POSITION_SLOW,
                role_assigner_data.defend_info);
            role_assigner_data.team_admin[idx].assigned = true;
            fixed_role_idx++;
        }
    }
}

void RoleAssigner::assignBeginPositions(RoleAssignerData& role_assigner_data) {
    // Fixed position assignment for field players
    // get list with player positions
    vector<MRA::Geometry::Point> fixedPlayerPositions = {};
    RolePosition::GetFixedPositions(fixedPlayerPositions, role_assigner_data); // only positions for field-players
    bool setPlayKickerPresent = false;
    bool setPlayReceiverPresent = false;
    for (auto idx = 0u; idx < role_assigner_data.team.size (); idx++) {
        if (role_assigner_data.team[idx].robotId == role_assigner_data.parameters.preferredSetplayKicker) {
            setPlayKickerPresent = true;

        }
        if (role_assigner_data.team[idx].robotId == role_assigner_data.parameters.preferredSetplayReceiver) {
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
    for (auto idx = 0u; idx < role_assigner_data.team.size(); idx++) {
        if (role_assigner_data.team[idx].player_type != player_type_e::GOALIE) {
            MRA::Geometry::Point targetPosition = {};

            if (role_assigner_data.team[idx].robotId == role_assigner_data.parameters.preferredSetplayKicker) {
                targetPosition = fixedPlayerPositions[0];  // setplay kicker always on position: 0
            }
            else if (role_assigner_data.team[idx].robotId == role_assigner_data.parameters.preferredSetplayReceiver) {
                // setplay receiver on position: 1 if setplay kicker is present, otherwise on position-0
                targetPosition = fixedPlayerPositions[setPlayReceiverIdx];
            }
            else {
                targetPosition = fixedPlayerPositions[firstFreeBeginPositionIdx++];
            }

            // fill best robot data in planner result.
            role_assigner_data.team_admin[idx].result = RoleAssignerResult(
                role_assigner_data.input_formation[idx],
                role_assigner_data.incrementAndGetRank(),
                fixedPlayerPositions[fixed_role_idx],
                planner_target_e::GOTO_TARGET_POSITION,
                role_assigner_data.defend_info);
            role_assigner_data.team_admin[idx].assigned = true;
            fixed_role_idx++;
        }
    }
}

//--------------------------------------------------------------------------
MRA::Geometry::Point RoleAssigner::updatePositionIfNotAllowed(const MRA::Geometry::Point& playerPosition, role_e role, const MRA::Geometry::Point& original_target_position, const FieldConfig& fieldConfig) {
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
            and (role != role_e::role_GOALKEEPER))
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

bool RoleAssigner::stayInPlayingField(game_state_e gamestate) const {
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

std::vector<dynamic_role_e> RoleAssigner::getListWithRoles(RoleAssignerData& role_assigner_data) {


    std::vector<dynamic_role_e> roles_to_assign = {};
    for (auto idx = 0u; idx < role_assigner_data.input_formation.size(); idx++) {
        auto odr = role_assigner_data.input_formation[idx];
        dynamic_role_e dr = dr_NONE;

        switch (odr) {
            case role_GOALKEEPER:
                dr = dr_GOALKEEPER;
                break;
            case role_ATTACKER_MAIN:
                if (isOneOf(role_assigner_data.gamestate, { FREEKICK, GOALKICK, CORNER, KICKOFF, THROWIN})) {
                    dr = dr_SETPLAY_KICKER;
                }
                else if (isOneOf(role_assigner_data.gamestate, {PENALTY, PENALTY_SHOOTOUT})) {
                    dr = dr_PENALTY_KICKER;
                }
                else if (isOneOf(role_assigner_data.ball.status, {OWNED_BY_PLAYER, OWNED_BY_TEAMMATE})) {
                    dr = dr_BALLPLAYER;
                }
                else if (isOneOf(role_assigner_data.gamestate, { FREEKICK_AGAINST, GOALKICK_AGAINST,
                                             CORNER_AGAINST, KICKOFF_AGAINST,
                                             THROWIN_AGAINST, PENALTY_AGAINST,
                                             DROPPED_BALL})) {
                    dr = dr_INTERCEPTOR;
                }
                else if (role_assigner_data.ball.status == OWNED_BY_TEAM) {
                    dr = dr_ATTACKSUPPORTER;
                }
                else if (isOneOf(role_assigner_data.ball.status, {FREE, OWNED_BY_OPPONENT})) {
                    dr = dr_INTERCEPTOR;
                }
                break;
            case role_ATTACKER_ASSIST:
                if (isOneOf(role_assigner_data.gamestate, { FREEKICK, GOALKICK, CORNER, KICKOFF, THROWIN})) {
                    dr = dr_SETPLAY_RECEIVER;
                }
                else {
                    dr = dr_ATTACKSUPPORTER;
                }
                break;
            case role_ATTACKER_GENERIC:
                dr = dr_ATTACKSUPPORTER;
                break;
            case role_DEFENDER_MAIN:
                if (isOneOf(role_assigner_data.gamestate, { FREEKICK, GOALKICK, CORNER, KICKOFF, THROWIN})) {
                    dr = dr_ATTACKSUPPORTER;
                }
                else {
                    dr = dr_SWEEPER;
                }
                break;
            case role_DEFENDER_GENERIC:
                if (isOneOf(role_assigner_data.gamestate, { PENALTY_SHOOTOUT, PENALTY_SHOOTOUT_AGAINST})) {
                    dr = dr_PENALTY_DEFENDER;
                }
                else {
                    dr = dr_DEFENDER;
                }
                break;
            case role_DISABLED_OUT:
                dr = dr_PARKING;
                break;
            case role_DISABLED_IN:
                dr = dr_BEGIN_POSITION;
                break;
            default:
                break;
        }
        roles_to_assign.push_back(dr);
    }

    return roles_to_assign;
}
