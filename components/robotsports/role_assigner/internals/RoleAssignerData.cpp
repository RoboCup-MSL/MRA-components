/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */

#include "RoleAssignerData.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <iostream>
#include "RoleAssignerResult.hpp"

using namespace std;
using namespace MRA;


    // ball_pickup_position_t ball_pickup_position;
    // bool passIsRequired;
    // pass_data_t pass_data;
    // MRA::Environment environment;

std::string RoleAssignerInput::toString() const {
    std::stringstream buffer;
    buffer << std::fixed << std::setprecision(2)
           << "gamestate: " << GameStateAsString(this->gamestate) << std::endl;
    buffer << "ball: " << this->ball.toString(true) << std::endl;
    buffer << "formation: " << endl;
    for (auto idx = 0u; idx < this->formation.size(); idx++) {
        buffer << "\t[" << idx << "] = " << RoleAsString(this->formation[idx]) << endl;
    }
    buffer << "team: " << endl;
    for (auto idx = 0u; idx < this->team.size(); idx++) {
        buffer << "\t[" << idx << "] = " << this->team[idx].toString() << endl;
    }
    if (this->team.size() == 0) {
        buffer << "\t<NONE>" << endl;
    }
    buffer << "opponents: " << endl;
    for (auto idx = 0u; idx < this->opponents.size(); idx++) {
        buffer << "\t[" << idx << "] = " << this->opponents[idx].toString() << endl;
    }
    if (this->opponents.size() == 0) {
        buffer << "\t<NONE>" << endl;
    }
    buffer << "no_opponent_obstacles: " << endl;
    for (auto idx = 0u; idx < this->no_opponent_obstacles.size(); idx++) {
        buffer << "\t[" << idx << "] = " << this->no_opponent_obstacles[idx].toString() << endl;
    }
    if (this->no_opponent_obstacles.size() == 0) {
        buffer << "\t<NONE>" << endl;
    }
    buffer << "parking_positions: " << endl;
    for (auto idx = 0u; idx < this->parking_positions.size(); idx++) {
        buffer << "\t[" << idx << "] = " << this->parking_positions[idx].toString() << endl;
    }
    if (this->parking_positions.size() == 0) {
        buffer << "\t<NONE>" << endl;
    }
    buffer << "ball_pickup_position: - valid " <<  this->ball_pickup_position.valid << endl;
    if (this->ball_pickup_position.valid) {
        buffer << "\tpickup ball x: "  <<  this->ball_pickup_position.x << endl;
        buffer << "\tpickup ball y: "  <<  this->ball_pickup_position.y << endl;
        buffer << "\tpickup ball ts: " << this->ball_pickup_position.ts << endl;
        buffer << endl;
    }
    buffer << "passIsRequired: " << passIsRequired << endl;
    buffer << "\tpass_data - valid " <<  this->pass_data.valid << endl;
    if (this->pass_data.valid) {
        buffer << "\torigin =" << this->pass_data.origin_pos.toString();
        buffer << "\ttarget = " << this->pass_data.target_pos.toString();
        buffer << "\tvelocity= " << this->pass_data.velocity;
        buffer << "\tangle= " << this->pass_data.angle;
        buffer << "\tts= " << this->pass_data.ts;
        buffer << "\tkicked= " << this->pass_data.kicked;
        buffer << "\ttarget_id= " << this->pass_data.kicked;
        buffer << endl;
    }
    buffer << "environment: " << environment.toString() << endl;
    buffer << endl;


    return buffer.str();
}

std::string RoleAssignerState::toString() const {
    std::stringstream buffer;
    buffer << "\tprevious_ball = " << this->previous_ball.present << endl;
    if (this->previous_ball.present) {
        buffer << " x= " << this->previous_ball.x << " y: " << this->previous_ball.y;
    }
    buffer << endl;
    for (auto idx = 0u; idx < previous_results.size(); idx++) {
        auto prev_res = this->previous_results[idx];
        buffer << "\t\t\tprev result [" << idx << "]: " << prev_res.present;
        if (prev_res.present)
        {
            buffer << " robotId: " << prev_res.robotId
                   << " role: " << RoleAsString(prev_res.role)
                   << " ts: " << prev_res.ts << endl
                   << " end-pos\n\tx: " << prev_res.end_position.x  << " y:  " << prev_res.end_position.y 
                   << " cost: " << prev_res.end_position.cost  
                   << " target: " << PlannerTargetAsString(static_cast<planner_target_e>(prev_res.end_position.target)) 
                   << std::endl;
        }
        buffer << "\n";
    }
    return buffer.str();
}


std::string RoleAssignerBall::toString(bool print_complete) const {
    std::stringstream buffer;
    if (not print_complete) {
    buffer << std::fixed << std::setprecision(2)
            << " x: " << this->position.x
            << " y: " << this->position.y;
    }
    else {

        buffer << std::fixed << std::setprecision(2)
               << " status: " << ballStatusAsString(this->status) 
               << " position: " << this->position.toString() 
               << " velocity: " << this->velocity.toString() 
               << " is_valid: " << (int) this->is_valid;
    }
    return buffer.str();
}


std::string RoleAssignerOutput::toString() const {
    std::stringstream buffer;
    buffer << "paths: " << this->player_paths.size() << std::endl;
    for (auto idx = 0u; idx < this->player_paths.size(); idx++) {
        buffer << "[" << idx << "] = robotId:" << player_paths[idx].robotId << endl;
        buffer << "\tgamestate: " << GameStateAsString(this->player_paths[idx].gamestate) << endl;;
        buffer << "\trole: " << RoleAsString(this->player_paths[idx].role) << " (" << this->player_paths[idx].role << ")" << endl;
        buffer << "\trank: " << this->player_paths[idx].role_rank << endl;;
        buffer << "\ttarget-pos: "<< this->player_paths[idx].target.toString() << endl;
        buffer << "\tis_pass_desitination: "<< this->player_paths[idx].is_pass_desitination << endl;
        buffer << "\tplanner_target: "<< PlannerTargetAsString(this->player_paths[idx].planner_target) << endl;
        buffer << "\tdefend_info.valid = " << player_paths[idx].defend_info.valid<< endl;
        if (this->player_paths[idx].defend_info.valid) {
            buffer << "\t\tdefending_id: " << this->player_paths[idx].defend_info.defending_id;
            buffer << "\t\tbetween_ball_and_defending_pos: " << this->player_paths[idx].defend_info.between_ball_and_defending_pos;
            buffer << "\t\tdist_from_defending_id: " << this->player_paths[idx].defend_info.dist_from_defending_id << endl;
        }
        for (unsigned long p_idx = 0; p_idx < this->player_paths[idx].path.size(); p_idx++) {
            buffer << std::fixed << setprecision(2) << "Path[" << p_idx << "]: x: " << this->player_paths[idx].path[p_idx].x <<
                    " y: " << this->player_paths[idx].path[p_idx].y <<
                    " cost: " << this->player_paths[idx].path[p_idx].cost;
            std::string targetString = PlannerTargetAsString(static_cast<planner_target_e>(this->player_paths[idx].path[p_idx].target));
            buffer << " target: " << targetString << " (" << this->player_paths[idx].path[p_idx].target<< ")"<<  std::endl;
        }
    }

    return buffer.str();
}

std::string RoleAssignerData::toString() const
{
    std::stringstream buffer;

    buffer << std::setw(2) << std::boolalpha; // bool as string
    buffer << "\tgamestate = " << GameStateAsString(this->gamestate) << " (" << this->gamestate << ")" << endl;
    buffer << "\torignal_gamestate = " << GameStateAsString(this->gamestate) << " (" << this->gamestate << ")" << endl;
    string controlBallByPlayerRemark = "";

    int controlBallByPlayerIdx = -1;
    int controlBallById = -1;
    for (auto idx = 0u; idx < this->team.size(); idx++) {
        if (this->team[idx].controlBall) {
            controlBallByPlayerIdx = idx;
            controlBallById = this->team[idx].robotId;
        }
    }

    if (controlBallByPlayerIdx == -1) {
        controlBallByPlayerRemark = "(not controlled by robot in team)";
    }
    else if (controlBallByPlayerIdx == (int) (this->this_player_idx)) {
        controlBallByPlayerRemark = "(controlled by me)";
    }
    else {
        controlBallByPlayerRemark = "(controlled by team-member)";
    }
    buffer << "\tcontrolball = " << controlBallByPlayerRemark;
    if (controlBallByPlayerIdx >= 0) {
        buffer <<" (idx: " << controlBallByPlayerIdx << " id: " << controlBallById << ")";
    }
    buffer << endl;

    buffer << "\tBall: " << ball.toString(true) << endl;
    buffer << "\tprevious_ball = " << this->previous_ball.present << endl;
    if (this->previous_ball.present) {
        buffer << " x= " << this->previous_ball.x << " y: " << this->previous_ball.y;
    }
    buffer << endl;

    buffer << "\tpass_is_required: " << this->passIsRequired << endl;

    buffer << "\tpass_data - valid " <<  this->pass_data.valid << endl;
    if (this->pass_data.valid) {
        buffer << "\t PassData origin_x=" << this->pass_data.origin_pos.x;
        buffer << " origin_y= " << this->pass_data.origin_pos.y;
        buffer << " target_x= " << this->pass_data.target_pos.x;
        buffer << " target_y= " << this->pass_data.target_pos.y;
        buffer << " velocity= " << this->pass_data.velocity;
        buffer << " angle= " << this->pass_data.angle;
        buffer << " ts= " << this->pass_data.ts;
    }
    buffer << endl;;

    buffer << "\tpickup ball valid: " << this->ball_pickup_position.valid << endl;
    buffer << "\tpickup ball x: "  <<  this->ball_pickup_position.x << endl;
    buffer << "\tpickup ball y: "  <<  this->ball_pickup_position.y << endl;
    buffer << "\tpickup ball ts: " << this->ball_pickup_position.ts << endl;
    buffer << "\n";


    buffer << "\tteam:" << endl;
    for (unsigned int idx = 0; idx < this->team.size(); idx++) {
        RoleAssignerRobot rbt = this->team[idx];
        buffer << "\t\tR" << rbt.robotId << " = " << rbt.position.toString() << " type =  ";
        buffer << PlayerTypeAsString(static_cast<player_type_e>(rbt.player_type)) << " (" << rbt.player_type << ")" << endl;
        buffer << "\t\t\tcontrol-ball: " << rbt.controlBall<< " passBall: " << rbt.passBall;
        buffer <<" role: " << RoleAsString(this->team_admin[idx].result.role);
        buffer << "time-own-PA: " << rbt.time_in_own_penalty_area;
        buffer << "time-opp-PA: " << rbt.time_in_opponent_penalty_area << endl;
        auto prev_res = this->getPreviousResultForPlayer(this->team[idx].robotId);
        buffer << "\t\t\tprev result:  " << prev_res.present;
        if (prev_res.present)
        {
            buffer << " role: " << RoleAsString(prev_res.role)
                   << " end-pos x: " << prev_res.end_position.x
                   << " y:  " << prev_res.end_position.y
                   << "target: " << PlannerTargetAsString(static_cast<planner_target_e>(prev_res.end_position.target))
                   << "ts: " << prev_res.ts << endl;
        }
        buffer << "\n";
    }

    buffer << "\topponents:\n";
    for (unsigned int idx = 0; idx < this->opponents.size(); idx++) {
        buffer << "\t\tplayer[" << idx << "] = " << this->opponents[idx].position.toString() << endl;
    }


    buffer << "team admin:\n" << endl;
    for (unsigned long p_idx = 0; p_idx < this->team_admin.size(); p_idx++) {
        long robotId = this->team_admin[p_idx].robotId;
        auto player_path = this->team_admin[p_idx];
        buffer << std::fixed << std::setprecision(2) << endl<< "Player " << p_idx << " (id: " << robotId<< ") : " << std::endl;
        buffer << "\tRole: " << RoleAsString(player_path.result.role) << endl;
        buffer << "\tGame State: " << GameStateAsString(player_path.result.gamestate) << endl;
        buffer << "\tTarget position : " << player_path.result.target.toString() << endl;
        buffer << "\tPlanner target type  " << PlannerTargetAsString(player_path.result.planner_target) << endl;
        buffer << "\tIs pass desitination: " << player_path.result.is_pass_desitination << endl;
        buffer << "\tDefend info valid:  " << player_path.result.defend_info.valid;
        if (player_path.result.defend_info.valid) {
            buffer << " defending_id: " << player_path.result.defend_info.defending_id;
            buffer << " between_ball_and_defending_pos: " << player_path.result.defend_info.between_ball_and_defending_pos;
            buffer << " dist_from_defending_id: " << player_path.result.defend_info.dist_from_defending_id << endl;
        }
        else {
            buffer << endl;
        }
        std::vector<path_piece_t> path;
        for (unsigned long idx = 0; idx < player_path.result.path.size(); idx++) {
            buffer << std::fixed << setprecision(2) << "Path[" << p_idx << "]: x: " << player_path.result.path[idx].x <<
                    " y: " << player_path.result.path[idx].y <<
                    " cost: " << player_path.result.path[idx].cost;
            std::string targetString = PlannerTargetAsString(static_cast<planner_target_e>(player_path.result.path[idx].target));
            buffer << " target: " << targetString << " (" << player_path.result.path[idx].target<< ")"<<  std::endl;
        }
    }

    buffer << "\tparameters:\n" << parameters.toString() << endl;
    buffer << endl;

    return buffer.str();
}


int RoleAssignerData::incrementAndGetRank() {
    return ++nr_players_assigned;
}

bool RoleAssignerData::teamControlsBall() const {
    return isOneOf(ball.status, {OWNED_BY_PLAYER, OWNED_BY_TEAM});
}


previous_role_assigner_result_t RoleAssignerData::getPreviousResultForPlayer(int robotId) const {
    previous_role_assigner_result_t previous_result = { .present=false };

    for (auto idx = 0u; idx < this->previous_results.size(); idx++) {
        if (this->previous_results[idx].robotId == robotId) {
           previous_result = this->previous_results[idx];
        }
    }

    return previous_result;
}
