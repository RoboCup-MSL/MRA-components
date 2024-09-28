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

std::string RoleAssignerBall::toString(bool print_complete) const {
    std::stringstream buffer;
    buffer << std::fixed << std::setprecision(2)
            << " x: " << this->position.x
            << " y: " << this->position.y;
    if (print_complete) {
        buffer << std::fixed << std::setprecision(2)
               << " vx: " << this->velocity.x
               << " vy: " << this->velocity.y
               << " valid: " << (int) this->is_valid;
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
        auto prev_res = this->team_admin[idx].previous_result;
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
        buffer << "\tTarget position : " << player_path.result.target.toString() << endl;
        buffer << "\tPlanner target type  " << PlannerTargetAsString(player_path.result.planner_target) << endl;
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
    return isOneOf(ball.status, {OWNED_BY_PLAYER, OWNED_BY_TEAMMATE, OWNED_BY_TEAM});
}
