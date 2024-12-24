/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */
#ifndef ROLE_ASSIGNGER_RESULT_HPP
#define ROLE_ASSIGNGER_RESULT_HPP 1


#include <vector>

#include "RoleAssigner_types.hpp"

namespace MRA {

class RoleAssignerResult {
public:
    int robotId = -1;
    std::vector<path_piece_t> path = std::vector<path_piece_t>();
    role_e role = role_e::role_UNDEFINED;
    int role_rank = -1;
    game_state_e gamestate = game_state_e::NONE;
    MRA::Geometry::Point target = {};
    planner_target_e planner_target = planner_target_e::GOTO_TARGET_POSITION;
    defend_info_t defend_info = {.valid = false, .defending_id = -1, .dist_from_defending_id = 0.0, .between_ball_and_defending_pos = 1};
    bool is_pass_desitination = false;

    RoleAssignerResult(int robot_id = -1,
                       const game_state_e&  gamestate = game_state_e::NONE,
                       const role_e& role = role_e::role_UNDEFINED,
                       int role_rank = -1,
                       const MRA::Geometry::Point& target = MRA::Geometry::Point(),
                       const planner_target_e& planner_target = planner_target_e::GOTO_BALL,
                       const defend_info_t& defend_info = {.valid = false, .defending_id = -1, .dist_from_defending_id = 0.0, .between_ball_and_defending_pos = 1},
                       bool desitination_of_pass = false) :
                            robotId(robot_id),
                            role(role),
                            role_rank(role_rank),
                            gamestate(gamestate),
                            target(target),
                            planner_target(planner_target),
                            defend_info(defend_info),
                            is_pass_desitination(desitination_of_pass)
                        {

                        }
    void set_path(const std::vector<path_piece_t>& r_path)  {
        path = r_path;
    }

};

} // namespace

#endif // ROLE_ASSIGNGER_RESULT_HPP
