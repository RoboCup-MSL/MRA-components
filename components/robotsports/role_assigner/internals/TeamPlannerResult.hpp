/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */
#ifndef TEAM_PLANNER_RESULT_H
#define TEAM_PLANNER_RESULT_H 1

#include "WmTypes.h"
#include "FieldConfig.h"
#include <vector>

namespace MRA {

class PlayerPlannerResult {
public:
    std::vector<planner_piece_t> path;
    dynamic_role_e dynamic_role;
    int role_rank = -1;
    game_state_e gamestate;
    MRA::Geometry::Point target;
    planner_target_e planner_target;
    defend_info_t defend_info;
    bool target_position_is_end_position_of_pass;

    PlayerPlannerResult(const game_state_e&  gamestate = game_state_e::NONE,
                        const dynamic_role_e& dynamic_role = dynamic_role_e::dr_NONE,
                        int role_rank = -1,
                        const MRA::Geometry::Point& target = MRA::Geometry::Point(),
                        const planner_target_e& planner_target = planner_target_e::GOTO_BALL,
                        const defend_info_t& defend_info = {},
                        bool target_position_is_end_position_of_pass = false) :
                            path(),
                            dynamic_role(dynamic_role),
                            role_rank(role_rank),
                            gamestate(gamestate),
                            target(target),
                            planner_target(planner_target),
                            defend_info(defend_info),
                            target_position_is_end_position_of_pass(target_position_is_end_position_of_pass)
                        {

                        }
};

typedef std::vector<PlayerPlannerResult> team_planner_result_t;

} // namespace

#endif /* TEAM_PLANNER_RESULT_H */
