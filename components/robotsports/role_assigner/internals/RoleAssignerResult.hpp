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
    std::vector<path_piece_t> path;
    role_e role;
    int role_rank = -1;
    MRA::Geometry::Point target;
    planner_target_e planner_target;
    defend_info_t defend_info;
    bool target_position_is_end_position_of_pass;

    RoleAssignerResult(const role_e& role = role_e::role_UNDEFINED,
                       int role_rank = -1,
                       const MRA::Geometry::Point& target = MRA::Geometry::Point(),
                       const planner_target_e& planner_target = planner_target_e::GOTO_BALL,
                       const defend_info_t& defend_info = {},
                       bool target_position_is_end_position_of_pass = false) :
                            path(),
                            role(role),
                            role_rank(role_rank),
                            target(target),
                            planner_target(planner_target),
                            defend_info(defend_info),
                            target_position_is_end_position_of_pass(target_position_is_end_position_of_pass)
                        {

                        }
};

} // namespace

#endif // ROLE_ASSIGNGER_RESULT_HPP
