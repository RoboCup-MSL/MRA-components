/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */
#ifndef TEAM_PLANNER_RESULT_H
#define TEAM_PLANNER_RESULT_H 1

#include "MovingObject.h"
#include "WmTypes.h"
#include "FieldConfig.h"
#include <vector>

namespace trs {

class PlayerPlannerResult {
public:
	std::vector<planner_piece_t> path;
	dynamic_role_e dynamic_role;
	game_state_e gamestate;
	MRA::Geometry::Point target;
	planner_target_e planner_target;
	defend_info_t defend_info;
	bool target_position_is_end_position_of_pass;
};

typedef std::vector<PlayerPlannerResult> team_planner_result_t;

} // namespace

#endif /* TEAM_PLANNER_RESULT_H */
