/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */
#ifndef TEAM_PLANNER_EXPORT_H
#define TEAM_PLANNER_EXPORT_H 1

#include "WmTypes.h"
#include <vector>
#include "FieldConfig.h"
#include <string>
#include "GlobalPathPlanner.hpp"
#include "TeamPlannerParameters.hpp"


namespace MRA {
	std::string GetTeamPlannerSVGname(game_state_e gamestate, std::string suffix = "");
} // namespace

#endif /* TEAM_PLANNER_H */
