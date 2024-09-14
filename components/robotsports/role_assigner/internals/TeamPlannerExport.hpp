/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */
#ifndef TEAM_PLANNER_EXPORT_HPP
#define TEAM_PLANNER_EXPORT_HPP 1

#include "FieldConfig.hpp"
#include "GlobalPathPlanner.hpp"
#include "TeamPlannerParameters.hpp"

#include <string>
#include <vector>

namespace MRA {
    std::string GetTeamPlannerSVGname(game_state_e gamestate, std::string suffix = "");
} // namespace

#endif // TEAM_PLANNER_EXPORT_HPP
