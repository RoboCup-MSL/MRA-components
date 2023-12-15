/*
 * TeamFormation.h
 *
 *  Created on: Sep 11, 2016
 *      Author: jurge
 */

#ifndef TEAMFORMATION_H
#define TEAMFORMATION_H 1

#include "WmTypes.h"
#include <vector>

#include "planner_types.hpp"
#include "PlannerOptions.hpp"

namespace trs {

class TeamFormation {
public:
	static std::vector<dynamic_role_e> selectTeamFormation(team_formation_e formation,
			trs::game_state_e gamestate, bool playerControlBall, bool playerPassedBall, const PlannerOptions& plannerOptions);

private:
	static std::vector<dynamic_role_e> getFormation013(trs::game_state_e gamestate, bool playerControlBall, bool playerPassedBall, const PlannerOptions& plannerOptions);
	static std::vector<dynamic_role_e> getFormation112(trs::game_state_e gamestate, bool playerControlBall, bool playerPassedBall, const PlannerOptions& plannerOptions);
	static std::vector<dynamic_role_e> getFormation211(trs::game_state_e gamestate, bool playerControlBall, bool playerPassedBall, const PlannerOptions& plannerOptions);
	static std::vector<dynamic_role_e> getFormation310(trs::game_state_e gamestate, bool playerControlBall, bool playerPassedBall, const PlannerOptions& plannerOptions);
	static std::vector<dynamic_role_e> roleOnlyFormation(dynamic_role_e dynamic_role);
};

} // namespace

#endif // TEAMFORMATION_H
