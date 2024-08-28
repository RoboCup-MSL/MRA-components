/*
 * TeamFormation.h
 *
 *  Created on: Sep 11, 2016
 *      Author: jurge
 */

#ifndef TEAMFORMATION_H
#define TEAMFORMATION_H 1

#include "planner_types.hpp"
#include "WmTypes.h"
#include <vector>
#include "TeamPlannerParameters.hpp"

namespace MRA {

class TeamFormation {
public:
	static std::vector<dynamic_role_e> selectTeamFormation(team_formation_e formation,
			MRA::game_state_e gamestate, ball_status_e ball_status, const TeamPlannerParameters& parameters);

private:
	static std::vector<dynamic_role_e> getFormation013(MRA::game_state_e gamestate, ball_status_e ball_status, const TeamPlannerParameters& parameters);
	static std::vector<dynamic_role_e> getFormation112(MRA::game_state_e gamestate, ball_status_e ball_status, const TeamPlannerParameters& parameters);
	static std::vector<dynamic_role_e> getFormation211(MRA::game_state_e gamestate, ball_status_e ball_status, const TeamPlannerParameters& parameters);
	static std::vector<dynamic_role_e> getFormation310(MRA::game_state_e gamestate, ball_status_e ball_status, const TeamPlannerParameters& parameters);
	static std::vector<dynamic_role_e> roleOnlyFormation(dynamic_role_e dynamic_role);
};

} // namespace

#endif // TEAMFORMATION_H
