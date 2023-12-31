/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */
#ifndef TEAM_PLANNER_H
#define TEAM_PLANNER_H 1

#include "TeamPlannerExport.hpp"

#include "WmTypes.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include <sys/time.h>
#include <cmath>
#include "GlobalPathPlanner.hpp"
#include "SvgUtils.hpp"
#include "TeamPlannerParameters.hpp"

namespace MRA {

std::string GetTeamPlannerSVGname(game_state_e gamestate, std::string suffix) {
	// get current time, and make a string from it.
	struct tm * timeinfo;
	char buffer [180];
	char buffer_tmp [80];

	int millisec;
	struct timeval tv;

	gettimeofday(&tv, NULL);
	millisec = lrint(tv.tv_usec/1000.0); // Round to nearest millisec
	if (millisec>=1000) { // Allow for rounding up to nearest second
		millisec -=1000;
		tv.tv_sec++;
	}
	timeinfo = localtime(&tv.tv_sec);
	strftime(buffer_tmp, 80, "%Y%m%dT%H%M%S", timeinfo);
	sprintf(buffer, "%s%03d", buffer_tmp, millisec);

    std::string state_string = GameStateAsString(static_cast<game_state_e>(gamestate));
    replace(state_string.begin(), state_string.end(), ' ', '_');
    std::string full_suffix = "";
    if (suffix.length() > 0) {
    	full_suffix = "_" + suffix;
    }
	int robot = 0;
	std::string filename = "export.svg";
	filename =  "planner_p" + std::to_string(robot) + "_" + buffer + "_" + state_string + full_suffix + ".svg";
	return filename;
}


} // namespace

#endif /* TEAM_PLANNER_H */
