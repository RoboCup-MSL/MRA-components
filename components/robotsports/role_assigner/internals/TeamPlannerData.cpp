/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */

#include "TeamPlannerData.hpp"

#include <iomanip>
#include <sstream>
#include <iostream>
#include <cmath>
#include "TeamPlannerResult.hpp"

using namespace std;
using namespace MRA;

std::string TeamPlannerRobot::toString() const {
	std::stringstream buffer;
	buffer << "ID: " + robotId   << " type: " << PlayerTypeAsString(player_type) << endl
		   << "Position x: " << position.x  << " y: " << position.y  << " rz: " << position.rz <<  " control ball: " <<  controlBall << endl
		   << "pass is on its way: " << passBall << endl
		    << "assigned: " << assigned << endl;
	buffer << " dynamic role: " << DynamicRoleAsString(result.dynamic_role) <<
      		  " gamestate: " << GameStateAsString(result.gamestate) << endl;
	if (result.defend_info.valid) {
		buffer << " Defend info: valid: true id: "<< result.defend_info.defending_id;
		buffer << " dist to id: " << result.defend_info.dist_from_defending_id;
		buffer << "	between ball and id: " << result.defend_info.between_ball_and_defending_pos << endl;
	}
	else {
		buffer << " Defend info: valid: false" << endl;
	}

	buffer << "time_in_own_penalty_area:" << time_in_own_penalty_area
		   << " time_in_opponent_penalty_area: " << time_in_opponent_penalty_area << endl;
	buffer <<  " End position: " << result.target.toString() << endl;

	if (previous_result.previous_result_present) {
		buffer << "previous dynamic role: " << DynamicRoleAsString(static_cast<dynamic_role_e>(previous_result.dynamic_role)) << endl;
		buffer << "previous end pos (x, y, ts) : " << previous_result.end_position.x
			   << ",  " << previous_result.end_position.y << " on : " << previous_result.ts << endl;
	}
	else {
		buffer << "NO previous_result" << endl;
	}

	if (!result.path.empty()) {
		buffer <<  " target: " << PlannerTargetAsString(static_cast<planner_target_e>(result.path[result.path.size()-1].target))
			<< " (x: " <<  std::setprecision(3) <<  result.path[result.path.size()-1].x
			<< " y: " << result.path[result.path.size()-1].y << ")" << endl;
	}
	buffer << endl;
//			player_planner_result_t result;
//			bool have_previous_result;
//			previous_planner_result_t previous_result;;
	return buffer.str();
}


std::string TeamPlannerBall::toString(bool full_details) {
	std::stringstream buffer;
	buffer << "ball position: x:" << this->position.x << " y:"<< this->position.y << endl;
	return buffer.str();
}

//std::string TeamPlannerOutput::pathToString() {
//	std::stringstream buffer;
//
//	team_planner_result_t result_paths = *(this->player_paths);
//	for (unsigned player_idx = 0; player_idx != result_paths.size(); player_idx++) {
//		buffer << "path for player  " << player_idx <<  " id: " << this->Team[player_idx].robotId <<  " -> " << DynamicRoleAsString(result_paths[player_idx].dynamic_role) <<  endl;
//		if (result_paths[player_idx].defend_info.valid) {
//			buffer << " Defend info: valid: true id: "<< result_paths[player_idx].defend_info.defending_id;
//			buffer << " dist to id: " << result_paths[player_idx].defend_info.dist_from_defending_id;
//			buffer << "	between ball and id: " << result_paths[player_idx].defend_info.between_ball_and_defending_pos << endl;
//		}
//		else {
//			buffer << " Defend info: valid: false" << endl;
//		}
//		std::vector<planner_piece_t> path = result_paths[player_idx].path;
//		for (unsigned int path_idx = 0; path_idx != path.size(); path_idx++) {
//			buffer << "path piece [ " << path_idx << "]  = (" << path[path_idx].x << ", "<< path[path_idx].y << ")" << endl;
//		}
//
//	}
//	return buffer.str();
//}

