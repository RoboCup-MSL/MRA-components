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

//
//static void addToCSV(const string& rstrId, double value, std::stringstream& rStringStream, const string& rsMRAeparator, bool printHeader, std::stringstream& rHeaderStringStream) {
//	rStringStream << std::fixed << value << rsMRAeparator;
//	if (printHeader) {
//		rHeaderStringStream << rstrId << rsMRAeparator;
//	}
//}
//
//static void addToCSV(const string& rstrId, bool value, std::stringstream& rStringStream, const string& rsMRAeparator, bool printHeader, std::stringstream& rHeaderStringStream) {
//	rStringStream << static_cast<int>(value) << rsMRAeparator;
//	if (printHeader) {
//		rHeaderStringStream << rstrId << rsMRAeparator;
//	}
//}
//
//static void addToCSV(const string& rstrId, int value, std::stringstream& rStringStream, const string& rsMRAeparator, bool printHeader, std::stringstream& rHeaderStringStream) {
//	rStringStream << value << rsMRAeparator;
//	if (printHeader) {
//		rHeaderStringStream << rstrId << rsMRAeparator;
//	}
//}

//static void MovingObjectToCSV(const string& rstrId, const MovingObject& rObject, std::stringstream& rStringStream, const string& rsMRAeparator, bool printHeader, std::stringstream& rHeaderStringStream) {
//    MRA::Geometry::Point linVel;
//	double rotVel;
//	rObject.getVelocity(linVel, rotVel);
//	addToCSV(rstrId+".valid",rObject.isValid(), rStringStream, rsMRAeparator, printHeader, rHeaderStringStream);
//	if (rObject.isValid()) {
//		addToCSV(rstrId+".x", rObject.getXYlocation().x, rStringStream, rsMRAeparator, printHeader, rHeaderStringStream);
//		addToCSV(rstrId+".y", rObject.getXYlocation().y, rStringStream, rsMRAeparator, printHeader, rHeaderStringStream);
//		addToCSV(rstrId+".vx", linVel.x, rStringStream, rsMRAeparator, printHeader, rHeaderStringStream);
//		addToCSV(rstrId+".vy", linVel.y, rStringStream, rsMRAeparator, printHeader, rHeaderStringStream);
//		addToCSV(rstrId+".vr", rotVel, rStringStream, rsMRAeparator, printHeader, rHeaderStringStream);
//	}
//	else
//	{
//		addToCSV(rstrId+".x", 0.0, rStringStream, rsMRAeparator, printHeader, rHeaderStringStream);
//		addToCSV(rstrId+".y", 0.0, rStringStream, rsMRAeparator, printHeader, rHeaderStringStream);
//		addToCSV(rstrId+".vx", 0.0, rStringStream, rsMRAeparator, printHeader, rHeaderStringStream);
//		addToCSV(rstrId+".vy", 0.0, rStringStream, rsMRAeparator, printHeader, rHeaderStringStream);
//		addToCSV(rstrId+".vr", 0.0, rStringStream, rsMRAeparator, printHeader, rHeaderStringStream);
//	}
//}



//string TeamPlannerData::toCSVlinestring(bool printHeader, bool inputOnly) {
//	const string separator = "; ";
//	std::stringstream buffer;
//	std::stringstream header_buffer;
//
//	if (printHeader) {
//		// start with comment line character
//		header_buffer << "# ";
//	}
//
//	if (inputOnly) {
//		buffer << std::setprecision(3);
//	}
//	addToCSV("gamestate", static_cast<double>(this->gamestate), buffer, separator, printHeader, header_buffer);
//
//	MovingObjectToCSV("global ball", this->globalBall, buffer, separator, printHeader, header_buffer);
//	MovingObjectToCSV("local ball", this->localBall, buffer, separator, printHeader, header_buffer);
//	for (unsigned idx = 0; idx < this->Team.size(); idx++){
//		TeamPlannerRobot* tpr = &(this->Team[idx]);
//		string player_string = "player" + to_string(idx+1);
//		MovingObjectToCSV(player_string, tpr->position,  buffer, separator, printHeader, header_buffer);
//
//		addToCSV(player_string+".robotId", static_cast<int>(tpr->robotId), buffer, separator, printHeader, header_buffer);
//		addToCSV(player_string+".controlBall", static_cast<bool>(tpr->controlBall), buffer, separator, printHeader, header_buffer);
//		addToCSV(player_string+".passBall", static_cast<bool>(tpr->passBall), buffer, separator, printHeader, header_buffer);
//		addToCSV(player_string+".player_type", static_cast<int>(tpr->player_type), buffer, separator, printHeader, header_buffer);
//
//		addToCSV(player_string+".previous_result.present", static_cast<double>(tpr->previous_result.previous_result_present), buffer, separator, printHeader, header_buffer);
//		if (tpr->previous_result.previous_result_present) {
//			// previous_result is NOT present
//			addToCSV(player_string+".previous_result.ts", static_cast<double>(tpr->previous_result.ts), buffer, separator, printHeader, header_buffer);
//			addToCSV(player_string+".previous_result.dynamic_role", static_cast<int>(tpr->previous_result.dynamic_role), buffer, separator, printHeader, header_buffer);
//			addToCSV(player_string+".previous_result.end_position.target", static_cast<int>(tpr->previous_result.end_position.target), buffer, separator, printHeader, header_buffer);
//			addToCSV(player_string+".previous_result.end_position.x", static_cast<double>(tpr->previous_result.end_position.x), buffer, separator, printHeader, header_buffer);
//			addToCSV(player_string+".previous_result.end_position.y", static_cast<double>(tpr->previous_result.end_position.y), buffer, separator, printHeader, header_buffer);
//		}
//		else {
//			// previous_result is NOT present
//			addToCSV(player_string+".previous_result.ts", 0.0, buffer, separator, printHeader, header_buffer);
//			addToCSV(player_string+".previous_result.dynamic_role", 0, buffer, separator, printHeader, header_buffer);
//			addToCSV(player_string+".previous_result.end_position.target", 0, buffer, separator, printHeader, header_buffer);
//			addToCSV(player_string+".previous_result.end_position.x", 0.0, buffer, separator, printHeader, header_buffer);
//			addToCSV(player_string+".previous_result.end_position.y", 0.0, buffer, separator, printHeader, header_buffer);
//		}
//	}
//	for (unsigned idx = 0; idx < this->Opponents.size(); idx++){
//		MovingObjectToCSV("opponent" + to_string(idx+1), this->Opponents[idx].position, buffer, separator, printHeader, header_buffer);
//	}
//
//	//	cerr << this->plannerOptions.toString() << endl;
//	//
//	// TODO PlannerOptions plannerOptions;
//
//	addToCSV("ball_pickup_position.valid", static_cast<bool>(this->ball_pickup_position.valid), buffer, separator, printHeader, header_buffer);
//	if (this->ball_pickup_position.valid) {
//		// ball_pickup_position is valid
//		addToCSV("ball_pickup_position.x", static_cast<double>(this->ball_pickup_position.x), buffer, separator, printHeader, header_buffer);
//		addToCSV("ball_pickup_position.y", static_cast<double>(this->ball_pickup_position.y), buffer, separator, printHeader, header_buffer);
//		addToCSV("ball_pickup_position.ts", static_cast<double>(this->ball_pickup_position.ts), buffer, separator, printHeader, header_buffer);
//	}
//	else {
//		// ball_pickup_position is invalid
//		addToCSV("ball_pickup_position.x", 0.0, buffer, separator, printHeader, header_buffer);
//		addToCSV("ball_pickup_position.y", 0.0, buffer, separator, printHeader, header_buffer);
//		addToCSV("ball_pickup_position.ts", 0.0, buffer, separator, printHeader, header_buffer);
//	}
//	addToCSV("passIsRequired", static_cast<bool>(this->passIsRequired), buffer, separator, printHeader, header_buffer);
//	addToCSV("pass_data.valid", static_cast<bool>(this->pass_data.valid), buffer, separator, printHeader, header_buffer);
//
//	if (this->pass_data.valid)
//	{
//		// pass data is valid
//		addToCSV("pass_data.kicked", static_cast<double>(this->pass_data.kicked), buffer, separator, printHeader, header_buffer);
//		addToCSV("pass_data.target_id", static_cast<double>(this->pass_data.target_id), buffer, separator, printHeader, header_buffer);
//		addToCSV("pass_data.origin_pos.x", static_cast<double>(this->pass_data.origin_pos.x), buffer, separator, printHeader, header_buffer);
//		addToCSV("pass_data.origin_pos.y", static_cast<double>(this->pass_data.origin_pos.y), buffer, separator, printHeader, header_buffer);
//		addToCSV("pass_data.target_pos.x", static_cast<double>(this->pass_data.target_pos.x), buffer, separator, printHeader, header_buffer);
//		addToCSV("pass_data.target_pos.y", static_cast<double>(this->pass_data.target_pos.y), buffer, separator, printHeader, header_buffer);
//		//addToCSV("pass_data.velocity", static_cast<double>(this->pass_data.velocity), buffer, separator, printHeader, header_buffer);
//		//addToCSV("pass_data.rz", fmod(this->pass_data.rz, M_PI*2), buffer, separator, printHeader, header_buffer);
//		addToCSV("pass_data.ts", static_cast<double>(this->pass_data.ts), buffer, separator, printHeader, header_buffer);
//		addToCSV("pass_data.eta", static_cast<double>(this->pass_data.eta), buffer, separator, printHeader, header_buffer);
//	}
//	else
//	{
//		// pass data is invalid
//		addToCSV("pass_data.kicked", 0, buffer, separator, printHeader, header_buffer);
//		addToCSV("pass_data.target_id", 0, buffer, separator, printHeader, header_buffer);
//		addToCSV("pass_data.origin_pos.x", 0.0, buffer, separator, printHeader, header_buffer);
//		addToCSV("pass_data.origin_pos.y", 0.0, buffer, separator, printHeader, header_buffer);
//		addToCSV("pass_data.target_pos.x", 0.0, buffer, separator, printHeader, header_buffer);
//		addToCSV("pass_data.target_pos.y", 0.0, buffer, separator, printHeader, header_buffer);
//		//addToCSV("pass_data.velocity", 0.0, buffer, separator, printHeader, header_buffer);
//		//addToCSV("pass_data.rz", 0.0, buffer, separator, printHeader, header_buffer);
//		addToCSV("pass_data.ts", 0.0, buffer, separator, printHeader, header_buffer);
//		addToCSV("pass_data.eta", 0.0, buffer, separator, printHeader, header_buffer);
//	}
//
//	if (!inputOnly) {
//		for (unsigned int player_idx = 0; player_idx < player_paths->size(); player_idx++) {
//			PlayerPlannerResult playerResult = (*player_paths)[player_idx];
//			double x = 0.0;
//			double y = 0.0;
//			if (playerResult.path.size() > 0) {
//				x = playerResult.path[playerResult.path.size()-1].x;
//				y = playerResult.path[playerResult.path.size()-1].y;
//			}
//			string result_string = "result" + to_string(player_idx+1);
//			bool path_present = playerResult.path.size() > 0;
//			addToCSV(result_string + ".present", path_present, buffer, separator, printHeader, header_buffer);
//			if (path_present){
//				addToCSV(result_string + ".x", x, buffer, separator, printHeader, header_buffer);
//				addToCSV(result_string + ".y", y, buffer, separator, printHeader, header_buffer);
//				addToCSV(result_string + ".dynamic_role", playerResult.dynamic_role, buffer, separator, printHeader, header_buffer);
//				addToCSV(result_string + ".planner_target", playerResult.planner_target, buffer, separator, printHeader, header_buffer);
//			}
//			else {
//				// no path present
//				addToCSV(result_string + ".x", 0.0, buffer, separator, printHeader, header_buffer);
//				addToCSV(result_string + ".y", 0.0, buffer, separator, printHeader, header_buffer);
//				addToCSV(result_string + ".dynamic_role", 0, buffer, separator, printHeader, header_buffer);
//				addToCSV(result_string + ".planner_target", 0, buffer, separator, printHeader, header_buffer);
//			}
//		}
//	}
//
//
//	/*
//// NO parking, parking not via learning!	std::vector<Vector2D> parking_positions + gamecmd PARKING;
//*/
//	string result_str = "";
//	if (printHeader) {
//		result_str += header_buffer.str() + "\n";
//	}
//	result_str += buffer.str();
//	return result_str;
//}
//
//void TeamPlannerData::fromCSVstring(std::string&)
//{
//};
//
//void TeamPlannerData::fillData(game_state_e gamestate, const MovingObject& globalBall, const MovingObject& localBall,
//		const std::vector<MovingObject>& myTeam, const std::vector<MovingObject>& opponents,
//		long controlBallByPlayerId, const std::vector<player_type_e>& teamTypes, const std::vector<long>& robotIds,
//		const PlannerOptions& plannerOptions,
//		const std::vector<MRA::Geometry::Point>& parking_postions, const FieldConfig& fieldConfig,
//		const previous_used_ball_by_planner_t& previous_global_ball,
//		const std::vector<final_planner_result_t>& previous_planner_results,
//		const ball_pickup_position_t& ball_pickup_position, bool passIsRequired,
//		long passBallByPlayerId, const pass_data_t& pass_data,
//		const std::vector<double>& time_in_own_penalty_area, const std::vector<double>& time_in_opponent_penalty_area)
//{
//
//	this->previous_global_ball = previous_global_ball;
//	// calculate assignment for the given situation
//	vector<TeamPlannerRobot> Team = vector<TeamPlannerRobot>();
//	for (unsigned idx = 0; idx < myTeam.size(); idx++) {
//		TeamPlannerRobot robot;
//		robot.robotId = robotIds[idx];
//		robot.position = myTeam[idx];
//		if (controlBallByPlayerId >= 0 && robotIds[idx] == static_cast<unsigned>(controlBallByPlayerId)) {
//			robot.controlBall = true;
//		}
//		else {
//			robot.controlBall = false;
//		}
//		robot.passBall = robot.robotId == static_cast<unsigned>(passBallByPlayerId);
//		robot.player_type = teamTypes[idx];
//		robot.dynamic_role = dynamic_role_e::dr_NONE;
//		robot.assigned = false;
//		robot.result = PlayerPlannerResult();
//  		robot.previous_result = previous_planner_results[idx];
//		robot.time_in_own_penalty_area = time_in_own_penalty_area[idx];
//  		robot.time_in_opponent_penalty_area = time_in_opponent_penalty_area[idx];
//		Team.push_back(robot);
//	}
//	std::vector<TeamPlannerOpponent> Opponents = std::vector<TeamPlannerOpponent>();
//	for (unsigned idx = 0; idx < opponents.size(); idx++) {
//		TeamPlannerOpponent opponent = TeamPlannerOpponent();
//		opponent.position = opponents[idx];
//		opponent.assigned = false;
//		Opponents.push_back(opponent);
//	}
//
//
//	this->player_paths = new team_planner_result_t(Team.size(), PlayerPlannerResult());
//	this->gamestate = gamestate;
//	this->globalBall = globalBall;
//	this->localBall = localBall;
//	this->Team = Team;
//	this->Opponents = Opponents;
//	this->plannerOptions = plannerOptions;
//	this->parking_positions = parking_postions;
//	this->ball_pickup_position = ball_pickup_position;
//	this->passIsRequired = passIsRequired;
//	this->pass_data = pass_data;
//};

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

