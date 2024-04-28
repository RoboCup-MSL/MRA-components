/**
 *  @file
 *  @brief   Utility to use xml inputfile to test the robot planner
 *  @curator Jürge van Eijck
 */
#include <ostream>
#include <iostream>
#include <istream>
#include <vector>
#include <string>
#include <cstdio>
#include <chrono>

#include "GlobalPathPlannerTester_generated.h" // generated
#include "MovingObject.h"
#include "GlobalPathPlanner.h"
#include "Field.h"
#include "SvgUtils.hpp"

using namespace MRA;
using namespace std;
using namespace robotsports;

void xmlGlobalPathPlanner(char * input_filename) {

	PlannerOptions plannerOptions = PlannerOptions();

	std::vector<trs::MovingObject> teammates = std::vector<trs::MovingObject>();
	trs::MovingObject ball = trs::MovingObject();
	trs::MovingObject me = trs::MovingObject();
	std::vector<trs::MovingObject> opponents = std::vector<trs::MovingObject>();
	planner_target_e plannerObjective;
	std::string description = "";
	std::vector<trs::Vertex> targets = std::vector<trs::Vertex>();;
	string filename = input_filename;

	cout << "reading file : " << filename << endl << flush;

	try
	{
		auto_ptr<robotsports::GlobalPathPlannerType> c(robotsports::Situation (filename));
		description = c->Description();
		ball.set(*(c->Ball().x()), *(c->Ball().y()), 0.0, (c->Ball().velx()), (c->Ball().vely()), 0.0, -1, true);

		me.set(*(c->Me().x()), *(c->Me().y()), 0.0, (c->Me().velx()), (c->Me().vely()), 0.0, -1, true);

		int label = 1;
		for (GlobalPathPlannerType::Team_const_iterator team_iter = c->Team().begin(); team_iter != c->Team().end(); ++team_iter) {
			teammates.push_back(trs::MovingObject(*(*team_iter).x(), *(*team_iter).y(), (*team_iter).rz(),
					(*team_iter).velx(), (*team_iter).vely(), (*team_iter).velrz(), label, true));
			label ++;
		}

		label = 11;
		for (GlobalPathPlannerType::Opponent_const_iterator opponent_iter = c->Opponent().begin(); opponent_iter != c->Opponent().end(); ++opponent_iter) {
			opponents.push_back(trs::MovingObject(*(*opponent_iter).x(), *(*opponent_iter).y(), (*opponent_iter).rz(),
					(*opponent_iter).velx(), (*opponent_iter).vely(), (*opponent_iter).velrz(), label, true));
			label ++;
		}

		for (GlobalPathPlannerType::Target_const_iterator target_iter = c->Target().begin(); target_iter != c->Target().end(); ++target_iter) {
			targets.push_back(trs::Vertex(Vector2D(*(*target_iter).x(), *(*target_iter).y()), (*target_iter).distToTarget(), (*target_iter).cost()));
		}



		string gs = c->GlobalPathPlannerObjective();
		if (gs == "GOTO_BALL") {
			plannerObjective = planner_target_e::GOTO_BALL;
		}
		else if (gs == "DRIBBLE") {
			plannerObjective = planner_target_e::DRIBBLE;
		}
		else if (gs == "SUPPORT_DEFENSE") {
			plannerObjective = planner_target_e::SUPPORT_DEFENSE;
		}
		else if (gs == "SUPPORT_ATTACK") {
			plannerObjective = planner_target_e::SUPPORT_ATTACK;
		}
		else if (gs == "PREPARE_RESTART_AGAINST") {
			plannerObjective = planner_target_e::PREPARE_RESTART_AGAINST;
		}
		else if (gs == "PREPARE_RESTART") {
			plannerObjective = planner_target_e::PREPARE_RESTART;
		}
		else if (gs == "PREPARE_DROPBALL") {
			plannerObjective = planner_target_e::PREPARE_DROPBALL;
		}
		else if (gs == "GOTO_TARGET_POSITION") {
			plannerObjective = planner_target_e::GOTO_TARGET_POSITION;
		}
		else if (gs == "GOTO_TARGET_POSITION_SLOW") {
			plannerObjective = planner_target_e::GOTO_TARGET_POSITION_SLOW;
		}
		else if (gs == "SWEEPER") {
			plannerObjective = planner_target_e::SWEEPER;
		}
		else if (gs == "GOALIE") {
			plannerObjective = planner_target_e::GOALIE;
		}
		else {
			cerr  << "Unknown game state in xml file: " << gs << endl;
			exit(-1);
		}

		// Copy all input
		plannerOptions.minimumEdgeLength = c->Options().minimumEdgeLength();
		plannerOptions.maximumEdgeLength = c->Options().maximumEdgeLength();
		plannerOptions.minimumDistanceToEndPoint = c->Options().minimumDistanceToEndPoint();
		plannerOptions.nrVerticesFirstCircle = c->Options().nrVerticesFirstCircle();
		plannerOptions.firstCircleRadius = c->Options().firstCircleRadius();
		plannerOptions.nrVerticesSecondCircle = c->Options().nrVerticesSecondCircle();
		plannerOptions.secondCircleRadius = c->Options().secondCircleRadius();
		plannerOptions.safetyFactor = c->Options().safetyFactor();
		plannerOptions.addBarierVertices = c->Options().addBarierVertices();
		plannerOptions.addUniformVertices = c->Options().addUniformVertices();
		plannerOptions.uniform_x_interval = c->Options().uniform_x_interval();
		plannerOptions.uniform_y_interval = c->Options().uniform_y_interval();
		plannerOptions.startingVelocityPenaltyFactor = c->Options().startingVelocityPenaltyFactor();
		plannerOptions.addBallApproachVertices = c->Options().addBallApproachVertices();
		plannerOptions.distToapplyBallApproachVertices = c->Options().distToapplyBallApproachVertices();
		plannerOptions.ballApproachVerticesRadius = c->Options().ballApproachVerticesRadius();
		plannerOptions.ballApproachNumberOfVertices = c->Options().ballApproachNumberOfVertices();
		plannerOptions.manDefenseBetweenBallAndPlayer = c->Options().manDefenseBetweenBallAndPlayer();
		plannerOptions.dist_before_penalty_area_for_sweeper = c->Options().dist_before_penalty_area_for_sweeper();
		plannerOptions.svgDrawVelocity = c->Options().svgDrawVelocity();
		plannerOptions.svgDrawEdges = c->Options().svgDrawEdges();
		plannerOptions.svgDefaultTargetColor = c->Options().svgDefaultTargetColor();
		plannerOptions.svgBallColor = c->Options().svgBallColor();
		plannerOptions.svgOriginalTargetColor = c->Options().svgOriginalTargetColor();
		plannerOptions.svgTeamColor = c->Options().svgTeamColor();
		plannerOptions.svgOpponentColor = c->Options().svgOpponentColor();
		plannerOptions.svgOutputFileName = c->Options().svgOutputFileName();
	}
	catch (const xml_schema::exception& e)
	{
	  cerr << e << endl;
	  return;
	}
	catch (const xml_schema::properties::argument&)
	{
	  cerr << "invalid property argument (empty namespace or location)" << endl;
	  return;
	}
	catch (const xsd::cxx::xml::invalid_utf16_string&)
	{
	  cerr << "invalid UTF-16 text in DOM model" << endl;
	  return;
	}
	catch (const xsd::cxx::xml::invalid_utf8_string&)
	{
	  cerr << "invalid UTF-8 text in object model" << endl;
	  return;
	}
	cerr << "start robot planner" << endl;
	cout << "running : " << endl << description << endl;

	auto start = std::chrono::system_clock::now();
	std::vector<std::vector<planner_piece_t>> player_paths = std::vector<vector<planner_piece_t>>();

	FieldConfig fieldConfig(Field::defaultFieldConfig());
	GlobalPathPlanner globalPathPlanner = GlobalPathPlanner(fieldConfig);
	globalPathPlanner.setOptions(plannerOptions);
	bool ballIsObstacle = false;
	bool avoidBallPath = false; // Set to false, this will not be tested here
	Vector2D BallTargePos;
	globalPathPlanner.createGraph(me, ball, teammates, opponents, targets, plannerObjective, ballIsObstacle, avoidBallPath, BallTargePos);
	std::vector<planner_piece_t> robotPath = globalPathPlanner.getShortestPath();
	for (unsigned idx = 0; idx < robotPath.size(); idx++) {
		planner_piece_t p = robotPath[idx];
		cerr << "[" << idx << "] on (" << p.x << ", " << p.y << ") = " << p.cost << " t: " << p.target << endl << flush;
	}

	player_paths.push_back(robotPath);
	teammates.insert(teammates.begin(), me); // insert this robot as first in the vector


	globalPathPlanner.save_graph_as_svg(player_paths[0]);
	//SvgUtils::save_graph_as_svg(ball, teammates, opponents, player_paths, plannerOptions, std::vector<Vertex*>());

	auto finish = std::chrono::system_clock::now();
	double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double> >(finish - start).count();
	// test start position assignments
	std::cout << "elapsed time: " << elapsed_seconds*1000 << " [ms]" << std::endl;

}

int main(int argc, char *argv[]) {
	if (argc < 2) {
		cerr << "use program: " << argv[0] << " <filename.xml> " << endl;
		return -1;
	}

	xmlGlobalPathPlanner(argv[1]);
}
