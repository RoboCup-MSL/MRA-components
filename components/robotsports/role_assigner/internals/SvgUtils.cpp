/**
 *  @file
 *  @brief   Utility class for plotting planner to svg file
 *  @curator Jürge van Eijck
 */
#include "SvgUtils.hpp"

#include "FieldConfig.h"
#include "MathUtils.h"
#include "FileUtils.h"
//#include "TeamPlannerRobot.hpp"
#include <iostream>
#include <ostream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <cstdio>
#include <cstdlib>
#include <list>
#include <limits>
#include <map>
#include <cmath>
#include "logging.hpp"

using namespace std;
using namespace trs;

FieldConfig SvgUtils::m_fieldConfig = FillDefaultFieldConfig();

/**
 * Get svg x coordinate for given field X
 */
double SvgUtils::svgX(double fieldX) {
	return m_fieldConfig.getMaxFullFieldX() + fieldX;
}

/**
 * Get svg x coordinate for given field Y
 */
double SvgUtils::svgY(double fieldY) {
	return m_fieldConfig.getMaxFullFieldY() - fieldY;
}

void SvgUtils::save_graph_as_svg(const MovingObject& globalBall, const MovingObject& localBall,
		const std::vector<MovingObject>& myTeam, const std::vector<MovingObject>& opponents,
		const team_planner_result_t& player_paths, const PlannerOptions& options, const std::vector<Vertex* >& vertices,
		game_state_e gamestate, long controlBallByPlayer, const std::vector<player_type_e>& teamTypes, const std::vector<long>& robotIds,
		const std::string& colorMe, const FieldConfig& fieldConfig, bool hasTeamPlannerInputInfo, const TeamPlannerInputInfo&  inputInfo) {
	team_planner_result_t compare_paths = team_planner_result_t();
	save_graph_as_svg(globalBall, localBall, myTeam, opponents,	 player_paths, compare_paths,
			options, vertices, gamestate, controlBallByPlayer, teamTypes, robotIds, colorMe, fieldConfig,
			hasTeamPlannerInputInfo, inputInfo );
}


void SvgUtils::save_graph_as_svg(const MovingObject& globalBall, const MovingObject& localBall,
		const std::vector<MovingObject>& myTeam, const std::vector<MovingObject>& opponents,
		const team_planner_result_t& player_paths,
		const team_planner_result_t&  comparing_player_paths,
		const PlannerOptions& options, const std::vector<Vertex* >& vertices,
		game_state_e gamestate, long controlBallByPlayer, const std::vector<player_type_e>& teamTypes,
		const std::vector<long>& robotIds, const std::string& colorMe, const FieldConfig& fieldConfig,
		bool hasTeamPlannerInputInfo, const TeamPlannerInputInfo&  inputInfo )
{
	m_fieldConfig = fieldConfig;

	if (options.svgOutputFileName.empty()) {
		return;  // No outputfile required
	}

	double totalFieldLength = fieldConfig.getFullFieldLength();
	double totalFieldWidth = fieldConfig.getFullFieldWidth();
	double halfRobotSize = fieldConfig.getRobotRadius();
	double robotSize = fieldConfig.getRobotSize();

	FileParts parts = FileUtils::fileparts(options.svgOutputFileName);
	if (parts.path.size() > 0) {
		if (!FileUtils::isDirectory(parts.path)) {
			MRA_LOG_INFO("Not existing directory: \"%s\"", parts.path.c_str());
			return;
		}
	}

	FILE* fp = fopen(options.svgOutputFileName.c_str(), "w");
	// SVG header
	fprintf(fp, "<?xml version=\"1.0\" standalone=\"yes\"?>\n");
	fprintf(fp,
			"<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n");
	fprintf(fp,
			"<svg width=\"%4.2fcm\" height=\"%4.2fcm\" version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\">\n",
			totalFieldWidth, totalFieldLength);
	fprintf(fp, "<desc>MSL field path</desc>\n");

	fprintf(fp, "<!-- \n\nfile: %s\n", options.svgOutputFileName.c_str()); // start svg comment


	std::stringstream Xtext;


	// print input data to svg file
	fprintf(fp, "\n\n");
	fprintf(fp, "\tgamestate = %s (%d)\n", GameStateAsString(gamestate).c_str(), gamestate);
	string controlBallByPlayerRemark = "";
	if (controlBallByPlayer == -1) {
		controlBallByPlayerRemark = "(not controlled by team)";
	}
	else if (controlBallByPlayer == 0) {
		controlBallByPlayerRemark = "(controlled by me)";
	}
	else {
		controlBallByPlayerRemark = "(controlled by team-member)";
	}
	fprintf(fp, "\tcontrolball = %s (%ld)\n", controlBallByPlayerRemark.c_str(), controlBallByPlayer);
	fprintf(fp, "\tglobalBall: %s\n", globalBall.toString().c_str());
	fprintf(fp, "\tlocalBall: %s\n", localBall.toString().c_str());
	fprintf(fp, "\tteam:\n");
	for (unsigned int idx = 0; idx < myTeam.size(); idx++) {
		int teamtypeId = -1;
		if (idx < teamTypes.size()) {
			teamtypeId = teamTypes[idx];
		}
		long robotId = idx;
		if (robotIds.size() > idx) {
			robotId = robotIds [idx];
		}
		fprintf(fp, "\t\tR%02ld = %s type = %s (%d)\n",
				robotId, myTeam[idx].toString().c_str(), PlayerTypeAsString(static_cast<player_type_e>(teamtypeId)).c_str(), teamtypeId );
	}
	fprintf(fp, "\topponents:\n");
	for (unsigned int idx = 0; idx < opponents.size(); idx++) {
		fprintf(fp, "\t\tplayer[%u] = %s\n", idx, opponents[idx].toString().c_str());
	}

	for (unsigned long p_idx = 0; p_idx < player_paths.size(); p_idx++) {
		Xtext << std::fixed << std::setprecision(2) << endl<< "Player " << p_idx << ":" << std::endl;
			Xtext << "\tDynamic-role: " << DynamicRoleAsString(player_paths[p_idx].dynamic_role) << endl;
			Xtext << "\tGame State: " << GameStateAsString(player_paths[p_idx].gamestate) << endl;
			Xtext << "\tTarget position : " << player_paths[p_idx].target.toString() << endl;
			Xtext << "\tPlanner target type  " << PlannerTargetAsString(player_paths[p_idx].planner_target) << endl;
			Xtext << "\tDefend info valid:  " << player_paths[p_idx].defend_info.valid ;
			Xtext << " id: " << player_paths[p_idx].defend_info.defending_id;
			Xtext << " ball-rbt: " << player_paths[p_idx].defend_info.between_ball_and_defending_pos;
			Xtext << " dist: " << player_paths[p_idx].defend_info.dist_from_defending_id << endl;
		for (unsigned long idx = 0; idx < player_paths[p_idx].path.size(); idx++) {
			Xtext << std::fixed << setprecision(2) << "Path[" << p_idx << "]: x: " << player_paths[p_idx].path[idx].x <<
					" y: " << player_paths[p_idx].path[idx].y <<
					" cost: " << player_paths[p_idx].path[idx].cost;
			std::string targetString = PlannerTargetAsString(static_cast<planner_target_e>(player_paths[p_idx].path[idx].target));
			Xtext << " target: " << targetString << " (" << player_paths[p_idx].path[idx].target<< ")"<<  std::endl;
		}
	}
	fprintf(fp, "paths:\n%s\n", Xtext.str().c_str());

	fprintf(fp, "\toptions:\n%s\n", options.toString().c_str());
	fprintf(fp, "\tfield:\n%s\n", fieldConfig.toString().c_str());
	fprintf(fp, "\n");

	if (hasTeamPlannerInputInfo)
	{
		fprintf(fp, "\tpass_is_required: %s\n", inputInfo.passIsRequired ? "true" : "false");
		fprintf(fp, "\tpassing player: %d\n", inputInfo.playerWhoIsPassing);
		fprintf(fp, "\tpickup ball valid: %s\n", inputInfo.ball_pickup_position.valid ? "true" : "false");
		fprintf(fp, "\tpickup ball x: %4.2f\n", inputInfo.ball_pickup_position.x);
		fprintf(fp, "\tpickup ball y: %4.2f\n", inputInfo.ball_pickup_position.y);
		fprintf(fp, "\tpickup ball ts: %4.2f\n", inputInfo.ball_pickup_position.ts);
		for (unsigned int idx = 0; idx < inputInfo.previous_results.size(); idx++) {
			fprintf(fp, "\tprev result[%u]: valid: %s\n", idx, inputInfo.previous_results[idx].previous_result_present ? "true" : "false");
			fprintf(fp, "\tprev result[%u]: role: %s\n", idx, DynamicRoleAsString((dynamic_role_e)inputInfo.previous_results[idx].dynamic_role).c_str());
			fprintf(fp, "\tprev result[%u]: end pos x: %4.2f\n", idx, inputInfo.previous_results[idx].end_position.x);
			fprintf(fp, "\tprev result[%u]: end pos y: %4.2f\n", idx, inputInfo.previous_results[idx].end_position.y);
			fprintf(fp, "\tprev result[%u]: end pos target: %ld\n", idx, inputInfo.previous_results[idx].end_position.target);
			fprintf(fp, "\tprev result[%u]: ts: %4.2f\n", idx, inputInfo.previous_results[idx].ts);
		}
		fprintf(fp, "\n");
	}

	// add xml output
	fprintf(fp, "  <tns:GameState>%s</tns:GameState>\n", GameStateAsString(gamestate).c_str());
	fprintf(fp, "  <tns:AttackFormation>%s</tns:AttackFormation>\n", FormationAsString(options.attack_formation).c_str());
	fprintf(fp, "  <tns:DefenseFormation>%s</tns:DefenseFormation>\n", FormationAsString(options.defense_formation).c_str());
	if (globalBall.getPosition().getConfidence() > 0.001) {
		MRA::Geometry::Point xyVel;
		double rzvel;
		globalBall.getVelocity(xyVel, rzvel);
		fprintf(fp, "  <tns:Ball x=\"%4.2f\" y=\"%4.2f\" velx=\"%4.2f\" vely=\"%4.2f\"/>\n",
				globalBall.getPosition().getPoint().x, globalBall.getPosition().getPoint().y, xyVel.x, xyVel.y);
	}
	if (localBall.getPosition().getConfidence() > 0.001) {
	    MRA::Geometry::Point xyVel;
		double rzvel;
		localBall.getVelocity(xyVel, rzvel);
		fprintf(fp, "  <tns:LocalBall x=\"%4.2f\" y=\"%4.2f\" velx=\"%4.2f\" vely=\"%4.2f\"/>\n",
				localBall.getPosition().getPoint().x, localBall.getPosition().getPoint().y, xyVel.x, xyVel.y);
	}
	for (unsigned int idx = 0; idx < myTeam.size(); idx++) {
		string goalieString = "";
		if (idx < teamTypes.size()) {
			if (teamTypes[idx] == GOALIE) {
				goalieString = " isGoalie=\"true\" ";
			}
		}

		string idString = "id=\""+ std::to_string(myTeam[idx].getLabel()) + "\"";

		string controlBallString = "";
		if (controlBallByPlayer  == static_cast<int>(idx))
		{
			controlBallString = " hasBall=\"true\" ";
		}
		string passedBallString = "";
		string previous_result_string = "";
		if (hasTeamPlannerInputInfo)
		{
			if (inputInfo.playerWhoIsPassing == static_cast<int>(idx))
			{
				passedBallString = "passedBall=\"true\"";
			}
			if (inputInfo.previous_results[idx].previous_result_present)
			{
				std::stringstream previous_result_Xtext;
				previous_result_Xtext << " previous_result_present=\"true\" "
						               <<" previous_result_ts=\""  << inputInfo.previous_results[idx].ts << "\""
									   <<" previous_result_x=\""   << inputInfo.previous_results[idx].end_position.x << "\""
									   <<" previous_result_y=\""   << inputInfo.previous_results[idx].end_position.y << "\""
									   <<" previous_result_dynamic_role=\""
									   << DynamicRoleAsString(static_cast<dynamic_role_e>(inputInfo.previous_results[idx].dynamic_role))
									   <<"\"";

				previous_result_string= previous_result_Xtext.str();;

			}
		}
		MRA::Geometry::Point xyVel;
		double rzvel;
		myTeam[idx].getVelocity(xyVel, rzvel);
		fprintf(fp, "  <tns:Team %s x=\"%4.3f\" y=\"%4.3f\" rz=\"%4.3f\" velx=\"%4.3f\" vely=\"%4.3f\" velrz=\"%4.3f\" %s %s %s %s/>\n",
				idString.c_str(),
				myTeam[idx].getPosition().getPoint().x, myTeam[idx].getPosition().getPoint().y, myTeam[idx].getPosition().getRotationZ(),
				xyVel.x, xyVel.y, rzvel, goalieString.c_str(), controlBallString.c_str(), passedBallString.c_str(), previous_result_string.c_str());

	}
	for (unsigned int idx = 0; idx < opponents.size(); idx++) {
	    MRA::Geometry::Point xyVel;
		double rzvel;
		opponents[idx].getVelocity(xyVel, rzvel);
		string idString = "id=\""+ std::to_string(opponents[idx].getLabel()) + "\"";
		fprintf(fp, "  <tns:Opponent %s x=\"%4.3f\" y=\"%4.3f\" rz=\"%4.3f\" velx=\"%4.3f\" vely=\"%4.3f\" velrz=\"%4.3f\" />\n",
				idString.c_str(),
				opponents[idx].getPosition().getPoint().x, opponents[idx].getPosition().getPoint().y, opponents[idx].getPosition().getRotationZ(),
				xyVel.x, xyVel.y, rzvel);

	}
	if (hasTeamPlannerInputInfo)
	{
		// write pickup ball info as svg input if possible
		string validString = "valid=\"false\"";
		if (inputInfo.ball_pickup_position.valid) {
			validString = "valid=\"true\"";
		}

		fprintf(fp, "  <tns:PickupPosition x=\"%4.3f\" y=\"%4.3f\" ts=\"%4.3f\" %s />\n",
				inputInfo.ball_pickup_position.x, inputInfo.ball_pickup_position.y, inputInfo.ball_pickup_position.ts, validString.c_str());


		// write situation info as svg input if possible
		string passRequiredString = "passing_required=\"false\"";
		if (inputInfo.passIsRequired)
		{
			passRequiredString = "passing_required=\"true\"";
		}

		fprintf(fp, "  <tns:SituationInfo %s>\n", passRequiredString.c_str());
		string pass_data_valid_str = "valid=\"false\"";
		if (inputInfo.pass_data.valid)
		{
			pass_data_valid_str = "valid=\"true\"";
		}
		fprintf(fp, "    <tns:PassData %s origin_x=\"%4.3f\" origin_y=\"%4.3f\" target_x=\"%4.3f\" target_y=\"%4.3f\" velocity=\"%4.3f\" angle=\"%4.3f\" ts=\"%4.3f\"/>\n",
				pass_data_valid_str.c_str(), inputInfo.pass_data.origin_pos.x, inputInfo.pass_data.origin_pos.y, inputInfo.pass_data.target_pos.x, inputInfo.pass_data.target_pos.y,
					inputInfo.pass_data.velocity, inputInfo.pass_data.angle, inputInfo.pass_data.ts);
		fprintf(fp, "  </tns:SituationInfo>\n");
		fprintf(fp, "</tns:Situation>\n");
	}

	fprintf(fp, "\n-->\n"); // // end svg comment


	//FIELD - total green field
	fprintf(fp,
			"<rect x=\"0cm\" y=\"0cm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"green\" stroke-width=\"0.02cm\" />\n",
			totalFieldWidth, totalFieldLength);
	//FIELD - outer field lines
	fprintf(fp,
			"<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"none\" stroke=\"white\" stroke-width=\"0.125cm\"/>",
			fieldConfig.getFieldMargin(), fieldConfig.getFieldMargin(), fieldConfig.getFieldWidth(), fieldConfig.getFieldLength());
	//FIELD - middle line
	fprintf(fp,
			"<line x1=\"%4.2fcm\" y1=\"%4.2fcm\" x2=\"%4.2fcm\" y2=\"%4.2fcm\" stroke-width=\"0.125cm\"  stroke=\"white\"/>\n",
	         fieldConfig.getFieldMargin(), fieldConfig.getMaxFieldY()+fieldConfig.getFieldMargin(), fieldConfig.getFieldWidth()+fieldConfig.getFieldMargin(), fieldConfig.getMaxFieldY()+fieldConfig.getFieldMargin());
	//FIELD - middle circle
	fprintf(fp,
			"<circle cx=\"%4.2fcm\" cy=\"%4.2fcm\" r=\"%4.2fcm\" fill=\"none\" stroke=\"white\" stroke-width=\"0.125cm\"  />\n",
			totalFieldWidth*0.5,  totalFieldLength*0.5, fieldConfig.getCenterCirleDiameter()*0.5);

	// PENALTY AREAS
	fprintf(fp,
			"<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"green\" stroke=\"white\" stroke-width=\"0.125cm\"/>\n",
			(totalFieldWidth*0.5)-(fieldConfig.getPenaltyAreaWidth()*0.5), fieldConfig.getFieldMargin(), fieldConfig.getPenaltyAreaWidth(), fieldConfig.getPenaltyAreaLength());

	fprintf(fp,
			"<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"green\" stroke=\"white\" stroke-width=\"0.125cm\"/>\n",
			(totalFieldWidth*0.5)-(fieldConfig.getPenaltyAreaWidth()*0.5), fieldConfig.getFieldMargin()+fieldConfig.getFieldLength()-fieldConfig.getPenaltyAreaLength(), fieldConfig.getPenaltyAreaWidth(), fieldConfig.getPenaltyAreaLength());

	// GOAL AREAS
	fprintf(fp,
			"<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"green\" stroke=\"white\" stroke-width=\"0.125cm\"/>\n",
			(totalFieldWidth*0.5)-(fieldConfig.getGoalAreaWidth()*0.5), fieldConfig.getFieldMargin(), fieldConfig.getGoalAreaWidth(), fieldConfig.getGoalAreaLength());

	fprintf(fp,
			"<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"green\" stroke=\"white\" stroke-width=\"0.125cm\"/>\n",
			(totalFieldWidth*0.5)-(fieldConfig.getGoalAreaWidth()*0.5), fieldConfig.getFieldMargin()+fieldConfig.getFieldLength()-fieldConfig.getGoalAreaLength(), fieldConfig.getGoalAreaWidth(), fieldConfig.getGoalAreaLength());

	//FIELD - goals
	fprintf(fp,
			"<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"plum\" stroke=\"white\" stroke-width=\"0.125cm\"/>\n",
			(totalFieldWidth*0.5)-(fieldConfig.getGoalWidth()*0.5), fieldConfig.getFieldMargin()-fieldConfig.getGoalLength(), fieldConfig.getGoalWidth(), fieldConfig.getGoalLength());
	fprintf(fp,
			"<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\"  fill=\"powderblue\" stroke=\"white\" stroke-width=\"0.125cm\"/>\n",
			(totalFieldWidth*0.5)-(fieldConfig.getGoalWidth()*0.5), fieldConfig.getFieldMargin()+fieldConfig.getFieldLength(), fieldConfig.getGoalWidth(), fieldConfig.getGoalLength());
	// indicate own goal with text own goal
	fprintf(fp,"<text x=\"%4.2fcm\" y=\"%4.2fcm\" fill=\"darkred\">OWN</text>",
			(totalFieldWidth*0.5)-(fieldConfig.getGoalWidth()*0.2),
			fieldConfig.getFieldMargin()+fieldConfig.getFieldLength()+(fieldConfig.getGoalLength()*0.75));

	//vertices
	for (std::vector<Vertex* >::size_type j = 0; j < vertices.size(); j++) {
		Vertex* v = vertices[j];

		fprintf(fp,
				"<circle cx=\"%4.2fcm\" cy=\"%4.2fcm\" r=\"%4.2fcm\" fill=\"orange\" stroke=\"orange\" stroke-width=\"0.125cm\"  />\n",
				svgX(v->m_coordinate.x),  svgY(v->m_coordinate.y), 0.01);
		if (options.svgDrawEdges) {
			for (std::vector<Edge>::iterator it = v->m_neighbours.begin(); it != v->m_neighbours.end(); ++it) {
				Edge e = *it;
				Vertex* t = e.m_pTarget;
				fprintf(fp,
						"<line x1=\"%4.2fcm\" y1=\"%4.2fcm\" x2=\"%4.2fcm\" y2=\"%4.2fcm\" stroke-width=\"0.025cm\"  stroke=\"pink\"/>\n",
						svgX(v->m_coordinate.x),  svgY(v->m_coordinate.y), svgX(t->m_coordinate.x),  svgY(t->m_coordinate.y));

			}
		}
	}

	// OPPONENTS
	for(std::vector<MRA::Geometry::Point>::size_type bar_idx = 0; bar_idx != opponents.size(); bar_idx++) {
	    MRA::Geometry::Point bar_pos = opponents[bar_idx].getPosition().getPoint();
		fprintf(fp,
				"\n<!-- Opponent -->\n<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"%s\" stroke=\"%s\" stroke-width=\"0.125cm\"/>\n",
				svgX(bar_pos.x - halfRobotSize), svgY(bar_pos.y + halfRobotSize), robotSize, robotSize, options.svgOpponentColor.c_str(), options.svgOpponentColor.c_str());
		fprintf(fp,"<text x=\"%4.2fcm\" y=\"%4.2fcm\" font-size=\"large\" font-weight-absolute=\"bold\" fill=\"black\">%lu</text>",
				svgX(bar_pos.x- 0.65*halfRobotSize), svgY(bar_pos.y- 0.65*halfRobotSize), bar_idx+1);
	}

	// TEAMMATES
	// draw me first.
	if (myTeam.size() > 0 ) {
	    MRA::Geometry::Point bar_pos = myTeam[0].getPosition().getPoint();
		double r = myTeam[0].getPosition().getRotationZ() + M_PI_2;
		string teamColor = options.svgTeamColor;
		string fillColor = options.svgTeamColor;
//		if (colorMe.length() > 0) {
//			fillColor = colorMe;
//		}
		fprintf(fp,
				"\n<!-- ME -->\n<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"%s\" stroke=\"%s\" stroke-width=\"0.125cm\"/>\n",
				svgX(bar_pos.x - halfRobotSize), svgY(bar_pos.y + halfRobotSize), robotSize, robotSize, teamColor.c_str(), fillColor.c_str());
		if (controlBallByPlayer == 0) {
			fprintf(fp,
					"\n<!-- aiming -->\n<line x1=\"%4.2fcm\" y1=\"%4.2fcm\" x2=\"%4.2fcm\" y2=\"%4.2fcm\" stroke-width=\"0.05cm\"  stroke=\"yellow\"/>\n",
					svgX(bar_pos.x), svgY(bar_pos.y), svgX(bar_pos.x + 12 * cos(r)), svgY(bar_pos.y + 12 * sin(r)));
		}
	}
	for(std::vector<MRA::Geometry::Point>::size_type bar_idx = 1; bar_idx < myTeam.size(); bar_idx++) {
	    MRA::Geometry::Point bar_pos = myTeam[bar_idx].getPosition().getPoint();
		fprintf(fp,
				"\n<!-- Teammate -->\n<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"%s\" stroke=\"%s\" stroke-width=\"0.125cm\"/>\n",
				svgX(bar_pos.x - halfRobotSize), svgY(bar_pos.y + halfRobotSize), robotSize, robotSize, options.svgTeamColor.c_str(), options.svgTeamColor.c_str());
	}

	string last_path_element_color = "yellow";
	string path_color = "blue";
	string additional_options = "";
	if (comparing_player_paths.size() > 0)
	{
		last_path_element_color = "darkolivegreen";
		path_color = "darkolivegreen";
		additional_options = " stroke-dasharray=\"10 4\" ";
	}
	// draw path
	for (team_planner_result_t::size_type pidx = 0;  pidx < player_paths.size(); pidx++) {
//		string startStroke = "blue";
//		if ((player_paths[pidx].path.size() > 0) && (static_cast<planner_target_e>(player_paths[pidx].path[0].target) == planner_target_e::DRIBBLE)) {
//			startStroke = options.svgBallColor;
//		}
		string fillColor = options.svgTeamColor;
//		if (pidx == 0) {
//			if (colorMe.length() > 0) {
//				fillColor = colorMe;
//			}
//		}
		if (player_paths[pidx].path.size() > 0) {
			fprintf(fp,
					"\n<!-- player-path start %d-->\n<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"%s\" stroke=\"%s\" stroke-width=\"0.125cm\"/>\n",
					(int)pidx, svgX(player_paths[pidx].path[0].x - halfRobotSize), svgY(player_paths[pidx].path[0].y + halfRobotSize), robotSize, robotSize,
					options.svgTeamColor.c_str(), fillColor.c_str());
		}
		for (std::vector<planner_piece_t>::size_type j = 1; j < player_paths[pidx].path.size(); j++) {
			double prev_x = (player_paths[pidx]).path[j-1].x;
			double prev_y = (player_paths[pidx]).path[j-1].y;
			double new_x = (player_paths[pidx]).path[j].x;
			double new_y = (player_paths[pidx]).path[j].y;
			if (j == 1) {
				// start first path piece not in middle of player but near the edge of the player
			    MRA::Geometry::Point prev_pos(prev_x, prev_y);
			    MRA::Geometry::Point new_pos(new_x, new_y);
				double alfa = prev_pos.angle(new_pos);
				prev_x = prev_x - (cos(alfa) * halfRobotSize);
				prev_y = prev_y - (sin(alfa) * halfRobotSize);

			}
			fprintf(fp,
					"\n<!-- path player %d-->\n<line x1=\"%4.2fcm\" y1=\"%4.2fcm\" x2=\"%4.2fcm\" y2=\"%4.2fcm\" stroke-width=\"0.125cm\"  stroke=\"%s\" %s/>\n",
					(int)pidx,svgX(prev_x),  svgY(prev_y), svgX(new_x), svgY(new_y), path_color.c_str(), additional_options.c_str());
			if (j == player_paths[pidx].path.size()-1) {
				// last element
					fprintf(fp,
							"\n<!-- path-end player %d-->\n<circle cx=\"%4.2fcm\" cy=\"%4.2fcm\" r=\"%4.2fcm\" fill=\"%s\" stroke=\"%s\" stroke-width=\"0.125cm\"  />\n",
							(int)pidx, svgX((player_paths[pidx].path[j]).x),  svgY((player_paths[pidx].path[j]).y), fieldConfig.getBallRadius()*1.5,
							last_path_element_color.c_str() /* fill color */, last_path_element_color.c_str() /* line color */); // half ball diameter

			}
		}
//		// TARGET: draw ball if first path piece is goto_ball, otherwise a red circle
//		string target_color = options.svgDefaultTargetColor;
//		if ((player_paths[pidx].path.size() > 0) && static_cast<planner_target_e>(player_paths[pidx].path[0].target) == planner_target_e::GOTO_BALL) {
//			target_color = options.svgBallColor;
//		}
//
//		// draw orange circle (ball) as target
//		for(std::vector<Vertex>::size_type idx = 0; idx != m_target.size(); idx++) {
//			fprintf(fp,
//					"<circle cx=\"%4.2fcm\" cy=\"%4.2fcm\" r=\"%4.2fcm\" fill=\"%s\" stroke=\"%s\" stroke-width=\"0.125cm\"  />\n",
//					svgX(m_target[idx]->m_coordinate.x), svgY(m_target[idx]->m_coordinate.y), fieldConfig.getBallRadius(), target_color.c_str(), target_color.c_str());
//		}

	}

	string compare_last_path_element_color = "yellow";
	string compare_path_color = "blue";

	for (team_planner_result_t::size_type pidx = 0;  pidx < comparing_player_paths.size(); pidx++) {
		string startStroke = "blue";

		for (std::vector<planner_piece_t>::size_type j = 1; j < comparing_player_paths[pidx].path.size(); j++) {
			double prev_x = (comparing_player_paths[pidx]).path[j-1].x;
			double prev_y = (comparing_player_paths[pidx]).path[j-1].y;
			double new_x = (comparing_player_paths[pidx]).path[j].x;
			double new_y = (comparing_player_paths[pidx]).path[j].y;
			if (j == 1) {
				// start first path piece not in middle of player but near the edge of the player
			    MRA::Geometry::Point prev_pos(prev_x, prev_y);
			    MRA::Geometry::Point new_pos(new_x, new_y);
				double alfa = prev_pos.angle(new_pos);
				prev_x = prev_x - (cos(alfa) * halfRobotSize);
				prev_y = prev_y - (sin(alfa) * halfRobotSize);

			}
			fprintf(fp,
					"\n<!-- comparing player %d path-->\n<line x1=\"%4.2fcm\" y1=\"%4.2fcm\" x2=\"%4.2fcm\" y2=\"%4.2fcm\" stroke-width=\"0.125cm\"  stroke=\"%s\"/>\n",
					(int)pidx, svgX(prev_x),  svgY(prev_y), svgX(new_x),  svgY(new_y), compare_path_color.c_str());
			if (j == comparing_player_paths[pidx].path.size()-1) {
				// last element
				fprintf(fp,
						"\n<!-- path-end player %d-->\n<circle cx=\"%4.2fcm\" cy=\"%4.2fcm\" r=\"%4.2fcm\" fill=\"%s\" stroke=\"%s\" stroke-width=\"0.125cm\"  />\n",
						(int)pidx, svgX((comparing_player_paths[pidx].path[j]).x),  svgY((comparing_player_paths[pidx].path[j]).y), fieldConfig.getBallRadius()*1.5,
						compare_last_path_element_color.c_str() /* fill color */, compare_last_path_element_color.c_str() /* line color */); // half ball diameter

			}
		}
//		// TARGET: draw ball if first path piece is goto_ball, otherwise a red circle
//		string target_color = options.svgDefaultTargetColor;
//		if ((comparing_player_paths[pidx].path.size() > 0) && static_cast<planner_target_e>(comparing_player_paths[pidx].path[0].target) == planner_target_e::GOTO_BALL) {
//			target_color = options.svgBallColor;
//		}

//		// draw orange circle (ball) as target
//		for(std::vector<Vertex>::size_type idx = 0; idx != m_target.size(); idx++) {
//			fprintf(fp,
//					"<circle cx=\"%4.2fcm\" cy=\"%4.2fcm\" r=\"%4.2fcm\" fill=\"%s\" stroke=\"%s\" stroke-width=\"0.125cm\"  />\n",
//					svgX(m_target[idx]->m_coordinate.x), svgY(m_target[idx]->m_coordinate.y), fieldConfig.getBallRadius(), target_color.c_str(), target_color.c_str());
//		}

	}

	// put player-id on top of the players
	for(std::vector<MRA::Geometry::Point>::size_type bar_idx = 0; bar_idx < myTeam.size(); bar_idx++) {
	    MRA::Geometry::Point bar_pos = myTeam[bar_idx].getPosition().getPoint();
		long robotId = bar_idx+1;
		if (robotIds.size() > bar_idx) {
			robotId = robotIds[bar_idx];
		}
		fprintf(fp,"<text x=\"%4.2fcm\" y=\"%4.2fcm\" font-size=\"large\" font-weight-absolute=\"bold\" fill=\"black\">%lu</text>", svgX(bar_pos.x - 0.65*halfRobotSize), svgY(bar_pos.y - 0.65*halfRobotSize), robotId);
	}

	// BALL
	if (globalBall.isValid()) {
		fprintf(fp,
				"\n<!-- globalBall -->\n<circle cx=\"%4.2fcm\" cy=\"%4.2fcm\" r=\"%4.2fcm\" fill=\"orange\" stroke=\"orange\" stroke-width=\"0.125cm\"  />\n",
				svgX(globalBall.getPosition().getPoint().x),  svgY(globalBall.getPosition().getPoint().y), fieldConfig.getBallRadius()); // half ball diameter
	}
	if (options.svgDrawVelocity) {
	    MRA::Geometry::Point linVel;
		double vrz;
		globalBall.getVelocity(linVel, vrz);
		double speed  = linVel.size();
		if (speed > 1e-6) {
		    MRA::Geometry::Point endVelocityVector = globalBall.getPosition().getPoint();
		    endVelocityVector += linVel;
			// globalBall
			//FIELD - middle line
			fprintf(fp,
					"\n<!-- velocity line -->\n<line x1=\"%4.2fcm\" y1=\"%4.2fcm\" x2=\"%4.2fcm\" y2=\"%4.2fcm\" stroke-width=\"0.125cm\"  stroke=\"red\"/>\n",
					svgX(globalBall.getPosition().getPoint().x), svgY(globalBall.getPosition().getPoint().y),
					svgX(endVelocityVector.x), svgY(endVelocityVector.y));
		}
	}

	fprintf(fp, "</svg>\n");
	fclose(fp);
	MRA_LOG_INFO("created SVG file : %s", options.svgOutputFileName.c_str());
}

