/**
 *  @file
 *  @brief   main file for testing the xml planner
 *  @curator Jürge van Eijck
 */
#include <iostream>
#include <signal.h>
#include <fstream>
#include <string>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <boost/filesystem.hpp>
#include "SvgUtils.hpp"
#include "FieldConfig.h"
#include "MovingObject.h"

using namespace std;
using namespace boost::filesystem;
using namespace MRA;


void read_svg_comment_section(string svgFileName, vector<string>& comment_lines)
{
	string STRING;
	std::ifstream infile;
	infile.open(svgFileName.c_str());
	string start_comment_section = "<!--";
	string end_comment_section = "-->";
	bool in_comment_section = false;
	while(!infile.eof()) // To get you all the lines.
	{
		getline(infile,STRING); // Saves the line in STRING.
		std::size_t found = STRING.find(start_comment_section);
		if (found != std::string::npos) {
			in_comment_section = true;
		}
		if (in_comment_section) {
			comment_lines.push_back(STRING);
		}
		found = STRING.find(end_comment_section);
		if (found != std::string::npos) {
			in_comment_section = false;
		}
	}
	infile.close();
}

//  globalBall:  x: 3.50 y: -2.25 vx: 0.00 vy: 0.00 vr: 0.00 valid: 1
MovingObject get_ball_line(const vector<string>& comment_lines) {
	string ball_string = "globalBall:";
	double x = 0;
	double y = 0;
	double rz = 0;
	double vx = 0;
	double vy = 0;
	double vrz = 0;
	int ivalid = false;
	for (std::vector<string>::const_iterator it = comment_lines.begin() ; it != comment_lines.end(); ++it) {
		string line = *it;
		std::size_t found_ball = line.find(ball_string);
		if (found_ball != std::string::npos) {
			//globalBall:  x: 1.69 y: 1.73 rz: 0.00 vx: 0.00 vy: 0.00 vr: 0.00 valid: 1 label: 0
			sscanf(line.c_str(), "	globalBall:  x: %lf y: %lf rz: %lf vx: %lf vy: %lf vr: %lf valid: %d", &x, &y, &rz, &vx, &vy, &vrz, &ivalid);
		}
	}
	// TODO what if NOT VALID
	MovingObject ball(x, y, rz, vx, vy, vrz, 0);
	return ball;
}

MovingObject get_players(bool& r_valid, bool ownPlayer, int playerId, const vector<string>& comment_lines, player_type_e& playerType ) {
    r_valid = false;
	string team_string = "team:";
	string opponents_string = "opponents:";
	string options_string = "options:";

	double x = 0;
	double y = 0;
	double rz = 0;
	double vx = 0;
	double vy = 0;
	double vrz = 0;
	int itype = 0;
	int ivalid = false;
	bool started = false;
	for (std::vector<string>::const_iterator it = comment_lines.begin() ; it != comment_lines.end(); ++it) {
		string line = *it;

		if (!started) {
			std::size_t start_item;
			if (ownPlayer) {
				start_item = line.find(team_string);
			}
			else {
				start_item = line.find(opponents_string);
			}
			if (start_item != std::string::npos) {
				started = true;
			}
		}
		else {
			std::size_t end_item;
			if (ownPlayer) {
				end_item = line.find(opponents_string);
			}
			else {
				end_item = line.find(options_string);
			}
			if (end_item != std::string::npos) {
				break; // done with part
			}
		}
		if (started) {
			int p = -1;
			if (ownPlayer) {
				sscanf(line.c_str(), " 		R%02d =  x: %lf y: %lf rz: %lf vx: %lf vy: %lf vr: %lf valid: %d",
						&p,&x, &y, &rz, &vx, &vy, &vrz, &ivalid);
			}
			else {
				sscanf(line.c_str(), "		player[%d] = x: %lf y: %lf rz: %lf vx: %lf vy: %lf vr: %lf valid: %d",
						&p,&x, &y, &rz, &vx, &vy, &vrz, &ivalid);
			}
			if (p == playerId) {
				break;
			}
			x = 0;
			y = 0;
			rz = 0;
			vx = 0;
			vy = 0;
			vrz = 0;
			itype = 0;
			ivalid = false;
		}
	}
	int label = playerId + ownPlayer?0:10;
	// TODO handle invalid
	MovingObject player = MovingObject(x, y, rz, vx, vy, vrz, label);
	playerType = static_cast<player_type_e>(itype);
	r_valid = ivalid;
	return player;
}

void get_player_path(int playerId, const vector<string>& comment_lines, vector<string>& player_lines) {
	// search in comment lines for Path[*] lines, this are results of the paths.
	// search can stop if "options:" is found

	string player_string = "Path[" + std::to_string(playerId) + "]:";
	string options_string = "options:";
	for (std::vector<string>::const_iterator it = comment_lines.begin() ; it != comment_lines.end(); ++it) {
		string line = *it;

		std::size_t found = line.find(player_string);
		if (found != std::string::npos) {
			// found: add to player list
			player_lines.push_back(line);
		}

		std::size_t found_options = line.find(options_string);
		if (found_options != std::string::npos) {
			break; // options section found
		}
	}
}

void get_files_in_directory(string dir_name, vector<string>& filenames, string filename_filter)
{
	path p(dir_name);
	for (auto i = directory_iterator(p); i != directory_iterator(); i++)
	{
		if (!is_directory(i->path())) //we eliminate directories
		{
			string filename = i->path().filename().string();
			if (filename_filter.size() > 0) {
				// filter defined, check if filename matches with filter
				std::size_t found = filename.find(filename_filter);
				if (found != std::string::npos) {
					filenames.push_back(filename);  // match filter, add filename to filelist
				}
			}
			else {
				filenames.push_back(filename); // no filter, add filename to filelist
			}
		}
		else
			continue;
	}
}

int compare_svg_files(const string& filename, const string& regression_dir, const string& output_dir, bool& collision_found,
		bool& area_fault_found,  bool& unreachable_area_found, bool& infinite_costs_found, const vector<string>& rKnown_area_faults) {
	FieldConfig fieldConfig = FillDefaultFieldConfig();
	bool known_area_fault = false;

	for (auto kaf_it = rKnown_area_faults.begin(); kaf_it != rKnown_area_faults.end(); ++kaf_it) {
		if (filename.compare(*kaf_it) == 0) { // strings are equal
			known_area_fault = true;
		}
	}

	int result = 0; // OK
	vector<string> output_comment_lines = vector<string>();
	read_svg_comment_section(output_dir + "/" + filename, output_comment_lines);
	vector<string> regression_comment_lines = vector<string>();
	read_svg_comment_section(regression_dir + "/" + filename, regression_comment_lines);

	cout << "filename: " << filename << endl;
	vector<player_type_e> teamTypes = vector<player_type_e>();
	vector<long> robotIds = vector<long>();
	vector<MovingObject>  myTeam = vector<MovingObject>();
	vector<MovingObject>  opponents = vector<MovingObject>();
	team_planner_result_t player_paths = team_planner_result_t();
	team_planner_result_t comparing_player_paths = team_planner_result_t();

	for (int i = 0; i < 6; i++) {
		vector<string> output_player_lines = vector<string>();
		get_player_path(i, output_comment_lines, output_player_lines);
		vector<string> regression_player_lines = vector<string>();
		get_player_path(i, regression_comment_lines, regression_player_lines);

		vector<planner_piece_t> new_path = vector<planner_piece_t>();
		for (unsigned idx = 0; idx < output_player_lines.size(); idx++) {  //== output_player_lines.size()) {
			//Path[4]: x: 0 y: -9 cost: 2.04274 target: Prepare restart (7)
			int p = 0;
			int target = 0;
			planner_piece_t piece = {0};
			sscanf(output_player_lines[idx].c_str(), "Path[%d]: x: %lf y: %lf cost: %lf",
					&p, &piece.x, &piece.y, &piece.cost);
			// create substring of the line after the first (
			string line = output_player_lines[idx].substr(output_player_lines[idx].find("(")+1);
			sscanf(line.c_str(), "%d)", &target);

			piece.target = static_cast<planner_target_e>(target);
			new_path.push_back(piece);
		}
		PlayerPlannerResult playerResult = PlayerPlannerResult();
		playerResult.path = new_path;
		comparing_player_paths.push_back(playerResult);

		vector<planner_piece_t> org_path = vector<planner_piece_t>();
		for (unsigned idx = 0; idx < regression_player_lines.size(); idx++) {  //== output_player_lines.size()) {
			//Path[4]: x: 0 y: -9 cost: 2.04274 target: 7 ( Prepare restart )
			int p = 0;
			int target = 0;
			planner_piece_t piece = {0};
			sscanf(regression_player_lines[idx].c_str(), "Path[%d]: x: %lf y: %lf cost: %lf",
					&p, &piece.x, &piece.y, &piece.cost);
			// create substring of the line after the first (
			string line = regression_player_lines[idx].substr(regression_player_lines[idx].find("(")+1);
			sscanf(line.c_str(), "%d)", &target);
			piece.target = static_cast<planner_target_e>(target);
			org_path.push_back(piece);
		}
		PlayerPlannerResult orgPlayerResult = PlayerPlannerResult();
		orgPlayerResult.path = org_path;
		player_paths.push_back(orgPlayerResult);

		if ( regression_player_lines.size() == output_player_lines.size()) {
			for (std::vector<string>::iterator it = regression_player_lines.begin(); it != regression_player_lines.end(); ++it) {
				if (std::find(output_player_lines.begin(), output_player_lines.end(), (*it)) != output_player_lines.end()) {
					// found; same
					//cerr << "SAME line of path for player: " << i << " (zero-based): " << *it <<  endl;
				}
				else {
					// not equal in length (regression !)
					cerr << "REGRESSION: NOT EQUAL path for player: " << i << " (zero-based) in file: " << filename << endl;
					result = 1; // FAILED
				}
			}
		}
		else {
			// not equal in length (regression !)
			cerr << "REGRESSION: DIFFERENT lengths of path for player: " << i << " (zero-based) in file: " << filename << endl;
			result = 1; // FAILED
		}
		player_type_e opponentType = player_type_e::RESERVE;
		bool is_valid = false;
		MovingObject opponent_player = get_players(is_valid, false, i, output_comment_lines, opponentType);
		if (is_valid) {
			opponents.push_back(opponent_player);
		}
		player_type_e playerType = player_type_e::RESERVE;
		MovingObject player = get_players(is_valid, true, i, output_comment_lines, playerType);
		if (is_valid) {
			myTeam.push_back(player);
			teamTypes.push_back(playerType);
			robotIds.push_back(i);
		}
	}
	if (result != 0) {
		game_state_e gamestate = game_state_e::NONE;
		long controlBallByPlayer = -1;
		MovingObject ball = get_ball_line(output_comment_lines);
		cerr << "DIFF BALL : " << ball.toString() << endl << flush;
		TeamPlannerParameters options = TeamPlannerParameters();
		options.svgOutputFileName = "diff_" + filename;

		// TODO handle localball correctly towards svg
		bool hasTeamPlannerInputInfo = false;
		SvgUtils::save_graph_as_svg(ball, myTeam, opponents, player_paths, comparing_player_paths,
				options, std::vector<Vertex* >(), gamestate, controlBallByPlayer, teamTypes, robotIds, "",  fieldConfig);
	}

	// valid if no collisions or area faults are planned.
	if (true)
	{
		bool collisionDetected = false;
		bool areaFault = false;
		bool outsideReachableArea = false;
		bool infiniteCostDetected = false;
		for (unsigned p_idx = 0; p_idx < comparing_player_paths.size(); p_idx++) {
			if (comparing_player_paths[p_idx].path.size() == 0) {
				continue; // no path found for this player
			}
			for (unsigned pp_idx = 0; pp_idx < (comparing_player_paths[p_idx]).path.size(); pp_idx++) {
				planner_piece_t	 piece = (comparing_player_paths[p_idx]).path[pp_idx];
				if (!fieldConfig.isInReachableField(piece.x, piece.y)) {
					outsideReachableArea = true;
				}
				if (std::isinf(piece.cost)) {
					infiniteCostDetected = true;
				}
			}


			planner_piece_t	 last_piece = (comparing_player_paths[p_idx]).path[(comparing_player_paths[p_idx]).path.size()-1];
			Geometry::Point end_position_current_player = Geometry::Point(last_piece.x, last_piece.y);
			cerr << "end position  robot[" << p_idx << "] = " << end_position_current_player.toString() << endl;

		//	QRectF(mX(-0.5*m_fieldConfig.getGoalAreaWidth()), mY(m_dYMin+m_fieldConfig.GOAL_AREA_LENGTH), mW(m_fieldConfig.getGoalAreaWidth()), mH(m_fieldConfig.GOAL_AREA_LENGTH)),0, m_pScene);

			// check on area fault: not in opponent penalty area
			if (fabs(end_position_current_player.x) < (fieldConfig.getPenaltyAreaWidth()*0.5 + fieldConfig.getRobotRadius())
			 && (end_position_current_player.y) > ((fieldConfig.getMaxFieldY()) - (fieldConfig.getPenaltyAreaLength()+ fieldConfig.getRobotRadius())) )
		    {
				if (static_cast<planner_target_e>(last_piece.target) != planner_target_e::DRIBBLE) {
					if (!known_area_fault) {
						areaFault = true; // this is not the player with the ball. So area fault
						cout << "Penalty area fault detected in: " << filename << endl;
						cout << "mx: " << end_position_current_player.x << endl;
						cout << "my: " << end_position_current_player.y << endl;
						cout << "x lim < " << (fieldConfig.getPenaltyAreaWidth()*0.5 + fieldConfig.getRobotRadius()) << endl;
						cout << "y lim > " << ((fieldConfig.getMaxFieldY()) - (fieldConfig.getPenaltyAreaLength()+ fieldConfig.getRobotRadius())) << endl;
					}
				}
			}

			// check on area fault: not in own goal area
			if (fabs(end_position_current_player.x) < (fieldConfig.getGoalAreaWidth()*0.5 + fieldConfig.getRobotRadius())
			 && (end_position_current_player.y) < (-(fieldConfig.getMaxFieldY()) + (fieldConfig.getGoalAreaLength()+ fieldConfig.getRobotRadius()) ) )
		    {
				if (static_cast<planner_target_e>(last_piece.target) != planner_target_e::GOALIE) {
					if (!known_area_fault) {
						areaFault = true; // this is not the goalie, so an area fault
						cout << "own area fault detected in: " << filename << endl;
						cout << "mx: " << end_position_current_player.x << endl;
						cout << "my: " << end_position_current_player.y << endl;
						cout << "x lim < " << (fieldConfig.getGoalAreaWidth()*0.5 + fieldConfig.getRobotRadius()) << endl;
						cout << "y lim < " << (-(fieldConfig.getMaxFieldY()) + (fieldConfig.getGoalAreaLength()+ fieldConfig.getRobotRadius())) << endl;
					}
				}
			}

			// check all other players on collision
			for (unsigned cmp_idx = 0; cmp_idx < comparing_player_paths.size(); cmp_idx++) {
				if (cmp_idx == p_idx) {
					continue; // comparing robot is current player
				}
				if (comparing_player_paths[cmp_idx].path.size() == 0) {
					continue; // no path found for comparing player
				}
				planner_piece_t	 last_piece_compare = comparing_player_paths[cmp_idx].path[comparing_player_paths[cmp_idx].path.size()-1];
				Geometry::Point end_position_comparing = Geometry::Point(last_piece_compare.x, last_piece_compare.y);
				if (end_position_comparing.distanceTo(end_position_current_player) < 0.5) {
					collisionDetected = true;
				}
			}
		}
		bool hasTeamPlannerInputInfo = false;
		TeamPlannerInputInfo  inputInfo;
		if (infiniteCostDetected) {
			infinite_costs_found = infiniteCostDetected;
			game_state_e gamestate = game_state_e::NONE;
			int controlBallByPlayer = -1;
			MovingObject ball = get_ball_line(output_comment_lines);
			PlannerOptions options = PlannerOptions();
			options.svgOutputFileName = "inifite_costs_" + filename;
			// TODO handle localball correctly towards svg
			SvgUtils::save_graph_as_svg(ball, myTeam, opponents, comparing_player_paths, team_planner_result_t(),
					options, std::vector<Vertex* >(), gamestate, controlBallByPlayer, teamTypes, robotIds, "",  fieldConfig,
					hasTeamPlannerInputInfo, inputInfo);
		}
		if (outsideReachableArea) {
			unreachable_area_found = outsideReachableArea;
			game_state_e gamestate = game_state_e::NONE;
			int controlBallByPlayer = -1;
			MovingObject ball = get_ball_line(output_comment_lines);
			PlannerOptions options = PlannerOptions();
			options.svgOutputFileName = "unreachable_area_" + filename;
			// TODO handle localball correctly towards svg
			SvgUtils::save_graph_as_svg(ball, myTeam, opponents, comparing_player_paths, team_planner_result_t(),
					options, std::vector<Vertex* >(), gamestate, controlBallByPlayer, teamTypes, robotIds, "",  fieldConfig,
					hasTeamPlannerInputInfo, inputInfo);
		}
		if (collisionDetected) {
			collision_found = collisionDetected;
			game_state_e gamestate = game_state_e::NONE;
			int controlBallByPlayer = -1;
			MovingObject ball = get_ball_line(output_comment_lines);
			PlannerOptions options = PlannerOptions();
			options.svgOutputFileName = "collision_" + filename;
			// TODO handle localball correctly towards svg
			SvgUtils::save_graph_as_svg(ball, myTeam, opponents, comparing_player_paths, team_planner_result_t(),
					options, std::vector<Vertex* >(), gamestate, controlBallByPlayer, teamTypes, robotIds, "",  fieldConfig,
					hasTeamPlannerInputInfo, inputInfo);
		}
		if (areaFault) {
			area_fault_found = areaFault;
			game_state_e gamestate = game_state_e::NONE;
			int controlBallByPlayer = -1;
			MovingObject ball = get_ball_line(output_comment_lines);
			PlannerOptions options = PlannerOptions();
			options.svgOutputFileName = "areafault_" + filename;
			// TODO handle localball correctly towards svg
			SvgUtils::save_graph_as_svg(ball, myTeam, opponents, comparing_player_paths, team_planner_result_t(),
					options, std::vector<Vertex* >(), gamestate, controlBallByPlayer, teamTypes, robotIds, "",  fieldConfig,
					hasTeamPlannerInputInfo, inputInfo);
		}
	}

	return result;
}

int main(int argc, char *argv[]) {
	//	if (argc < 2) {
	//		cerr << "use program: " << argv[0] << " <filename.xml> " << endl;
	//		return -1;
	//	}
	(void)signal(SIGSEGV, SegmentationHandler);   // install our handler
	try {
	    INIReader reader("planner_regression_checker.ini");
	    string all_known_area_faults = reader.Get("Known Areafaults", "files", ""); // read known area issue files from ini-file (multiline input)
	    // convert string to vector of strings (filenames)
	    istringstream iss(all_known_area_faults);
	    vector<string> known_area_faults(istream_iterator<string>{iss}, istream_iterator<string>());

	    std::vector<std::string> missing_files = std::vector<std::string>();
		// get files in output-directory
		vector<string> output_files = vector<string>();
		string output_dir = "output_team";
		get_files_in_directory(output_dir, output_files, ".svg");

		// get files in regression-directory
		string regression_dir = "regression";
		vector<string> regression_files = vector<string>();
		get_files_in_directory(regression_dir, regression_files, ".svg");

		// sort output-list and regression-list
		std::sort (output_files.begin(),output_files.end());
		std::sort (regression_files.begin(),regression_files.end());


 		cout << endl << "+---------------------------------+" << endl
                     << "|  START checking on regression   |" << endl
					 << "+---------------------------------+" << endl;
		cout << "Number of files in regression directory: " << regression_files.size() << endl << endl;
		for (std::vector<string>::iterator it = output_files.begin() ; it != output_files.end(); ++it) {
			string output_file = *it;
			// find corresponding output file
			if (std::find(regression_files.begin(), regression_files.end(), output_file) == regression_files.end()) {
				// filename not in regression list
				//file is missing in output
				cerr << "REGRESSION: "<<  output_file << " is NOT in the regression-directory." << endl;
			}
		}
		int nr_missing = 0;
		int nr_ok = 0;
		int nr_failed = 0;
		int nr_collisions_found = 0;
		int nr_area_faults_found = 0;
		int nr_unreachable_area = 0;
		int nr_infinite_costs = 0;
		for (std::vector<string>::iterator rit = regression_files.begin() ; rit != regression_files.end(); ++rit) {
			string reg_file = *rit;
			// find corresponding output file
			if (std::find(output_files.begin(), output_files.end(), reg_file) != output_files.end()) {
				// filename in both lists. Compare the files
				bool collision_found = false;
				bool area_fault_found = false;
				bool unreachable_area_found = false;
				bool infinite_costs_found = false;
				int cmp_result = compare_svg_files(reg_file, regression_dir, output_dir, collision_found, area_fault_found,
						unreachable_area_found, infinite_costs_found, known_area_faults);
				if (infinite_costs_found) {
					nr_infinite_costs++;
				}
				if (collision_found) {
					nr_collisions_found++;
				}
				if (area_fault_found) {
					nr_area_faults_found++;
				}
				if (unreachable_area_found) {
					nr_unreachable_area++;
				}
				if (cmp_result == 0) {
					nr_ok++;
					cerr << "OK no difference found." << endl;
				}
				else {
					// files are not equal
					// copy files to compare directories
					namespace fs = boost::filesystem;
					path compare_new ("./compare_new");
					path compare_old ("./compare_old");
					if (!fs::is_directory(compare_new)) {
						fs::create_directories(compare_new);
					}
					if (!fs::is_directory(compare_old)) {
						fs::create_directories(compare_old);
					}
					fs::copy_file(path(regression_dir) / path(reg_file), compare_old / path(reg_file), copy_option::overwrite_if_exists);
					fs::copy_file(path(output_dir) / path(reg_file), compare_new / path(reg_file), copy_option::overwrite_if_exists);
					nr_failed++;
				}
			}
			else {
				//file is missing in output
				cerr << "REGRESSION: "<<  reg_file << " is NOT in the output-directory." << endl;
				nr_missing++;

				// copy missing file to compare directory
				namespace fs = boost::filesystem;
				path compare_new ("./compare_new");
				if (!fs::is_directory(compare_new)) {
					fs::create_directories(compare_new);
				}
				fs::copy_file(path(regression_dir) / path(reg_file), compare_new / path(reg_file));

				missing_files.push_back(reg_file);
			}
		}

 		cout << endl << "+---------------------------------+" << endl
                     << "| FINISHED checking on regression |" << endl
					 << "+---------------------------------+" << endl;
 		for (auto it = missing_files.begin(); it != missing_files.end(); ++it) {
 			cout << "missing: " << *it << endl;
 		}
 		cout << "#OK = " << nr_ok << endl;
 		cout << "#FAILED = " << nr_failed << endl;
 		cout << "#MISSING = " << nr_missing << endl;
 		cout << "#COLLISIONS = " << nr_collisions_found << endl;
 		cout << "#AREA_FAULTS = " << nr_area_faults_found << endl;
 		cout << "#UNREACHABLE AREA = " << nr_unreachable_area << endl;
 		cout << "#INFINITE COSTS = " << nr_infinite_costs << endl;
	}
	catch (std::exception & e)
	{
		cerr << "Exception:: " << e.what() << endl << flush;
	}
	catch (...) {
		cerr << "General Exception in " << argv[0] << endl << flush;
	}
}
