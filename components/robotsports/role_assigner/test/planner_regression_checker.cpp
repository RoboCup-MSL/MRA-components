/**
 *  @file
 *  @brief   main file for testing the xml planner
 *  @curator Jürge van Eijck
 */
#include "SvgUtils.hpp"
#include "FieldConfig.hpp"

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

using namespace std;
using namespace boost::filesystem;
using namespace MRA;

class playerSVGresult {
public:
    int robotId;
    std::string role;
    double target_x;
    double target_y;
    int role_rank;
};


void read_svg_comment_section_and_text_lines(string svgFileName, vector<string>& comment_lines, vector<playerSVGresult>& player_results)
{
    string STRING;
    std::ifstream infile;
    infile.open(svgFileName.c_str());
    string start_comment_section = "<!--";
    string end_comment_section = "-->";
    string end_text_line = "</text>";
    bool in_comment_section = false;
    bool result_section = false;
    int result_idx = 0;
    string result_str = "";
    int line_nr = 0;
    while(!infile.eof()) // To get you all the lines.
    {
        line_nr++;
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
        found = STRING.find(end_text_line);
        if (found != std::string::npos) {
            // </text> in line

            // get text of <text>-section
            std::string text_str = STRING.substr(0, found);
            found = text_str.find(">");
            text_str = text_str.substr(found+1, text_str.length());

            // check if text is start of result (line starts with "R0")
            // when result start is found, next two lines contain the target and role-rank
            found = text_str.find("R0");
            if (found != std::string::npos) {
                result_str = text_str;
                result_section = true;
                result_idx = 1;
            }
            else if (result_section and result_idx < 3) {
                result_str += " " + text_str;
                result_idx++;
                if (result_idx == 3) {
                    result_section = false;
                    // example result_str : "R01: GOALKEEPER target-pos: x: 0.00 y: -8.50 role-rank: 1 costs: 4.221385"
                    int robotId;
                    char role_str[80];
                    double x, y;
                    int rank;
                    playerSVGresult result;
                    sscanf(result_str.c_str(), "R%d: %s target-pos: x: %lf y: %lf role-rank:%d",
                           &robotId, role_str, &x, &y, &rank);
                    result.robotId = robotId;
                    result.role = role_str;
                    result.target_x = x;
                    result.target_y = y;
                    result.role_rank = rank;
                    player_results.push_back(result);
                }
            }
        }
        else {
            if (result_section) {
                result_section = false;
            }
        }
    }
    infile.close();
}

////  globalBall:  x: 3.50 y: -2.25 vx: 0.00 vy: 0.00 vr: 0.00 valid: 1
//MovingObject get_ball_line(const vector<string>& comment_lines) {
//    string ball_string = "globalBall:";
//    double x = 0;
//    double y = 0;
//    double rz = 0;
//    double vx = 0;
//    double vy = 0;
//    double vrz = 0;
//    int ivalid = false;
//    for (std::vector<string>::const_iterator it = comment_lines.begin() ; it != comment_lines.end(); ++it) {
//        string line = *it;
//        std::size_t found_ball = line.find(ball_string);
//        if (found_ball != std::string::npos) {
//            //globalBall:  x: 1.69 y: 1.73 rz: 0.00 vx: 0.00 vy: 0.00 vr: 0.00 valid: 1 label: 0
//            sscanf(line.c_str(), "    globalBall:  x: %lf y: %lf rz: %lf vx: %lf vy: %lf vr: %lf valid: %d", &x, &y, &rz, &vx, &vy, &vrz, &ivalid);
//        }
//    }
//    MovingObject ball = MovingObject(x, y, rz, vx, vy, vrz, 0, static_cast<bool>(ivalid));
//    return ball;
//}
//
//MovingObject get_players(bool ownPlayer, int playerId, const vector<string>& comment_lines, player_type_e& playerType ) {
//    string team_string = "team:";
//    string opponents_string = "opponents:";
//    string options_string = "options:";
//
//    double x = 0;
//    double y = 0;
//    double rz = 0;
//    double vx = 0;
//    double vy = 0;
//    double vrz = 0;
//    int itype = 0;
//    int ivalid = false;
//    bool started = false;
//    for (std::vector<string>::const_iterator it = comment_lines.begin() ; it != comment_lines.end(); ++it) {
//        string line = *it;
//
//        if (!started) {
//            std::size_t start_item;
//            if (ownPlayer) {
//                start_item = line.find(team_string);
//            }
//            else {
//                start_item = line.find(opponents_string);
//            }
//            if (start_item != std::string::npos) {
//                started = true;
//            }
//        }
//        else {
//            std::size_t end_item;
//            if (ownPlayer) {
//                end_item = line.find(opponents_string);
//            }
//            else {
//                end_item = line.find(options_string);
//            }
//            if (end_item != std::string::npos) {
//                break; // done with part
//            }
//        }
//        if (started) {
//            int p = -1;
//            if (ownPlayer) {
//                sscanf(line.c_str(), "         R%02d =  x: %lf y: %lf rz: %lf vx: %lf vy: %lf vr: %lf valid: %d",
//                        &p,&x, &y, &rz, &vx, &vy, &vrz, &ivalid);
//            }
//            else {
//                sscanf(line.c_str(), "        player[%d] = x: %lf y: %lf rz: %lf vx: %lf vy: %lf vr: %lf valid: %d",
//                        &p,&x, &y, &rz, &vx, &vy, &vrz, &ivalid);
//            }
//            if (p == playerId) {
//                break;
//            }
//            x = 0;
//            y = 0;
//            rz = 0;
//            vx = 0;
//            vy = 0;
//            vrz = 0;
//            itype = 0;
//            ivalid = false;
//        }
//    }
//    int label = playerId + ownPlayer?0:10;
//    MovingObject player = MovingObject(x, y, rz, vx, vy, vrz, label, static_cast<bool>(ivalid));
//    playerType = static_cast<player_type_e>(itype);
//
//    return player;
//}

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
        bool& area_fault_found,  bool& unreachable_area_found, bool& infinite_costs_found) {

    int result = 0; // OK
    vector<string> output_comment_lines = vector<string>();
    vector<playerSVGresult> output_svg_results = {};
    read_svg_comment_section_and_text_lines(output_dir + "/" + filename, output_comment_lines, output_svg_results);

    vector<string> regression_comment_lines = vector<string>();
    vector<playerSVGresult> regression_svg_results = {};
    read_svg_comment_section_and_text_lines(regression_dir + "/" + filename, regression_comment_lines, regression_svg_results);

    team_planner_result_t player_paths = team_planner_result_t();
    team_planner_result_t comparing_player_paths = team_planner_result_t();

    if (output_svg_results.size() != regression_svg_results.size()) {
        // different number of results
        result = 1; // FAILED
    }
    else {
        for (auto idx = 0u; idx < output_svg_results.size(); idx++) {
            if (
                (output_svg_results[idx].robotId != regression_svg_results[idx].robotId) or
                (output_svg_results[idx].role.compare(regression_svg_results[idx].role) != 0) or
                (output_svg_results[idx].role_rank != regression_svg_results[idx].role_rank) or
                (fabs(output_svg_results[idx].target_x - regression_svg_results[idx].target_x) > 1e-3) or
                (fabs(output_svg_results[idx].target_y - regression_svg_results[idx].target_y) > 1e-3)
            )
            {
                result = 1; // FAILED
            }
        }


    }

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
            planner_piece_t piece = {};
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
            planner_piece_t piece = {};
            sscanf(regression_player_lines[idx].c_str(), "Path[%d]: x: %lf y: %lf cost: %lf",
                    &p, &piece.x, &piece.y, &piece.cost);
            // create substring of the line after the first (
            string line = regression_player_lines[idx].substr(regression_player_lines[idx].find("(")+1);
            sscanf(line.c_str(), "%d)", &target);
            piece.target = static_cast<planner_target_e>(target);
            org_path.push_back(piece);
        }

        if ( regression_player_lines.size() == output_player_lines.size()) {
            for (std::vector<string>::iterator it = regression_player_lines.begin(); it != regression_player_lines.end(); ++it) {
                if (std::find(output_player_lines.begin(), output_player_lines.end(), (*it)) != output_player_lines.end()) {
                    // found; same
                    //cerr << "SAME line of path for player: " << i << " (zero-based): " << *it <<  endl;
                }
                else {
                    // not equal in length (regression !)
                    //cerr << "REGRESSION: NOT EQUAL path for player: " << i << " (zero-based) in file: " << filename << endl;
                }
            }
        }
        else {
            // not equal in length (regression !)
            //cerr << "REGRESSION: DIFFERENT lengths of path for player: " << i << " (zero-based) in file: " << filename << endl;
        }
    }
//    bool create_diff_svg = false;
//    if (create_diff_svg and result != 0) {
//        game_state_e gamestate = game_state_e::NONE;
//        long controlBallByPlayerId = -1;
//        ball_status_t ball_status = ball_status_e::FREE;
//        std::vector<Vector2D> parking_positions = {};
//        previous_used_ball_by_planner_t previous_ball = {};
//        ball_pickup_position_t ball_pickup_position = {};
//        bool passIsRequired = false;
//        long passBallByPlayerId = -1;
//        pass_data_t pass_data = {};
//        const std::vector<final_planner_result_t> previous_planner_results = {};
//        const std::vector<double> time_in_own_penalty_area = {};
//        const std::vector<double> time_in_opponent_penalty_area = {};
//        MovingObject ball = get_ball_line(output_comment_lines);
//        cerr << "DIFF BALL : " << ball.toString() << endl << flush;
//        PlannerOptions options = PlannerOptions();
//        string save_name = "diff_" + filename;
//
//        TeamPlannerData teamplannerData(fieldConfig);
//        teamplannerData.fillData(gamestate, ball, myTeam, opponents, ball_status,
//                controlBallByPlayerId, teamTypes, robotIds, options,
//                parking_positions, previous_ball, previous_planner_results, ball_pickup_position, passIsRequired,
//                passBallByPlayerId, pass_data, time_in_own_penalty_area, time_in_opponent_penalty_area);
//
//        SvgUtils::plannerdata_to_svg(player_paths, teamplannerData, fieldConfig, save_name, comparing_player_paths);
//    }

    // valid if no collisions or area faults are planned.
//        bool collisionDetected = false;
//        bool areaFault = false;
//        bool outsideReachableArea = false;
//        bool infiniteCostDetected = false;
//        for (unsigned p_idx = 0; p_idx < comparing_player_paths.size(); p_idx++) {
//            if (comparing_player_paths[p_idx].path.size() == 0) {
//                continue; // no path found for this player
//            }
//            for (unsigned pp_idx = 0; pp_idx < (comparing_player_paths[p_idx]).path.size(); pp_idx++) {
//                planner_piece_t     piece = (comparing_player_paths[p_idx]).path[pp_idx];
//                if (!fieldConfig.isInReachableField(piece.x, piece.y)) {
//                    outsideReachableArea = true;
//                }
//                if (std::isinf(piece.cost)) {
//                    infiniteCostDetected = true;
//                }
//            }
//
//
//            planner_piece_t     last_piece = (comparing_player_paths[p_idx]).path[(comparing_player_paths[p_idx]).path.size()-1];
//            Vector2D end_position_current_player = Vector2D(last_piece.x, last_piece.y);
//            // cerr << "end position  robot[" << p_idx << "] = " << end_position_current_player.toString() << endl;
//
//        //    QRectF(mX(-0.5*m_fieldConfig.getGoalAreaWidth()), mY(m_dYMin+m_fieldConfig.GOAL_AREA_LENGTH), mW(m_fieldConfig.getGoalAreaWidth()), mH(m_fieldConfig.GOAL_AREA_LENGTH)),0, m_pScene);
//
//            // check on area fault: not in opponent goal area
//            if (fabs(end_position_current_player.m_x) < (fieldConfig.getGoalAreaWidth()*0.5 + fieldConfig.getRobotRadius())
//             && (end_position_current_player.m_y) > ((fieldConfig.getMaxFieldY()) - (fieldConfig.getGoalAreaLength()+ fieldConfig.getRobotRadius())) )
//            {
//                if (static_cast<planner_target_e>(last_piece.target) != planner_target_e::DRIBBLE) {
//                    if (!known_area_fault) {
//                        areaFault = true; // this is not the player with the ball. So area fault
//                        cout << "Goal area fault detected in: " << filename << endl;
//                        cout << "mx: " << end_position_current_player.m_x << endl;
//                        cout << "my: " << end_position_current_player.m_y << endl;
//                        cout << "x lim < " << (fieldConfig.getGoalAreaWidth()*0.5 + fieldConfig.getRobotRadius()) << endl;
//                        cout << "y lim > " << ((fieldConfig.getMaxFieldY()) - (fieldConfig.getGoalAreaLength()+ fieldConfig.getRobotRadius())) << endl;
//                    }
//                }
//            }
//
//            // check on area fault: not in own goal area
//            if (fabs(end_position_current_player.m_x) < (fieldConfig.getGoalAreaWidth()*0.5 + fieldConfig.getRobotRadius())
//             && (end_position_current_player.m_y) < (-(fieldConfig.getMaxFieldY()) + (fieldConfig.getGoalAreaLength()+ fieldConfig.getRobotRadius()) ) )
//            {
//                if (static_cast<planner_target_e>(last_piece.target) != planner_target_e::GOALIE) {
//                    if (!known_area_fault) {
//                        areaFault = true; // this is not the goalie, so an area fault
//                        cout << "own area fault detected in: " << filename << endl;
//                        cout << "mx: " << end_position_current_player.m_x << endl;
//                        cout << "my: " << end_position_current_player.m_y << endl;
//                        cout << "x lim < " << (fieldConfig.getGoalAreaWidth()*0.5 + fieldConfig.getRobotRadius()) << endl;
//                        cout << "y lim < " << (-(fieldConfig.getMaxFieldY()) + (fieldConfig.getGoalAreaLength()+ fieldConfig.getRobotRadius())) << endl;
//                    }
//                }
//            }
//
//            // check all other players on collision
//            for (unsigned cmp_idx = 0; cmp_idx < comparing_player_paths.size(); cmp_idx++) {
//                if (cmp_idx == p_idx) {
//                    continue; // comparing robot is current player
//                }
//                if (comparing_player_paths[cmp_idx].path.size() == 0) {
//                    continue; // no path found for comparing player
//                }
//                planner_piece_t     last_piece_compare = comparing_player_paths[cmp_idx].path[comparing_player_paths[cmp_idx].path.size()-1];
//                Vector2D end_position_comparing = Vector2D(last_piece_compare.x, last_piece_compare.y);
//                if (end_position_comparing.distanceTo(end_position_current_player) < 0.5) {
//                    collisionDetected = true;
//                }
//            }
//        }
//        if (infiniteCostDetected or infiniteCostDetected or collisionDetected or areaFault)
//        {
//            string save_name = filename;
//            game_state_e gamestate = game_state_e::NONE;
//            int controlBallByPlayerId = -1;
//            ball_status_t ball_status = ball_status_e::FREE;
//            MovingObject ball = get_ball_line(output_comment_lines);
//            PlannerOptions options = PlannerOptions();
//            std::vector<Vector2D> parking_positions = {};
//            previous_used_ball_by_planner_t previous_ball = {};
//            ball_pickup_position_t ball_pickup_position = {};
//            bool passIsRequired = false;
//            long passBallByPlayerId = -1;
//            pass_data_t pass_data = {};
//            const std::vector<final_planner_result_t> previous_planner_results = {};
//            const std::vector<double> time_in_own_penalty_area = {};
//            const std::vector<double> time_in_opponent_penalty_area = {};
//
//            TeamPlannerData teamplannerData(fieldConfig);
//            teamplannerData.fillData(gamestate, ball, myTeam, opponents, ball_status,
//                    controlBallByPlayerId, teamTypes, robotIds, options,
//                    parking_positions, previous_ball, previous_planner_results, ball_pickup_position, passIsRequired,
//                    passBallByPlayerId, pass_data, time_in_own_penalty_area, time_in_opponent_penalty_area);
//
//            if (infiniteCostDetected) {
//                infinite_costs_found = infiniteCostDetected;
//                save_name = "infinite_costs_" + filename;
//                SvgUtils::plannerdata_to_svg(player_paths, teamplannerData, fieldConfig, save_name, comparing_player_paths);
//            }
//
//            if (outsideReachableArea) {
//                unreachable_area_found = outsideReachableArea;
//                save_name = "unreachable_area_" + filename;
//                SvgUtils::plannerdata_to_svg(player_paths, teamplannerData, fieldConfig, save_name, comparing_player_paths);
//            }
//            if (collisionDetected) {
//                collision_found = collisionDetected;
//                save_name = "collision_" + filename;
//                SvgUtils::plannerdata_to_svg(player_paths, teamplannerData, fieldConfig, save_name, comparing_player_paths);
//            }
//            if (areaFault) {
//                area_fault_found = areaFault;
//                save_name = "areafault_" + filename;
//                SvgUtils::plannerdata_to_svg(player_paths, teamplannerData, fieldConfig, save_name, comparing_player_paths);
//            }
//        }
//    }
    return result;
}


int main(int argc, char *argv[]) {
        if (argc < 3) {
            cerr << "use program: " << argv[0] << " <regression-directory> <compare-to-direcotry> " << endl;
            return -1;
        }
    try {
        path compare_new ("./compare_new");
        path compare_old ("./compare_old");

        if (!boost::filesystem::is_directory(compare_new)) {
            boost::filesystem::remove(compare_new);
        }
        if (!boost::filesystem::is_directory(compare_old)) {
            boost::filesystem::remove(compare_old);
        }


        // convert string to vector of strings (filenames)
        std::vector<std::string> missing_files = std::vector<std::string>();
        // get files in output-directory
        vector<string> output_files = vector<string>();
        string output_dir = argv[2];
        get_files_in_directory(output_dir, output_files, ".svg");

        // get files in regression-directory
        string regression_dir = argv[1];
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
                        unreachable_area_found, infinite_costs_found);
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
                    // cerr << "OK no difference found." << endl;
                }
                else {
                    // files are not equal
                    cerr << "difference found in: " << reg_file  << endl;

                    // copy files to compare directories
                    if (!boost::filesystem::is_directory(compare_new)) {
                        boost::filesystem::create_directories(compare_new);
                    }
                    if (!boost::filesystem::is_directory(compare_old)) {
                        boost::filesystem::create_directories(compare_old);
                    }
                    boost::filesystem::copy_file(path(regression_dir) / path(reg_file), compare_old / path(reg_file), copy_option::overwrite_if_exists);
                    boost::filesystem::copy_file(path(output_dir) / path(reg_file), compare_new / path(reg_file), copy_option::overwrite_if_exists);
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
                if (!boost::filesystem::is_directory(compare_new)) {
                    boost::filesystem::create_directories(compare_new);
                }
                boost::filesystem::copy_file(path(regression_dir) / path(reg_file), compare_new / path(reg_file));

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
        cout << endl;
        cout << endl;
        if (nr_failed > 0) {
            cout << "Failed files can be compared:" << endl;
            cout << "   reference-files in directory: " << compare_old << endl;
            cout << "   new-files  in directory     : " << compare_new << endl;
        }

    }
    catch (std::exception & e)
    {
        cerr << "Exception:: " << e.what() << endl << flush;
    }
    catch (...) {
        cerr << "General Exception in " << argv[0] << endl << flush;
    }
}
