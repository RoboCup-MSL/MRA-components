/**
 *  @file
 *  @brief   Utility class for plotting planner to svg file
 *  @curator Jürge van Eijck
 */
#include "MathUtils.hpp"
#include "logging.hpp"
#include "Vertex.hpp"
#include "RoleAssignerData.hpp"
//class RoleAssignerResult;

#include <cmath>
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
#include <vector>
#include <sys/stat.h>
#include "RoleAssignerSvg.hpp"
#include "Environment.hpp"

using namespace std;
using namespace MRA;




Environment RoleAssignerSvg::m_environment(FillDefaultEnvironment());

/**
 * Get svg x coordinate for given field X
 */
double RoleAssignerSvg::svgX(double fieldX) {
    return m_environment.getMaxFullFieldX() + fieldX;
}

/**
 * Get svg x coordinate for given field Y
 */
double RoleAssignerSvg::svgY(double fieldY) {
    return m_environment.getMaxFullFieldY() - fieldY;
}


// Data class for storing file parts
class FileParts
{
public:
    std::string path;
    std::string name;
    std::string ext;
};

// split filename in fileparts (compared to matlab fileparts function)
// split in path, name and extension
static FileParts fileparts(const std::string& filename)
{
    int idx0 = filename.rfind("/");
    int idx1 = filename.rfind(".");

    FileParts fp;
    fp.path = filename.substr(0,idx0+1);
    fp.name = filename.substr(idx0+1,idx1-idx0-1);
    fp.ext  = filename.substr(idx1);
    return fp;
}

/**
 * return true if path is a directory, otherwise return false
 */
static bool isDirectory(const string& path)
{
    struct stat info;

    if(stat( path.c_str(), &info ) != 0) {
        return false;
    }
    else if(info.st_mode & S_IFDIR) {
        return true;
    }
    return false;
}

void RoleAssignerSvg::role_assigner_data_to_svg(const RoleAssignerInput& r_input,
                const RoleAssignerState& r_state,
                const RoleAssignerOutput& r_output,
                const RoleAssignerParameters& r_parameters,
                const std::string& save_name) 
{
    bool addInfoBox = true;
    double boxWidth = addInfoBox ? 6.0 : 0.0;

    if (save_name.empty()) {
        return;  // No outputfile required
    }

    FileParts parts = fileparts(save_name);
    if (parts.path.size() > 0) {
        if (!isDirectory(parts.path)) {
            MRA_LOG_ERROR("Not existing directory: \"%s\"", parts.path.c_str());
            return;
        }
    }

    m_environment = r_input.environment;

    std::vector<Vertex* > vertices = std::vector<Vertex* >();
    std::vector<MRA::Geometry::Point> parking_positions;
    std::vector<MRA::Geometry::Position> myTeam = {};
    for (auto idx = 0u; idx < r_input.team.size(); idx++) {
        myTeam.push_back(r_input.team[idx].position);
    }

    double totalFieldLength = r_input.environment.getFullFieldLength();
    double totalFieldWidth = r_input.environment.getFullFieldWidth();
    double halfRobotSize = r_input.environment.getRobotRadius();
    double robotSize = r_input.environment.getRobotSize();

    FILE* fp = fopen(save_name.c_str(), "w");
    // SVG header
    fprintf(fp, "<?xml version=\"1.0\" standalone=\"yes\"?>\n");
    fprintf(fp,
            "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n");
    fprintf(fp,
            "<svg width=\"%4.2fcm\" height=\"%4.2fcm\" version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\">\n",
            totalFieldWidth+boxWidth, totalFieldLength);
    fprintf(fp, "<desc>MSL field path</desc>\n");

    fprintf(fp, "<!-- \n\nfile: %s\n", save_name.c_str()); // start svg comment


    std::stringstream Xtext;

    fprintf(fp, "INPUT:\n%s\n", r_input.toString().c_str());
    fprintf(fp, "STATE:\n%s\n", r_state.toString().c_str());
    fprintf(fp, "PARAMS:\n%s\n",r_parameters.toString().c_str());
    fprintf(fp, "OUTPUT:\n%s\n", r_output.toString().c_str());


    // print input data to svg file
    fprintf(fp, "\n\n");
    fprintf(fp, "\tgamestate = %s (%d)\n", GameStateAsString(r_input.gamestate).c_str(), r_input.gamestate);
    // fprintf(fp, "\toriginal_gamestate = %s (%d)\n", GameStateAsString(data.original_gamestate).c_str(), data.original_gamestate);
    string controlBallByPlayerRemark = "";

    fprintf(fp, "\tball_status = \"%s\"\n", ballStatusAsString(r_input.ball.status).c_str());

    fprintf(fp, "\tBall: position: %s velocity: %s\n", r_input.ball.position.toString().c_str(), r_input.ball.velocity.toString().c_str());
    fprintf(fp, "\tprevious_ball = %s", boolToString(r_state.previous_ball.present).c_str());
    if (r_state.previous_ball.present) {
        fprintf(fp, " x=%4.2f y=%4.2f", r_state.previous_ball.x, r_state.previous_ball.y);
    }
    fprintf(fp, "\n");

    fprintf(fp, "\tpass_is_required: %s\n", boolToString(r_input.passIsRequired).c_str());

    fprintf(fp, "\tpass_data - valid %s\n", boolToString(r_input.pass_data.valid).c_str());
    if (r_input.pass_data.valid) {
        fprintf(fp, "\t PassData origin_x=%4.3f origin_y=%4.3f target_x=%4.3f target_y=%4.3f velocity=%4.3f angle= %4.3f ts=%4.3f/>\n",
                r_input.pass_data.origin_pos.x, r_input.pass_data.origin_pos.y, r_input.pass_data.target_pos.x, r_input.pass_data.target_pos.y,
                r_input.pass_data.velocity, r_input.pass_data.angle, r_input.pass_data.ts);
    }

    //fprintf(fp, "\tpassing player: %d\n", data.playerWhoIsPassing);
    fprintf(fp, "\tpickup ball valid: %s\n", boolToString(r_input.ball_pickup_position.valid).c_str());
    fprintf(fp, "\tpickup ball x: %4.2f\n", r_input.ball_pickup_position.x);
    fprintf(fp, "\tpickup ball y: %4.2f\n", r_input.ball_pickup_position.y);
    fprintf(fp, "\tpickup ball ts: %4.2f\n", r_input.ball_pickup_position.ts);
    fprintf(fp, "\n");


    fprintf(fp, "\tteam:\n");
    for (unsigned int idx = 0; idx < r_input.team.size(); idx++) {
        RoleAssignerRobot rbt = r_input.team[idx];
        fprintf(fp, "\t\tR%02ld = %s vel: %s trackingId: %ld type = %s (%d)\n",
                rbt.robotId, rbt.position.toString().c_str(), rbt.velocity.toString().c_str(),
                rbt.trackingId,
                PlayerTypeAsString(static_cast<player_type_e>(rbt.player_type)).c_str(),
                rbt.player_type );
        auto dr_role = RoleToDynamicRole(role_UNDEFINED, r_input.gamestate, r_input.ball.status);
        for (auto res_idx = 0u; res_idx < r_output.player_paths.size(); res_idx++) {
            if (r_output.player_paths[res_idx].robotId ==  rbt.robotId) {
                dr_role = RoleToDynamicRole(r_output.player_paths[res_idx].role, r_input.gamestate, r_input.ball.status);
            }
        }
        fprintf(fp, "\t\t\tcontrol-ball: %s passBall: %s role: %s time-own-PA: %4.2f time-opp-PA: %4.2f\n",
                boolToString(rbt.controlBall).c_str(), boolToString(rbt.passBall).c_str(), DynamicRoleAsString(dr_role).c_str(),
                rbt.time_in_own_penalty_area, rbt.time_in_opponent_penalty_area);
        bool prev_found = false;
        for (auto p_idx = 0u; p_idx < r_state.previous_results.size(); p_idx++) {
            auto prev_res = r_state.previous_results[p_idx];
            if (prev_res.robotId == rbt.robotId) {
                fprintf(fp, "\t\t\tprev result:  %s", boolToString(prev_res.present).c_str());
                if (prev_res.present)
                {
                    prev_found = true;
                    fprintf(fp, " robotId: %ld role: %s end-pos x: %4.2f y: %4.2f target: %s ts: %4.2f",
                            prev_res.robotId, RoleAsString(prev_res.role).c_str(), prev_res.end_position.x, prev_res.end_position.y,
                            PlannerTargetAsString(static_cast<planner_target_e>(prev_res.end_position.target)).c_str(), prev_res.ts);
                }
            }
        }
        if (not prev_found) {
                fprintf(fp, "\t\t\tprev result:  false");
        }
        fprintf(fp, "\n");
    }

    fprintf(fp, "\topponents:\n");
    for (unsigned int idx = 0; idx < r_input.opponents.size(); idx++) {
        fprintf(fp, "\t\tplayer[%u] = position %s velocity %s\n", idx,
                r_input.opponents[idx].position.toString().c_str(),
                r_input.opponents[idx].velocity.toString().c_str());
    }


    for (unsigned long p_idx = 0; p_idx < r_output.player_paths.size(); p_idx++) {
        auto player_path = r_output.player_paths[p_idx];
        long robotId = player_path.robotId;
        Xtext << std::fixed << std::setprecision(2) << endl<< "Player " << p_idx << " (id: " << robotId<< ") : " << std::endl;
        auto dr_role = RoleToDynamicRole(player_path.role, r_input.gamestate, r_input.ball.status);
        Xtext << "\trole: " << DynamicRoleAsString(dr_role) << endl;
        Xtext << "\tGame State: " << GameStateAsString(player_path.gamestate) << endl;
        Xtext << "\tTarget position : " << player_path.target.toString() << endl;
        Xtext << "\tPlanner target type  " << PlannerTargetAsString(player_path.planner_target) << endl;
        Xtext << "\tIs pass_desitination: " << boolToString(player_path.is_pass_desitination) << endl;
        Xtext << "\tDefend info valid:  " << boolToString(player_path.defend_info.valid);
        if (player_path.defend_info.valid) {
            Xtext << " defending_id: " << player_path.defend_info.defending_id;
            Xtext << " between_ball_and_defending_pos: " << boolToString(player_path.defend_info.between_ball_and_defending_pos);
            Xtext << " dist_from_defending_id: " << player_path.defend_info.dist_from_defending_id << endl;
        }
        else {
            Xtext << endl;
        }
        std::vector<path_piece_t> path;
        for (unsigned long idx = 0; idx < player_path.path.size(); idx++) {
            Xtext << std::fixed << setprecision(2) << "Path[" << p_idx << "]: x: " << player_path.path[idx].x <<
                    " y: " << player_path.path[idx].y <<
                    " cost: " << player_path.path[idx].cost;
            std::string targetString = PlannerTargetAsString(static_cast<planner_target_e>(player_path.path[idx].target));
            Xtext << " target: " << targetString << " (" << player_path.path[idx].target<< ")"<<  std::endl;
        }
    }

    fprintf(fp, "paths:\n%s\n", Xtext.str().c_str());

    fprintf(fp, "\tparameters:\n%s\n", r_parameters.toString().c_str());
    fprintf(fp, "\tfield:\n%s\n", r_input.environment.toString().c_str());
    fprintf(fp, "\n");

    // add xml output
    string decription = "xml from svg-file: " + save_name;
    string new_save_name = save_name;
    new_save_name.replace(new_save_name.find(".svg"), new_save_name.length(), "_NEW.svg");
    fprintf(fp, "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
    fprintf(fp, "<tns:Situation xmlns:tns=\"http://www.robotsports.nl\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"http://www.robotsports.nl ../StrategyTester.xsd\">\n");
    fprintf(fp, "<tns:Description>%s</tns:Description>\n", decription.c_str());
    fprintf(fp, "<tns:Options svgOutputFileName=\"%s_NEW\"\n", save_name.c_str());
    fprintf(fp, "   addBallApproachVertices=\"%s\"\n", boolToString(r_parameters.addBallApproachVertices).c_str());
    fprintf(fp, "   addBarierVertices=\"%s\"\n", boolToString(r_parameters.addBarierVertices).c_str());
    fprintf(fp, "   addUniformVertices=\"%s\"\n", boolToString(r_parameters.addBallApproachVertices).c_str());
    fprintf(fp, "   autoAssignGoalie=\"%s\"\n", boolToString(r_parameters.autoAssignGoalie).c_str());
    fprintf(fp, "   ballApproachNumberOfVertices=\"%d\"\n", r_parameters.ballApproachNumberOfVertices);
    fprintf(fp, "   ballApproachVerticesRadius=\"%4.3f\"\n", r_parameters.ballApproachVerticesRadius);
    fprintf(fp, "   calculateAllPaths=\"true\"\n"); // always calculate all paths
    fprintf(fp, "   dist_before_penalty_area_for_sweeper=\"%4.3f\"\n", r_parameters.dist_before_penalty_area_for_sweeper);
    fprintf(fp, "   dist_to_goal_to_mark_opponent_as_goalie=\"%4.3f\"\n", r_parameters.dist_to_goal_to_mark_opponent_as_goalie);
    fprintf(fp, "   distToapplyBallApproachVertices=\"%4.3f\"\n", r_parameters.distToapplyBallApproachVertices);
    fprintf(fp, "   firstCircleRadius=\"%4.3f\"\n", r_parameters.firstCircleRadius);
    fprintf(fp, "   grid_close_to_ball_normal_penalty=\"%4.3f\"\n", r_parameters.grid_close_to_ball_normal_penalty);
    fprintf(fp, "   grid_close_to_ball_normal_radius=\"%4.3f\"\n", r_parameters.grid_close_to_ball_normal_radius);
    fprintf(fp, "   grid_close_to_ball_restart_dropball_penalty=\"%4.3f\"\n", r_parameters.grid_close_to_ball_restart_dropball_penalty);
    fprintf(fp, "   grid_close_to_ball_restart_dropball_radius=\"%4.3f\"\n", r_parameters.grid_close_to_ball_restart_dropball_radius);
    fprintf(fp, "   grid_close_to_ball_restart_normal_penalty=\"%4.3f\"\n", r_parameters.grid_close_to_ball_restart_normal_penalty);
    fprintf(fp, "   grid_close_to_ball_restart_normal_radius=\"%4.3f\"\n", r_parameters.grid_close_to_ball_restart_normal_radius);
    fprintf(fp, "   grid_close_to_ball_restart_penalty_penalty=\"%4.3f\"\n", r_parameters.grid_close_to_ball_restart_penalty_penalty);
    fprintf(fp, "   grid_close_to_ball_restart_penalty_radius=\"%4.3f\"\n", r_parameters.grid_close_to_ball_restart_penalty_radius);
    fprintf(fp, "   grid_opponent_goal_clearance_x=\"%4.3f\"\n", r_parameters.grid_opponent_goal_clearance_x);
    fprintf(fp, "   grid_opponent_goal_clearance_y=\"%4.3f\"\n", r_parameters.grid_opponent_goal_clearance_y);
    fprintf(fp, "   grid_own_goal_clearance_x=\"%4.3f\"\n", r_parameters.grid_own_goal_clearance_x);
    fprintf(fp, "   grid_own_goal_clearance_y=\"%4.3f\"\n", r_parameters.grid_own_goal_clearance_y);
    fprintf(fp, "   grid_size=\"%4.3f\"\n", r_parameters.grid_size);
    fprintf(fp, "   interceptionChanceIncreasePerMeter=\"%4.3f\"\n", r_parameters.interceptionChanceIncreasePerMeter);
    fprintf(fp, "   interceptionChancePenaltyFactor=\"%4.3f\"\n", r_parameters.interceptionChancePenaltyFactor);
    fprintf(fp, "   interceptionChanceStartDistance=\"%4.3f\"\n", r_parameters.interceptionChanceStartDistance);
    fprintf(fp, "   interceptor_assign_min_velocity_for_calculate_interception_position=\"%4.3f\"\n", r_parameters.interceptor_assign_min_velocity_for_calculate_interception_position);
    fprintf(fp, "   interceptor_assign_use_ball_velocity=\"%s\"\n", boolToString(r_parameters.interceptor_assign_use_ball_velocity).c_str());
    fprintf(fp, "   man_to_man_defense_during_normal_play=\"%s\"\n", boolToString(r_parameters.man_to_man_defense_during_normal_play).c_str());
    fprintf(fp, "   man_to_man_defense_during_setplay_against=\"%s\"\n", boolToString(r_parameters.man_to_man_defense_during_setplay_against).c_str());
    fprintf(fp, "   manDefenseBetweenBallAndPlayer=\"%s\"\n", boolToString(r_parameters.manDefenseBetweenBallAndPlayer).c_str());
    fprintf(fp, "   maximumEdgeLength=\"%4.3f\"\n", r_parameters.maximumEdgeLength);
    fprintf(fp, "   maxPossibleLinearAcceleration=\"%4.3f\"\n", r_parameters.maxPossibleLinearAcceleration);
    fprintf(fp, "   maxPossibleLinearSpeed=\"%4.3f\"\n", r_parameters.maxPossibleLinearSpeed);
    fprintf(fp, "   minimumDistanceToEndPoint=\"%4.3f\"\n", r_parameters.minimumDistanceToEndPoint);
    fprintf(fp, "   minimumEdgeLength=\"%4.3f\"\n", r_parameters.minimumEdgeLength);
    fprintf(fp, "   move_to_ball_left_field_position=\"%s\"\n", boolToString(r_parameters.move_to_ball_left_field_position).c_str());
    fprintf(fp, "   nrDynamicPlannerIterations=\"%d\"\n", r_parameters.nrDynamicPlannerIterations);
    fprintf(fp, "   nrVerticesFirstCircle=\"%d\"\n", r_parameters.nrVerticesFirstCircle);
    fprintf(fp, "   nrVerticesSecondCircle=\"%d\"\n", r_parameters.nrVerticesSecondCircle);
    fprintf(fp, "   preferredSetplayKicker=\"%d\"\n", r_parameters.preferredSetplayKicker);
    fprintf(fp, "   preferredSetplayReceiver=\"%d\"\n", r_parameters.preferredSetplayReceiver);
    fprintf(fp, "   previous_role_end_pos_threshold=\"%4.3f\"\n", r_parameters.previous_role_end_pos_threshold);
    fprintf(fp, "   previous_role_bonus_end_pos_radius=\"%4.3f\"\n", r_parameters.previous_role_bonus_end_pos_radius);
    fprintf(fp, "   priority_block_min_distance=\"%4.3f\"\n", r_parameters.priority_block_min_distance);
    fprintf(fp, "   priority_block_max_distance=\"%4.3f\"\n", r_parameters.priority_block_max_distance);
    fprintf(fp, "   safetyFactor=\"%4.3f\"\n", r_parameters.safetyFactor);
    fprintf(fp, "   secondCircleRadius=\"%4.3f\"\n", r_parameters.secondCircleRadius);
    fprintf(fp, "   setplay_against_dist_to_opponent=\"%4.3f\"\n", r_parameters.setplay_against_dist_to_opponent);
    fprintf(fp, "   startingVelocityPenaltyFactor=\"%4.3f\"\n", r_parameters.startingVelocityPenaltyFactor);
    fprintf(fp, "   svgBallColor=\"orange\"\n");
    fprintf(fp, "   svgDefaultTargetColor=\"red\"\n");
    fprintf(fp, "   svgDrawEdges=\"false\"\n");
    fprintf(fp, "   svgDrawVelocity=\"true\"\n");
    fprintf(fp, "   svgOriginalTargetColor=\"darkred\"\n");
    fprintf(fp, "   svgOpponentColor=\"cyan\"\n");
    fprintf(fp, "   svgTeamColor=\"magenta\"\n");
    fprintf(fp, "   saveGridDataToFile=\"%s\"\n", boolToString(r_parameters.saveGridDataToFile).c_str());
    fprintf(fp, "   svgRobotPlanner=\"false\"\n");
    fprintf(fp, "   uniform_x_interval=\"%4.3f\"\n", r_parameters.uniform_x_interval);
    fprintf(fp, "   uniform_y_interval=\"%4.3f\"\n", r_parameters.uniform_y_interval);
    fprintf(fp, "   use_pass_to_position_for_attack_support=\"%s\"\n", boolToString(r_parameters.use_pass_to_position_for_attack_support).c_str());
    fprintf(fp, "   wait_on_non_optimal_position_during_prepare_phase=\"%s\"\n", boolToString(r_parameters.wait_on_non_optimal_position_during_prepare_phase).c_str());
    fprintf(fp, "/>\n");
    fprintf(fp, "  <tns:Field field_width=\"%4.3f\" field_length=\"%4.3f\" field_margin=\"%4.2f\" goal_width=\"%4.3f\" goal_length=\"%4.3f\" goal_area_width=\"%4.3f\" goal_area_length=\"%4.3f\"\n"
                "             penalty_area_present=\"%s\" penalty_area_width=\"%4.3f\" penalty_area_length=\"%4.3f\" center_circle_diameter=\"%4.3f\" robot_size=\"%4.3f\"\n"
                "             ball_radius=\"%4.3f\" field_markings_width=\"%4.3f\" parking_area_width=\"%4.3f\" parking_area_length=\"%4.3f\" parking_distance_between_robots=\"%4.3f\"\n"
                "             parking_distance_to_line=\"%4.3f\" corner_circle_diameter=\"%4.3f\" penalty_spot_to_backline=\"%4.3f\" />\n",
                r_input.environment.getFieldWidth(),r_input.environment.getFieldLength(), r_input.environment.getFieldMargin(), r_input.environment.getGoalWidth(), r_input.environment.getGoalLength(), r_input.environment.getGoalAreaWidth(), r_input.environment.getGoalAreaLength(),
        boolToString(r_input.environment.isPenaltyAreaPresent()).c_str(), r_input.environment.getPenaltyAreaWidth(), r_input.environment.getPenaltyAreaLength(), r_input.environment.getCenterCirleDiameter(), r_input.environment.getRobotSize(),
        r_input.environment.getBallRadius(), r_input.environment.getFieldMarkingsWidth(), r_input.environment.getParkingAreaWidth(), r_input.environment.getParkingAreaLength(), r_input.environment.getParkingDistanceBetweenPlayers(), r_input.environment.getParkingDistanceToLine(),
        r_input.environment.getCornerCircleDiameter(), r_input.environment.getPenaltySpotToBackline());
    fprintf(fp, "  <tns:GameState>%s</tns:GameState>\n", GameStateAsString(r_input.gamestate).c_str());
    if (r_input.ball.is_valid) {
        fprintf(fp, "  <tns:Ball x=\"%4.2f\" y=\"%4.2f\" velx=\"%4.2f\" vely=\"%4.2f\"/>\n",
                r_input.ball.position.x, r_input.ball.position.y, r_input.ball.velocity.x, r_input.ball.velocity.y);
    }
    if (r_state.previous_ball.present) {
        fprintf(fp, "  <tns:PreviousBall x=\"%4.2f\" y=\"%4.2f\"/>\n", r_state.previous_ball.x, r_state.previous_ball.y);
     }
    for (unsigned int idx = 0; idx < r_input.team.size(); idx++) {
        string goalieString = "";
        if (r_input.team[idx].player_type == GOALIE) {
            goalieString = " isGoalie=\"true\" ";
        }

        string idString = "id=\""+ std::to_string(r_input.team[idx].robotId) + "\"";

        // set trackingId for this player: use tracking if define (> 0), otherwise make tracking-id same as robot-id.
        auto tracking_id = r_input.team[idx].trackingId;
        if (tracking_id <= 0) {
            tracking_id = r_input.team[idx].robotId;
        }
        string trackingString = "tracking=\""+ std::to_string(tracking_id) + "\"";

        string controlBallString = "";
        if (r_input.team[idx].controlBall)
        {
            controlBallString = " controlBall=\"true\" ";
        }
        string passedBallString = "";
        string previous_result_string = "";
        if (r_input.team[idx].passBall)
        {
            passedBallString = "passedBall=\"true\"";
        }
        for (auto p_idx = 0u; p_idx < r_state.previous_results.size(); p_idx++) {
            auto prev_res = r_state.previous_results[p_idx];
            if (prev_res.robotId == r_input.team[idx].robotId) {
                std::stringstream previous_result_Xtext;
                previous_result_Xtext << " previous_result_present=\"true\" "
                        <<" previous_result_ts=\""  << prev_res.ts << "\""
                        <<" previous_result_x=\""   << prev_res.end_position.x << "\""
                        <<" previous_result_y=\""   << prev_res.end_position.y << "\""
                        <<" previous_result_role=\""
                        << RoleAsString(prev_res.role)
                        <<"\"";

                previous_result_string = previous_result_Xtext.str();;

            }
        }
        fprintf(fp, "  <tns:Team %s %s x=\"%4.3f\" y=\"%4.3f\" rz=\"%4.3f\" velx=\"%4.3f\" vely=\"%4.3f\" velrz=\"%4.3f\" %s %s %s %s/>\n",
                idString.c_str(), trackingString.c_str(),
                r_input.team[idx].position.x, r_input.team[idx].position.y, r_input.team[idx].position.rz,
                r_input.team[idx].velocity.x, r_input.team[idx].velocity.y, r_input.team[idx].velocity.rz,
                goalieString.c_str(), controlBallString.c_str(), passedBallString.c_str(), previous_result_string.c_str());

    }
    for (unsigned int idx = 0; idx < r_input.opponents.size(); idx++) {
        string idString = "id=\""+ std::to_string(idx+1) + "\"";
        string trackingString = "trackingId=\""+ std::to_string(r_input.opponents[idx].trackingId) + "\"";
        fprintf(fp, "  <tns:Opponent %s %s x=\"%4.3f\" y=\"%4.3f\" rz=\"%4.3f\" velx=\"%4.3f\" vely=\"%4.3f\" velrz=\"%4.3f\" />\n",
                idString.c_str(), trackingString.c_str(),
                r_input.opponents[idx].position.x, r_input.opponents[idx].position.y, r_input.opponents[idx].position.rz,
                r_input.opponents[idx].velocity.x, r_input.opponents[idx].velocity.y, r_input.opponents[idx].velocity.rz);

    }
    // write pickup ball info as svg input if possible
    string validString = "valid=\"false\"";
    if (r_input.ball_pickup_position.valid) {
        validString = "valid=\"true\"";
    }

    fprintf(fp, "  <tns:PickupPosition x=\"%4.3f\" y=\"%4.3f\" ts=\"%4.3f\" %s />\n",
            r_input.ball_pickup_position.x, r_input.ball_pickup_position.y, r_input.ball_pickup_position.ts, validString.c_str());


    // write situation info as svg input if possible
    string passRequiredString = "passing_required=\"false\"";
    if (r_input.passIsRequired)
    {
        passRequiredString = "passing_required=\"true\"";
    }

    string ball_statusString = "ball_status=\"" + ballStatusAsString(r_input.ball.status) + "\"";
    fprintf(fp, "  <tns:SituationInfo %s %s>\n", passRequiredString.c_str(), ball_statusString.c_str());
    string pass_data_valid_str = "valid=\"false\"";
    if (r_input.pass_data.valid)
    {
        pass_data_valid_str = "valid=\"true\"";
    }
    fprintf(fp, "    <tns:PassData %s origin_x=\"%4.3f\" origin_y=\"%4.3f\" target_x=\"%4.3f\" target_y=\"%4.3f\" velocity=\"%4.3f\" angle=\"%4.3f\" ts=\"%4.3f\" target_id=\"%ld\"/>\n",
            pass_data_valid_str.c_str(), r_input.pass_data.origin_pos.x, r_input.pass_data.origin_pos.y, 
            r_input.pass_data.target_pos.x, r_input.pass_data.target_pos.y, 
            r_input.pass_data.velocity, r_input.pass_data.angle, r_input.pass_data.ts, r_input.pass_data.target_id);
    fprintf(fp, "  </tns:SituationInfo>\n");
    fprintf(fp, "  <tns:ParkingInfo>\n");
    for (auto idx = 0u; idx < r_input.parking_positions.size(); idx++) {
        fprintf(fp, "    <tns:ParkingPosition x=\"%4.2f\" y=\"%4.2f\"/>\n", r_input.parking_positions[idx].x, r_input.parking_positions[idx].y);
    }
    fprintf(fp, "  </tns:ParkingInfo>\n");
    fprintf(fp, "</tns:Situation>\n");

    fprintf(fp, "\n-->\n"); // // end svg comment


    //FIELD - total green field
    fprintf(fp,
            "<rect x=\"0cm\" y=\"0cm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"green\" stroke-width=\"0.02cm\" />\n",
            totalFieldWidth, totalFieldLength);

    if (addInfoBox) {
        fprintf(fp,
                "<rect x=\"%4.2fcm\" y=\"0cm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"white\" stroke-width=\"0.02cm\" />\n",
                totalFieldWidth, boxWidth, totalFieldLength);

        double boxTextOffset = 0.1;
        double boxLineStart = 1.0;
        unsigned boxTextSize = 14;
        double boxLineOffset = 0.5;
        double boxLineIndent = 0.2;
        const int LINES_PER_ROBOT = 4;

        auto line_offset = boxLineStart;
        fprintf(fp,"<text x=\"%4.2fcm\" y=\"%4.2fcm\" font-size=\"%u\" font-weight-absolute=\"bold\" fill=\"black\">"
                            "Game-state: %s</text>\n",
                            totalFieldWidth+boxTextOffset, line_offset, boxTextSize,
                            GameStateAsString(r_input.gamestate).c_str());

        line_offset += boxLineOffset;
        std::string ball_status = ballStatusAsString(r_input.ball.status);
        toLower(ball_status);
        fprintf(fp,"<text x=\"%4.2fcm\" y=\"%4.2fcm\" font-size=\"%u\" font-weight-absolute=\"bold\" fill=\"black\">"
                            "Ball status: %s</text>\n",
                            totalFieldWidth+boxTextOffset, line_offset, boxTextSize,
                            ball_status.c_str());

        line_offset += boxLineOffset;
        line_offset += boxLineOffset;
        line_offset += boxLineOffset;
        line_offset += boxLineOffset;
        auto robot_block_start = line_offset;
        for (unsigned int idx = 0; idx < r_output.player_paths.size(); idx++) {
            auto player_path = r_output.player_paths[idx];
            auto rbt = r_input.team[idx];
            auto dr_role = RoleToDynamicRole(player_path.role, r_input.gamestate, r_input.ball.status);
            fprintf(fp,"<text x=\"%4.2fcm\" y=\"%4.2fcm\" font-size=\"%u\" font-weight-absolute=\"bold\" fill=\"black\">"
                    "R%02ld: %s</text>\n",
                    totalFieldWidth+boxTextOffset, robot_block_start + (idx*LINES_PER_ROBOT) * boxLineOffset, boxTextSize,
                    rbt.robotId, DynamicRoleAsString(dr_role).c_str());

            fprintf(fp,"<text x=\"%4.2fcm\" y=\"%4.2fcm\" font-size=\"%u\" font-weight-absolute=\"bold\" fill=\"black\">"
                    "target-pos: %s</text>\n",
                    totalFieldWidth+boxTextOffset+boxLineIndent, robot_block_start + (idx*LINES_PER_ROBOT+1) * boxLineOffset, boxTextSize,
                    player_path.target.toString().c_str()
                    );

            string costs_str = "unknown";
            if (player_path.path.size() > 0) {
                costs_str = std::to_string(player_path.path[player_path.path.size()-1].cost);
            }
            fprintf(fp,"<text x=\"%4.2fcm\" y=\"%4.2fcm\" font-size=\"%u\" font-weight-absolute=\"bold\" fill=\"black\">"
                    "role-rank: %d costs: %s</text>\n",
                    totalFieldWidth+boxTextOffset+boxLineIndent, robot_block_start + (idx*LINES_PER_ROBOT+2) * boxLineOffset, boxTextSize,
                    player_path.role_rank, costs_str.c_str());

            // empty line(s) added if LINES_PER_ROBOT > written lines
        }
    }

    //FIELD - outer field lines
    fprintf(fp,
            "<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"none\" stroke=\"white\" stroke-width=\"0.125cm\"/>",
            r_input.environment.getFieldMargin(), r_input.environment.getFieldMargin(), r_input.environment.getFieldWidth(), r_input.environment.getFieldLength());
    //FIELD - middle line
    fprintf(fp,
            "<line x1=\"%4.2fcm\" y1=\"%4.2fcm\" x2=\"%4.2fcm\" y2=\"%4.2fcm\" stroke-width=\"0.125cm\"  stroke=\"white\"/>\n",
            r_input.environment.getFieldMargin(), r_input.environment.getMaxFieldY()+r_input.environment.getFieldMargin(), r_input.environment.getFieldWidth()+r_input.environment.getFieldMargin(), r_input.environment.getMaxFieldY()+r_input.environment.getFieldMargin());
    //FIELD - middle circle
    fprintf(fp,
            "<circle cx=\"%4.2fcm\" cy=\"%4.2fcm\" r=\"%4.2fcm\" fill=\"none\" stroke=\"white\" stroke-width=\"0.125cm\"  />\n",
            totalFieldWidth*0.5,  totalFieldLength*0.5, r_input.environment.getCenterCirleDiameter()*0.5);

    // PENALTY AREAS
    fprintf(fp,
            "<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"green\" stroke=\"white\" stroke-width=\"0.125cm\"/>\n",
            (totalFieldWidth*0.5)-(r_input.environment.getPenaltyAreaWidth()*0.5), r_input.environment.getFieldMargin(), r_input.environment.getPenaltyAreaWidth(), r_input.environment.getPenaltyAreaLength());

    fprintf(fp,
            "<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"green\" stroke=\"white\" stroke-width=\"0.125cm\"/>\n",
            (totalFieldWidth*0.5)-(r_input.environment.getPenaltyAreaWidth()*0.5), r_input.environment.getFieldMargin()+r_input.environment.getFieldLength()-r_input.environment.getPenaltyAreaLength(), r_input.environment.getPenaltyAreaWidth(), r_input.environment.getPenaltyAreaLength());

    // GOAL AREAS
    fprintf(fp,
            "<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"green\" stroke=\"white\" stroke-width=\"0.125cm\"/>\n",
            (totalFieldWidth*0.5)-(r_input.environment.getGoalAreaWidth()*0.5), r_input.environment.getFieldMargin(), r_input.environment.getGoalAreaWidth(), r_input.environment.getGoalAreaLength());

    fprintf(fp,
            "<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"green\" stroke=\"white\" stroke-width=\"0.125cm\"/>\n",
            (totalFieldWidth*0.5)-(r_input.environment.getGoalAreaWidth()*0.5), r_input.environment.getFieldMargin()+r_input.environment.getFieldLength()-r_input.environment.getGoalAreaLength(), r_input.environment.getGoalAreaWidth(), r_input.environment.getGoalAreaLength());

    //FIELD - goals
    fprintf(fp,
            "<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"plum\" stroke=\"white\" stroke-width=\"0.125cm\"/>\n",
            (totalFieldWidth*0.5)-(r_input.environment.getGoalWidth()*0.5), r_input.environment.getFieldMargin()-r_input.environment.getGoalLength(), r_input.environment.getGoalWidth(), r_input.environment.getGoalLength());
    fprintf(fp,
            "<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\"  fill=\"powderblue\" stroke=\"white\" stroke-width=\"0.125cm\"/>\n",
            (totalFieldWidth*0.5)-(r_input.environment.getGoalWidth()*0.5), r_input.environment.getFieldMargin()+r_input.environment.getFieldLength(), r_input.environment.getGoalWidth(), r_input.environment.getGoalLength());
    // indicate own goal with text own goal
    fprintf(fp,"<text x=\"%4.2fcm\" y=\"%4.2fcm\" fill=\"darkred\">OWN</text>",
            (totalFieldWidth*0.5)-(r_input.environment.getGoalWidth()*0.2),
            r_input.environment.getFieldMargin()+r_input.environment.getFieldLength()+(r_input.environment.getGoalLength()*0.75));

    //vertices
    for (auto j = 0u; j < vertices.size(); j++) {
        Vertex* v = vertices[j];

        fprintf(fp,
                "<circle cx=\"%4.2fcm\" cy=\"%4.2fcm\" r=\"%4.2fcm\" fill=\"orange\" stroke=\"orange\" stroke-width=\"0.125cm\"  />\n",
                svgX(v->m_coordinate.x),  svgY(v->m_coordinate.y), 0.01);
        if (r_parameters.svgDrawEdges) {
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
    for(auto bar_idx = 0u; bar_idx != r_input.opponents.size(); bar_idx++) {
        Geometry::Position bar_pos = r_input.opponents[bar_idx].position;
        fprintf(fp,
                "\n<!-- Opponent -->\n<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"%s\" stroke=\"%s\" stroke-width=\"0.125cm\"/>\n",
                svgX(bar_pos.x - halfRobotSize), svgY(bar_pos.y + halfRobotSize), robotSize, robotSize, 
                r_parameters.svgOpponentColor.c_str(), r_parameters.svgOpponentColor.c_str());
        fprintf(fp,"<text x=\"%4.2fcm\" y=\"%4.2fcm\" font-size=\"large\" font-weight-absolute=\"bold\" fill=\"black\">%u</text>",
                svgX(bar_pos.x- 0.65*halfRobotSize), svgY(bar_pos.y- 0.65*halfRobotSize), bar_idx+1);
    }

    // TEAMMATES
    // draw me first.
    if (r_input.team.size() > 0 ) {
        int this_player_idx = 0; // ME robot is first player
        Geometry::Position bar_pos = r_input.team[this_player_idx].position;
        double r = r_input.team[this_player_idx].position.rz + M_PI_2;
        string teamColor = r_parameters.svgTeamColor;
        string fillColor = r_parameters.svgTeamColor;
        fprintf(fp,
                "\n<!-- ME -->\n<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"%s\" stroke=\"%s\" stroke-width=\"0.125cm\"/>\n",
                svgX(bar_pos.x - halfRobotSize), svgY(bar_pos.y + halfRobotSize), robotSize, robotSize, teamColor.c_str(), fillColor.c_str());
        if (r_input.team[this_player_idx].controlBall) {
            fprintf(fp,
                    "\n<!-- aiming -->\n<line x1=\"%4.2fcm\" y1=\"%4.2fcm\" x2=\"%4.2fcm\" y2=\"%4.2fcm\" stroke-width=\"0.05cm\"  stroke=\"yellow\"/>\n",
                    svgX(bar_pos.x), svgY(bar_pos.y), svgX(bar_pos.x + 12 * cos(r)), svgY(bar_pos.y + 12 * sin(r)));
        }
    }
    for(auto bar_idx = 1u; bar_idx < r_input.team.size(); bar_idx++) {
        Geometry::Point bar_pos = r_input.team[bar_idx].position;
        fprintf(fp,
                "\n<!-- Teammate -->\n<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"%s\" stroke=\"%s\" stroke-width=\"0.125cm\"/>\n",
                svgX(bar_pos.x - halfRobotSize), svgY(bar_pos.y + halfRobotSize), robotSize, robotSize, r_parameters.svgTeamColor.c_str(), r_parameters.svgTeamColor.c_str());
    }

    string last_path_element_color = "yellow";
    string path_color = "blue";
    string additional_parameters = "";
    // draw path
    for (auto pidx = 0u;  pidx < r_output.player_paths.size(); pidx++) {
        auto player_path = r_output.player_paths[pidx];
        string fillColor = r_parameters.svgTeamColor;
        if (player_path.path.size() > 0) {
            fprintf(fp,
                    "\n<!-- player-path start %d-->\n<rect x=\"%4.2fcm\" y=\"%4.2fcm\" width=\"%4.2fcm\" height=\"%4.2fcm\" fill=\"%s\" stroke=\"%s\" stroke-width=\"0.125cm\"/>\n",
                    (int)pidx, svgX(player_path.path[0].x - halfRobotSize), svgY(player_path.path[0].y + halfRobotSize), robotSize, robotSize,
                    r_parameters.svgTeamColor.c_str(), fillColor.c_str());
        }
        for (auto j = 1u; j < player_path.path.size(); j++) {
            double prev_x = (player_path).path[j-1].x;
            double prev_y = (player_path).path[j-1].y;
            double new_x = (player_path).path[j].x;
            double new_y = (player_path).path[j].y;
            if (j == 1) {
                // start first path piece not in middle of player but near the edge of the player
                Geometry::Point prev_pos(prev_x, prev_y);
                Geometry::Point new_pos(new_x, new_y);
                double alfa = prev_pos.angle(new_pos);
                prev_x = prev_x - (cos(alfa) * halfRobotSize);
                prev_y = prev_y - (sin(alfa) * halfRobotSize);

            }
            fprintf(fp,
                    "\n<!-- path player %d-->\n<line x1=\"%4.2fcm\" y1=\"%4.2fcm\" x2=\"%4.2fcm\" y2=\"%4.2fcm\" stroke-width=\"0.125cm\"  stroke=\"%s\" %s/>\n",
                    (int)pidx,svgX(prev_x),  svgY(prev_y), svgX(new_x), svgY(new_y), path_color.c_str(), additional_parameters.c_str());
            if (j == player_path.path.size()-1) {
                // last element
                fprintf(fp,
                        "\n<!-- path-end player %d-->\n<circle cx=\"%4.2fcm\" cy=\"%4.2fcm\" r=\"%4.2fcm\" fill=\"%s\" stroke=\"%s\" stroke-width=\"0.125cm\"  />\n",
                        (int)pidx, svgX((player_path.path[j]).x),  svgY((player_path.path[j]).y), r_input.environment.getBallRadius()*1.5,
                        last_path_element_color.c_str() /* fill color */, last_path_element_color.c_str() /* line color */); // half ball diameter

            }
        }
        //        // TARGET: draw ball if first path piece is goto_ball, otherwise a red circle
        //        string target_color = parameters.svgDefaultTargetColor;
        //        if ((player_paths[pidx].path.size() > 0) && static_cast<planner_target_e>(player_paths[pidx].path[0].target) == planner_target_e::GOTO_BALL) {
        //            target_color = parameters.svgBallColor;
        //        }
        //
        //        // draw orange circle (ball) as target
        //        for(std::vector<Vertex>::size_type idx = 0; idx != m_target.size(); idx++) {
        //            fprintf(fp,
        //                    "<circle cx=\"%4.2fcm\" cy=\"%4.2fcm\" r=\"%4.2fcm\" fill=\"%s\" stroke=\"%s\" stroke-width=\"0.125cm\"  />\n",
        //                    svgX(m_target[idx]->m_coordinate.x), svgY(m_target[idx]->m_coordinate.y), r_input.environment.getBallRadius(), target_color.c_str(), target_color.c_str());
        //        }

    }

    string compare_last_path_element_color = "yellow";
    string compare_path_color = "blue";

    // put player-id on top of the players 
    for(auto bar_idx = 0u; bar_idx < r_input.team.size(); bar_idx++) {
        Geometry::Point bar_pos = r_input.team[bar_idx].position;
        long robotId = r_input.team[bar_idx].robotId;
        fprintf(fp,"<text x=\"%4.2fcm\" y=\"%4.2fcm\" font-size=\"large\" font-weight-absolute=\"bold\" fill=\"black\">%lu</text>", svgX(bar_pos.x - 0.65*halfRobotSize), svgY(bar_pos.y - 0.65*halfRobotSize), robotId);
    }

    // BALL
    if (r_input.ball.is_valid) {
        fprintf(fp,
                "\n<!-- Ball -->\n<circle cx=\"%4.2fcm\" cy=\"%4.2fcm\" r=\"%4.2fcm\" fill=\"orange\" stroke=\"orange\" stroke-width=\"0.125cm\"  />\n",
                svgX(r_input.ball.position.x),  svgY(r_input.ball.position.y), r_input.environment.getBallRadius()); // half ball diameter
    }
    if (r_parameters.svgDrawVelocity) {
        Geometry::Point linVel(r_input.ball.velocity.x, r_input.ball.velocity.y);
        double speed  = linVel.size();
        if (speed > 1e-6) {
            Geometry::Point endVelocityVector(r_input.ball.position.x, r_input.ball.position.y);
            endVelocityVector += linVel;
            //FIELD - middle line
            fprintf(fp,
                    "\n<!-- velocity line -->\n<line x1=\"%4.2fcm\" y1=\"%4.2fcm\" x2=\"%4.2fcm\" y2=\"%4.2fcm\" stroke-width=\"0.125cm\"  stroke=\"red\"/>\n",
                    svgX(r_input.ball.position.x), svgY(r_input.ball.position.y),
                    svgX(endVelocityVector.x), svgY(endVelocityVector.y));
        }
    }

    fprintf(fp, "</svg>\n");
    fclose(fp);
    MRA_LOG_INFO("created SVG file : %s", save_name.c_str());
}


/**
 * return true if path is a directory, otherwise return false
*/
bool RoleAssignerSvg::doesDirectoryExists(const std::string& filename) {
    // split filename in fileparts (compared to matlab fileparts function)
    // split in path, name and extension
    bool is_dir = false;
    int idx0 = filename.rfind("/");

    std::string path = filename.substr(0,idx0+1);
    if (path.size() > 0) {
        struct stat info;

        if(stat( path.c_str(), &info ) == 0) {
            if(info.st_mode & S_IFDIR) {
                is_dir = true;
            }
        }
        else {
            MRA_LOG_INFO("Not existing directory: \"%s\"", path.c_str());
        }
    }
    return is_dir;
}

std::string RoleAssignerSvg::boolToString(bool b)
{
    return b ? "true" : "false";
}


void RoleAssignerSvg::toLower(std::string& r_string) {
    std::locale loc;
    for (auto i=0u; i < r_string.length(); ++i) {
        r_string[i] = std::tolower(r_string[i], loc);
    }
}
