/**
 *  @file
 *  @brief    Options for generating the visibility graph and the path planner
 *  @curator Jürge van Eijck
 */
#include "TeamPlannerParameters.hpp"

#include <ostream>
#include <sstream>
#include <iostream>
#include "WmTypes.h"

using namespace std;

namespace MRA {

bool TeamPlannerParameters::calculateAllPaths = true;
double TeamPlannerParameters::minimumEdgeLength = 0.30;
double TeamPlannerParameters::maximumEdgeLength = 4.00;
double TeamPlannerParameters::minimumDistanceToEndPoint = 3.00;
int TeamPlannerParameters::nrVerticesFirstCircle = 6;
double TeamPlannerParameters::firstCircleRadius = 0.80;
int TeamPlannerParameters::nrVerticesSecondCircle = 4;
double TeamPlannerParameters::secondCircleRadius = 1.60;
double TeamPlannerParameters::safetyFactor = 1.00;
bool TeamPlannerParameters::addBarierVertices = true;
bool TeamPlannerParameters::addUniformVertices = true;
bool TeamPlannerParameters::manDefenseBetweenBallAndPlayer = true;
double TeamPlannerParameters::dist_before_penalty_area_for_sweeper = 0.75;
double TeamPlannerParameters::uniform_x_interval = 2.00;
double TeamPlannerParameters::uniform_y_interval = 2.00;
double TeamPlannerParameters::startingVelocityPenaltyFactor = 1.00;
double TeamPlannerParameters::distToapplyBallApproachVertices = 10.00;
bool TeamPlannerParameters::addBallApproachVertices = false;
double TeamPlannerParameters::ballApproachVerticesRadius = 0.6;
int TeamPlannerParameters::ballApproachNumberOfVertices = 8;
double TeamPlannerParameters::grid_size = 0.75;
int TeamPlannerParameters::nrDynamicPlannerIterations = 2;
double TeamPlannerParameters::maxPossibleLinearSpeed = 1.5;
double TeamPlannerParameters::maxPossibleLinearAcceleration = 3.0;
std::string TeamPlannerParameters::svgOutputFileName = "";
std::string TeamPlannerParameters::svgDefaultTargetColor = "red";
std::string TeamPlannerParameters::svgBallColor = "orange";
std::string TeamPlannerParameters::svgOriginalTargetColor = "darkred";
std::string TeamPlannerParameters::svgTeamColor = "magenta";
std::string TeamPlannerParameters::svgOpponentColor = "cyan";
bool TeamPlannerParameters::svgDrawVelocity = true;
bool TeamPlannerParameters::svgDrawEdges = false;
bool TeamPlannerParameters::saveGridDataToFile = false;
bool TeamPlannerParameters::svgRobotPlanner = false;
double TeamPlannerParameters::interceptionChanceStartDistance = 1.0;
double TeamPlannerParameters::interceptionChanceIncreasePerMeter = 0.25;
double TeamPlannerParameters::interceptionChancePenaltyFactor = 4.0;
double TeamPlannerParameters::grid_close_to_ball_normal_penalty = 17000;
double TeamPlannerParameters::grid_close_to_ball_normal_radius = 3.0;
double TeamPlannerParameters::grid_close_to_ball_restart_normal_penalty = 17000;
double TeamPlannerParameters::grid_close_to_ball_restart_normal_radius = 3.0; // default the standard MSL rules, can be overwritten in ini file
double TeamPlannerParameters::grid_close_to_ball_restart_penalty_penalty = 17000;
double TeamPlannerParameters::grid_close_to_ball_restart_penalty_radius = 3.25;
double TeamPlannerParameters::grid_close_to_ball_restart_dropball_penalty = 17000;
double TeamPlannerParameters::grid_close_to_ball_restart_dropball_radius = 1.0; // default the standard MSL rules, can be overwritten in ini file
double TeamPlannerParameters::grid_opponent_goal_clearance_x = 7.0;
double TeamPlannerParameters::grid_opponent_goal_clearance_y = 2.5;
double TeamPlannerParameters::grid_own_goal_clearance_x = 3.25;
double TeamPlannerParameters::grid_own_goal_clearance_y = 1.00;
int  TeamPlannerParameters::nr_robots_needed_for_pass_play = 2;
int  TeamPlannerParameters::nr_attack_support_during_defensive_period = 0;
bool TeamPlannerParameters::wait_on_non_optimal_position_during_prepare_phase = false;
double TeamPlannerParameters::priority_block_min_distance = 0.5;
double TeamPlannerParameters::priority_block_max_distance = 2.0;
double TeamPlannerParameters::priority_block_max_distance_to_defense_line = 0.25;
double TeamPlannerParameters::priority_block_max_ball_y = 0.0;
double TeamPlannerParameters::priority_block_max_opponent_to_ball_dist = 3.0;
bool TeamPlannerParameters::priority_block_check_ball_in_area = true;
bool TeamPlannerParameters::priority_block_check_opponent_close_to_ball = true;
bool TeamPlannerParameters::man_to_man_defense_during_normal_play = true;
double TeamPlannerParameters::attack_supporter_extra_distance_to_stay_from_sideline = 0.75;
double TeamPlannerParameters::auto_save_svg_period = 10.0;
double TeamPlannerParameters::restart_receiver_ball_dist = 2.5;
double TeamPlannerParameters::restart_shooter_ball_dist = 0.8;
double TeamPlannerParameters::equality_cost_threshold = 1.5;
bool TeamPlannerParameters::previous_role_bonus_must_be_applied = true;
double TeamPlannerParameters::previous_role_bonus_end_pos_radius = 1.5;
bool TeamPlannerParameters::use_pass_to_position_for_attack_support = true;
bool TeamPlannerParameters::man_to_man_defense_during_setplay_against = true;
double  TeamPlannerParameters::dist_to_goal_to_mark_opponent_as_goalie = 1.5;
double TeamPlannerParameters::setplay_against_dist_to_opponent = 1.5;

bool TeamPlannerParameters::move_to_ball_left_field_position = true;
bool TeamPlannerParameters::select_lowest_robot_nr_for_dynamic_role = true;
int TeamPlannerParameters::preferredSetplayKicker = 0;
int TeamPlannerParameters::preferredSetplayReceiver = 0;
double TeamPlannerParameters::setplay_margin_to_penalty_area_side = 0.75;
bool TeamPlannerParameters::no_sweeper_during_setplay = true;
bool TeamPlannerParameters::interceptor_assign_use_ball_velocity = true;
double TeamPlannerParameters::interceptor_assign_min_velocity_for_calculate_interception_position = 0.5;

int TeamPlannerParameters::dedicatedSweeper = 0;
bool TeamPlannerParameters::autoAssignGoalie = false;
double TeamPlannerParameters::outsideFieldMargin = 0.5;
bool TeamPlannerParameters::lobShotWhenPossible = true;
double TeamPlannerParameters::min_y_for_lob_shot = +0.25;
double TeamPlannerParameters::kickoff_fp1_x = -1.00;
double TeamPlannerParameters::kickoff_fp1_y =  0.00;
double TeamPlannerParameters::kickoff_fp2_x = +1.70;
double TeamPlannerParameters::kickoff_fp2_y = -1.50;
double TeamPlannerParameters::kickoff_fp3_x = -5.00;
double TeamPlannerParameters::kickoff_fp3_y = -0.75;
double TeamPlannerParameters::kickoff_fp4_x = +5.00;
double TeamPlannerParameters::kickoff_fp4_y = -0.75;


double TeamPlannerParameters::kickoff_against_fp1_x = 1.87;
double TeamPlannerParameters::kickoff_against_fp1_y = -1.87;
double TeamPlannerParameters::kickoff_against_fp2_x = -1.87;
double TeamPlannerParameters::kickoff_against_fp2_y = -1.87;
double TeamPlannerParameters::kickoff_against_fp3_x = 3.5;
double TeamPlannerParameters::kickoff_against_fp3_y = -0.4;
double TeamPlannerParameters::kickoff_against_fp4_x = -3.5;
double TeamPlannerParameters::kickoff_against_fp4_y = -0.4;

team_formation_e TeamPlannerParameters::attack_formation = FORMATION_211;
team_formation_e TeamPlannerParameters::defense_formation = FORMATION_112;



TeamPlannerParameters::TeamPlannerParameters() {

}


std::string FormationAsString(team_formation_e formation) {
  std::string formationString = "";
  switch (formation) {
  case team_formation_e::FORMATION_013: formationString = "FORMATION_013"; break;
  case team_formation_e::FORMATION_112: formationString = "FORMATION_112"; break;
  case team_formation_e::FORMATION_211: formationString = "FORMATION_211"; break;
  case team_formation_e::FORMATION_310: formationString = "FORMATION_310"; break;
  case team_formation_e::FORMATION_ATTACK_SUPPORT_ONLY: formationString = "FORMATION_ATTACK_SUPPORT_ONLY"; break;
  case team_formation_e::FORMATION_DEFENDER_ONLY: formationString = "FORMATION_DEFENDER_ONLY"; break;
  case team_formation_e::FORMATION_INTERCEPTOR_ONLY: formationString = "FORMATION_INTERCEPTOR_ONLY"; break;
  case team_formation_e::FORMATION_SWEEPER_ONLY: formationString = "FORMATION_SWEEPER_ONLY"; break;
  case team_formation_e::FORMATION_SETPLAY_RECEIVER_ONLY: formationString = "FORMATION_SETPLAY_RECEIVER_ONLY"; break;
  case team_formation_e::FORMATION_SETPLAY_KICKER_ONLY: formationString = "FORMATION_SETPLAY_KICKER_ONLY"; break;
  case team_formation_e::FORMATION_BALLPLAYER_ONLY: formationString = "FORMATION_BALLPLAYER_ONLY"; break;
  case team_formation_e::FORMATION_SEARCHFORBALL_ONLY: formationString = "FORMATION_SEARCHFORBALL_ONLY"; break;
  case team_formation_e::FORMATION_BEGINPOSITION_ONLY: formationString = "FORMATION_BEGINPOSITION_ONLY"; break;
  case team_formation_e::FORMATION_PARKING_ONLY: formationString = "FORMATION_PARKING_ONLY"; break;
  case team_formation_e::FORMATION_PENALTYKICKER_ONLY: formationString = "FORMATION_PENALTYKICKER_ONLY"; break;
  case team_formation_e::FORMATION_PENALTY_SHOOTOUT: formationString = "FORMATION_PENALTY_SHOOTOUT"; break;
  default:
      formationString = "unknown formation (ERROR situation)";
  }
  return formationString;
}

std::string TeamPlannerParameters::toString() const  {
    std::stringstream buffer;
    buffer << "calculateAllPaths = " << calculateAllPaths << std::endl;
    buffer << "minimumEdgeLength = " << minimumEdgeLength  << std::endl;
    buffer << "maximumEdgeLength = " << maximumEdgeLength  << std::endl;
    buffer << "minimumDistanceToEndPoint = " << minimumDistanceToEndPoint  << std::endl;
    buffer << "nrVerticesFirstCircle = " << nrVerticesFirstCircle  << std::endl;
    buffer << "firstCircleRadius = " << firstCircleRadius  << std::endl;
    buffer << "VerticesSecondCircle  = " << nrVerticesSecondCircle  << std::endl;
    buffer << "secondCircleRadius  = " <<secondCircleRadius  << std::endl;
    buffer << "safetyFactor  = " <<safetyFactor  << std::endl;
    buffer << "addBarierVertices  = " <<addBarierVertices  << std::endl;
    buffer << "addUniformVertices  = " <<addUniformVertices  << std::endl;
    buffer << "manDefenseBetweenBallAndPlayer = " <<  manDefenseBetweenBallAndPlayer  << std::endl;
    buffer << "dist_before_penalty_area_for_sweeper = " <<dist_before_penalty_area_for_sweeper  << std::endl;
    buffer << "uniform_x_interval  = " <<uniform_x_interval  << std::endl;
    buffer << "uniform_y_interval  = " <<uniform_y_interval  << std::endl;
    buffer << "startingVelocityPenaltyFactor = " <<startingVelocityPenaltyFactor  << std::endl;
    buffer << "distToapplyBallApproachVertices  = " <<distToapplyBallApproachVertices  << std::endl;
    buffer << "addBallApproachVertices  = " <<addBallApproachVertices  << std::endl;
    buffer << "ballApproachVerticesRadius  = " <<ballApproachVerticesRadius  << std::endl;
    buffer << "ballApproachNumberOfVertices  = " <<ballApproachNumberOfVertices  << std::endl;
    buffer << "grid_size  = " <<grid_size  << std::endl;
    buffer << "nrDynamicPlannerIterations = " << nrDynamicPlannerIterations << endl;
    buffer << "maxPossibleLinearSpeed = " << maxPossibleLinearSpeed << endl;
    buffer << "maxPossibleLinearAcceleration = " << maxPossibleLinearAcceleration << endl;
    buffer << "interceptionChanceStartDistance = " << interceptionChanceStartDistance << endl;
    buffer << "interceptionChanceIncreasePerMeter = " << interceptionChanceIncreasePerMeter << endl;
    buffer << "interceptionChancePenaltyFactor = " << interceptionChancePenaltyFactor << endl;

    buffer << "grid_close_to_ball_normal_penalty = " << grid_close_to_ball_normal_penalty << endl;
    buffer << "grid_close_to_ball_normal_radius = " << grid_close_to_ball_normal_radius << endl;
    buffer << "grid_close_to_ball_restart_normal_penalty = " << grid_close_to_ball_restart_normal_penalty << endl;
    buffer << "grid_close_to_ball_restart_normal_radius = " << grid_close_to_ball_restart_normal_radius << endl;
    buffer << "grid_close_to_ball_restart_penalty_penalty = " << grid_close_to_ball_restart_penalty_penalty << endl;
    buffer << "grid_close_to_ball_restart_penalty_radius = " << grid_close_to_ball_restart_penalty_radius << endl;
    buffer << "grid_close_to_ball_restart_dropball_penalty = " << grid_close_to_ball_restart_dropball_penalty << endl;
    buffer << "grid_close_to_ball_restart_dropball_radius = " << grid_close_to_ball_restart_dropball_radius << endl;
    buffer << "grid_opponent_goal_clearance_x = " << grid_opponent_goal_clearance_x << endl;
    buffer << "grid_opponent_goal_clearance_y = "<< grid_opponent_goal_clearance_y << endl;
    buffer << "grid_own_goal_clearance_x = " << grid_own_goal_clearance_x << endl;
    buffer << "grid_own_goal_clearance_y = "<< grid_own_goal_clearance_y << endl;
    buffer << "nr_robots_needed_for_pass_play = " << nr_robots_needed_for_pass_play << endl;
    buffer << "nr_attack_support_during_defensive_period = " << nr_attack_support_during_defensive_period << endl;
    buffer << "wait_on_non_optimal_position_during_prepare_phase = " << wait_on_non_optimal_position_during_prepare_phase << endl;
    buffer << "priority_block_max_ball_y = "<< priority_block_max_ball_y << endl;
    buffer << "priority_block_max_opponent_to_ball_dist = " << priority_block_max_opponent_to_ball_dist << endl;
    buffer << "priority_block_check_ball_in_area = " << priority_block_check_ball_in_area << endl;
    buffer << "priority_block_check_opponent_close_to_ball = " << priority_block_check_opponent_close_to_ball << endl;
    buffer << "priority_block_min_distance = "<< priority_block_min_distance << endl;
    buffer << "priority_block_max_distance = "<< priority_block_max_distance << endl;
    buffer << "priority_block_max_distance_to_defense_line = "<< priority_block_max_distance_to_defense_line << endl;
    buffer << "attack_supporter_extra_distance_to_stay_from_sideline = " << attack_supporter_extra_distance_to_stay_from_sideline << endl;
    buffer << "restart_receiver_ball_dist = " <<  restart_receiver_ball_dist << endl;
    buffer << "restart_shooter_ball_dist = " <<  restart_shooter_ball_dist << endl;
    buffer << "equality_cost_threshold = " <<  equality_cost_threshold << endl;
    buffer << "select_lowest_robot_nr_for_dynamic_role = " << select_lowest_robot_nr_for_dynamic_role << endl;
    buffer << "previous_role_bonus_must_be_applied = " << previous_role_bonus_must_be_applied << endl;
    buffer << "previous_role_bonus_end_pos_radius = " << previous_role_bonus_end_pos_radius << endl;
    buffer << "use_pass_to_position_for_attack_support = " << use_pass_to_position_for_attack_support << endl;
    buffer << "no_sweeper_during_setplay= " << no_sweeper_during_setplay << endl;
    buffer << "interceptor_assign_use_ball_velocity = " << interceptor_assign_use_ball_velocity << endl;
    buffer << "interceptor_assign_min_velocity_for_calculate_interception_position = " << interceptor_assign_min_velocity_for_calculate_interception_position << endl;
    buffer << "man_to_man_defense_during_normal_play = " << man_to_man_defense_during_normal_play << endl;
    buffer << "man_to_man_defense_during_setplay_against = " << man_to_man_defense_during_setplay_against << endl;
    buffer << "dist_to_goal_to_mark_opponent_as_goalie = " << dist_to_goal_to_mark_opponent_as_goalie << endl;
    buffer << "setplay_against_dist_to_opponent = "  << setplay_against_dist_to_opponent << endl;


    buffer << "move_to_ball_left_field_position = " << move_to_ball_left_field_position << endl;
    buffer << "auto_save_svg_period = " << auto_save_svg_period << endl;
    buffer << "svgOutputFileName = " <<svgOutputFileName  << std::endl;
    buffer << "svgDefaultTargetColor  = " <<svgDefaultTargetColor  << std::endl;
    buffer << "svgBallColor = " << svgBallColor  << std::endl;
    buffer << "svgOriginalTargetColor = " << svgOriginalTargetColor << std::endl;
    buffer << "svgTeamColor =  " <<svgTeamColor << std::endl;
    buffer << "svgOpponentColor =  = " << svgOpponentColor  << std::endl;
    buffer << "svgDrawVelocity = " << svgDrawVelocity  << std::endl;
    buffer << "svgDrawEdges = " << svgDrawEdges  << std::endl;
    buffer << "saveGridDataToFile = " << saveGridDataToFile << std::endl;
    buffer << "svgRobotPlanner = " << svgRobotPlanner << std::endl;

    buffer << "preferredSetplayKicker = " << preferredSetplayKicker << std::endl;
    buffer << "preferredSetplayReceiver = " << preferredSetplayReceiver << std::endl;
    buffer << "setplay_margin_to_penalty_area_side = " << setplay_margin_to_penalty_area_side << std::endl;
    buffer << "lobShotWhenPossible = " << this->lobShotWhenPossible << std::endl;
    buffer << "min_y_for_lob_shot = " <<  this->min_y_for_lob_shot << std::endl;
    buffer << "outsideFieldMargin = " <<  this->outsideFieldMargin << std::endl;
    buffer << "kickoff_fp1 x = " << this->kickoff_fp1_x << std::endl;
    buffer << "kickoff_fp1 y = " << this->kickoff_fp1_y << std::endl;
    buffer << "kickoff_fp2 x = " << this->kickoff_fp2_x << std::endl;
    buffer << "kickoff_fp2 y = " << this->kickoff_fp2_y << std::endl;
    buffer << "kickoff_fp3 x = " << this->kickoff_fp3_x << std::endl;
    buffer << "kickoff_fp3 y = " << this->kickoff_fp3_y << std::endl;
    buffer << "kickoff_fp4 x = " << this->kickoff_fp4_x << std::endl;
    buffer << "kickoff_fp4 y = " << this->kickoff_fp4_y << std::endl;
    buffer << "kickoff_against_fp1 x = " << this->kickoff_against_fp1_x << std::endl;
    buffer << "kickoff_against_fp1 y = " << this->kickoff_against_fp1_y << std::endl;
    buffer << "kickoff_against_fp2 x = " << this->kickoff_against_fp2_x << std::endl;
    buffer << "kickoff_against_fp2 y = " << this->kickoff_against_fp2_y << std::endl;
    buffer << "kickoff_against_fp3 x = " << this->kickoff_against_fp3_x << std::endl;
    buffer << "kickoff_against_fp3 y = " << this->kickoff_against_fp3_y << std::endl;
    buffer << "kickoff_against_fp4 x = " << this->kickoff_against_fp4_x << std::endl;
    buffer << "kickoff_against_fp4 y = " << this->kickoff_against_fp4_y << std::endl;

    buffer << "dedicatedSweeper = " << dedicatedSweeper << std::endl;
    buffer << "autoAssignGoalie = "<< autoAssignGoalie << std::endl;
    return buffer.str();
}




} // namespace

