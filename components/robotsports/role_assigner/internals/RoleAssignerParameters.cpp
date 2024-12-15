/**
 *  @file
 *  @brief    Options for generating the visibility graph and the path planner
 *  @curator Jürge van Eijck
 */
#include "RoleAssignerParameters.hpp"

#include <ostream>
#include <sstream>
#include <iostream>

using namespace std;

namespace MRA {

bool RoleAssignerParameters::calculateAllPaths = true;
double RoleAssignerParameters::minimumEdgeLength = 0.30;
double RoleAssignerParameters::maximumEdgeLength = 4.00;
double RoleAssignerParameters::minimumDistanceToEndPoint = 3.00;
int RoleAssignerParameters::nrVerticesFirstCircle = 6;
double RoleAssignerParameters::firstCircleRadius = 0.80;
int RoleAssignerParameters::nrVerticesSecondCircle = 4;
double RoleAssignerParameters::secondCircleRadius = 1.60;
double RoleAssignerParameters::safetyFactor = 1.00;
bool RoleAssignerParameters::addBarierVertices = true;
bool RoleAssignerParameters::addUniformVertices = true;
bool RoleAssignerParameters::manDefenseBetweenBallAndPlayer = true;
double RoleAssignerParameters::dist_before_penalty_area_for_sweeper = 0.75;
double RoleAssignerParameters::uniform_x_interval = 2.00;
double RoleAssignerParameters::uniform_y_interval = 2.00;
double RoleAssignerParameters::startingVelocityPenaltyFactor = 1.00;
double RoleAssignerParameters::distToapplyBallApproachVertices = 10.00;
bool RoleAssignerParameters::addBallApproachVertices = false;
double RoleAssignerParameters::ballApproachVerticesRadius = 0.6;
int RoleAssignerParameters::ballApproachNumberOfVertices = 8;
double RoleAssignerParameters::grid_size = 0.75;
int RoleAssignerParameters::nrDynamicPlannerIterations = 2;
double RoleAssignerParameters::maxPossibleLinearSpeed = 1.5;
double RoleAssignerParameters::maxPossibleLinearAcceleration = 3.0;
std::string RoleAssignerParameters::svgOutputFileName = "";
std::string RoleAssignerParameters::svgDefaultTargetColor = "red";
std::string RoleAssignerParameters::svgBallColor = "orange";
std::string RoleAssignerParameters::svgOriginalTargetColor = "darkred";
std::string RoleAssignerParameters::svgTeamColor = "magenta";
std::string RoleAssignerParameters::svgOpponentColor = "cyan";
bool RoleAssignerParameters::svgDrawVelocity = true;
bool RoleAssignerParameters::svgDrawEdges = false;
bool RoleAssignerParameters::saveGridDataToFile = false;
bool RoleAssignerParameters::svgRobotPlanner = false;
double RoleAssignerParameters::interceptionChanceStartDistance = 1.0;
double RoleAssignerParameters::interceptionChanceIncreasePerMeter = 0.25;
double RoleAssignerParameters::interceptionChancePenaltyFactor = 4.0;
double RoleAssignerParameters::grid_close_to_ball_normal_penalty = 17000;
double RoleAssignerParameters::grid_close_to_ball_normal_radius = 3.0;
double RoleAssignerParameters::grid_close_to_ball_restart_normal_penalty = 17000;
double RoleAssignerParameters::grid_close_to_ball_restart_normal_radius = 3.0; // default the standard MSL rules, can be overwritten in ini file
double RoleAssignerParameters::grid_close_to_ball_restart_penalty_penalty = 17000;
double RoleAssignerParameters::grid_close_to_ball_restart_penalty_radius = 3.25;
double RoleAssignerParameters::grid_close_to_ball_restart_dropball_penalty = 17000;
double RoleAssignerParameters::grid_close_to_ball_restart_dropball_radius = 1.0; // default the standard MSL rules, can be overwritten in ini file
double RoleAssignerParameters::grid_opponent_goal_clearance_x = 7.0;
double RoleAssignerParameters::grid_opponent_goal_clearance_y = 2.5;
double RoleAssignerParameters::grid_own_goal_clearance_x = 3.25;
double RoleAssignerParameters::grid_own_goal_clearance_y = 1.00;
bool RoleAssignerParameters::wait_on_non_optimal_position_during_prepare_phase = false;
double RoleAssignerParameters::priority_block_min_distance = 0.5;
double RoleAssignerParameters::priority_block_max_distance = 2.0;
double RoleAssignerParameters::priority_block_max_distance_to_defense_line = 0.25;
bool RoleAssignerParameters::man_to_man_defense_during_normal_play = true;
double RoleAssignerParameters::attack_supporter_extra_distance_to_stay_from_sideline = 0.75;
double RoleAssignerParameters::auto_save_svg_period = 10.0;
double RoleAssignerParameters::restart_receiver_ball_dist = 2.5;
double RoleAssignerParameters::restart_shooter_ball_dist = 0.8;
double RoleAssignerParameters::equality_cost_threshold = 1.5;
bool RoleAssignerParameters::previous_role_bonus_must_be_applied = true;
double RoleAssignerParameters::previous_role_end_pos_threshold = 1.5;
double RoleAssignerParameters::previous_role_bonus_end_pos_radius = 5.0;
bool RoleAssignerParameters::use_pass_to_position_for_attack_support = true;
bool RoleAssignerParameters::man_to_man_defense_during_setplay_against = true;
double  RoleAssignerParameters::dist_to_goal_to_mark_opponent_as_goalie = 1.5;
double RoleAssignerParameters::setplay_against_dist_to_opponent = 1.5;

bool RoleAssignerParameters::move_to_ball_left_field_position = true;
int RoleAssignerParameters::preferredSetplayKicker = 0;
int RoleAssignerParameters::preferredSetplayReceiver = 0;
double RoleAssignerParameters::setplay_margin_to_penalty_area_side = 0.75;
bool RoleAssignerParameters::interceptor_assign_use_ball_velocity = true;
double RoleAssignerParameters::interceptor_assign_min_velocity_for_calculate_interception_position = 0.5;

bool RoleAssignerParameters::autoAssignGoalie = false;
double RoleAssignerParameters::outsideFieldMargin = 0.5;
double RoleAssignerParameters::min_y_for_lob_shot = +0.25;
double RoleAssignerParameters::kickoff_fp1_x = -1.00;
double RoleAssignerParameters::kickoff_fp1_y =  0.00;
double RoleAssignerParameters::kickoff_fp2_x = +1.70;
double RoleAssignerParameters::kickoff_fp2_y = -1.50;
double RoleAssignerParameters::kickoff_fp3_x = -5.00;
double RoleAssignerParameters::kickoff_fp3_y = -0.75;
double RoleAssignerParameters::kickoff_fp4_x = +5.00;
double RoleAssignerParameters::kickoff_fp4_y = -0.75;

double RoleAssignerParameters::kickoff_against_fp1_x = 1.87;
double RoleAssignerParameters::kickoff_against_fp1_y = -1.87;
double RoleAssignerParameters::kickoff_against_fp2_x = -1.87;
double RoleAssignerParameters::kickoff_against_fp2_y = -1.87;
double RoleAssignerParameters::kickoff_against_fp3_x = 3.5;
double RoleAssignerParameters::kickoff_against_fp3_y = -0.4;
double RoleAssignerParameters::kickoff_against_fp4_x = -3.5;
double RoleAssignerParameters::kickoff_against_fp4_y = -0.4;

RoleAssignerParameters::RoleAssignerParameters() {

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

std::string RoleAssignerParameters::toString() const  {
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
    buffer << "wait_on_non_optimal_position_during_prepare_phase = " << wait_on_non_optimal_position_during_prepare_phase << endl;
    buffer << "priority_block_min_distance = "<< priority_block_min_distance << endl;
    buffer << "priority_block_max_distance = "<< priority_block_max_distance << endl;
    buffer << "priority_block_max_distance_to_defense_line = "<< priority_block_max_distance_to_defense_line << endl;
    buffer << "attack_supporter_extra_distance_to_stay_from_sideline = " << attack_supporter_extra_distance_to_stay_from_sideline << endl;
    buffer << "restart_receiver_ball_dist = " <<  restart_receiver_ball_dist << endl;
    buffer << "restart_shooter_ball_dist = " <<  restart_shooter_ball_dist << endl;
    buffer << "equality_cost_threshold = " <<  equality_cost_threshold << endl;
    buffer << "previous_role_bonus_must_be_applied = " << previous_role_bonus_must_be_applied << endl;
    buffer << "previous_role_end_pos_threshold = " << previous_role_end_pos_threshold << endl;
    buffer << "previous_role_bonus_end_pos_radius = " << previous_role_bonus_end_pos_radius << endl;
    buffer << "use_pass_to_position_for_attack_support = " << use_pass_to_position_for_attack_support << endl;
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

    buffer << "autoAssignGoalie = "<< autoAssignGoalie << std::endl;
    return buffer.str();
}


} // namespace

