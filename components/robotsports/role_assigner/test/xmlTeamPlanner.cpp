/**
 *  @file
 *  @brief   Utility for handling xml input to test team planner
 *  @curator Jürge van Eijck
 */

#include <ostream>
#include <iostream>
#include <iomanip>
#include <istream>
#include <vector>
#include <string>
#include <cstdio>
#include <chrono>
#include <unistd.h>
#include <cmath>
#include <cstddef>
#include <boost/filesystem.hpp>

#include "xmlTeamPlanner.h"
#include "RobotsportsRobotStrategy.hpp"  // include robot strategy to get list of roles to assign


#include "StrategyTester_generated.h" // generated
#include "FieldConfig.hpp"
#include "SvgUtils.hpp"
#include "GlobalPathPlanner.hpp" // for print path
#include "TeamPlay.hpp"
#include "planner_types.hpp"
#include "TeamPlannerResult.hpp"

using namespace MRA;
using namespace std;
using namespace robotsports;

class RunData {
public:
    RunData(const TeamPlannerData& r_tpd, const std::vector<PlayerPlannerResult>& r_result) :
        teamplanner_data(r_tpd),
        player_paths(r_result)
    {

    }
    TeamPlannerData teamplanner_data;
    std::vector<PlayerPlannerResult> player_paths;
};

#include <string>
#include <sstream>
#include <vector>

std::vector<MRA::RobotsportsRobotStrategy::Output_DynamicRole> getListWithRoles(game_state_e gameState, ball_status_e ball_status,
                                                                                bool no_sweeper_during_setplay,
                                                                                team_formation_e attack_formation,
                                                                                team_formation_e defense_formation ) {

    auto robot_strategy = RobotsportsRobotStrategy::RobotsportsRobotStrategy();
    auto robot_strategy_input = RobotsportsRobotStrategy::Input();
    auto robot_strategy_output = RobotsportsRobotStrategy::Output();
    auto robot_strategy_params = RobotsportsRobotStrategy::Params();
    robot_strategy_params.set_no_sweeper_during_setplay(no_sweeper_during_setplay);
    robot_strategy_params.set_attack_formation((RobotsportsRobotStrategy::Params_TeamFormation)attack_formation);
    robot_strategy_params.set_defense_formation((RobotsportsRobotStrategy::Params_TeamFormation)defense_formation);

    MRA::RobotsportsRobotStrategy::Input_GameState gs = (MRA::RobotsportsRobotStrategy::Input_GameState) (gameState);
    robot_strategy_input.set_game_state(gs);
    robot_strategy_input.set_ball_status((MRA::RobotsportsRobotStrategy::Input_BallStatus) ball_status);

    auto error_value = robot_strategy.tick(robot_strategy_input, robot_strategy_params, robot_strategy_output);
    if (error_value != 0) {
        cout << "robot_strategy failed" << endl;
        exit(1);
    }

    std::vector<MRA::RobotsportsRobotStrategy::Output_DynamicRole> roles_to_assign = {};
    for (auto idx = 0; idx < robot_strategy_output.dynamic_roles_size(); idx++) {
        MRA::RobotsportsRobotStrategy::Output_DynamicRole odr = robot_strategy_output.dynamic_roles(idx);
        roles_to_assign.push_back(odr);
    }

    return roles_to_assign;
}



static std::string TeamPlannerResultToString(const team_planner_result_t& player_paths, const std::vector<TeamPlannerRobot>& team) {
    std::stringstream buffer;

    for (unsigned player_idx = 0; player_idx != player_paths.size(); player_idx++) {
        buffer << "path for player  " << player_idx <<  " id: " << team[player_idx].robotId <<  " -> " << DynamicRoleAsString(player_paths[player_idx].dynamic_role) <<  endl;
        if (player_paths[player_idx].defend_info.valid) {
            buffer << " Defend info: valid: true id: "<< player_paths[player_idx].defend_info.defending_id;
            buffer << " dist to id: " << player_paths[player_idx].defend_info.dist_from_defending_id;
            buffer << " between ball and id: " << player_paths[player_idx].defend_info.between_ball_and_defending_pos << endl;
        }
        else {
            buffer << " Defend info: valid: false" << endl;
        }
        std::vector<planner_piece_t> path = player_paths[player_idx].path;
        for (unsigned int path_idx = 0; path_idx != path.size(); path_idx++) {
            buffer << "path piece [ " << path_idx << "]  = (" << path[path_idx].x << ", "<< path[path_idx].y << ")" << endl;
        }

    }
    return buffer.str();
}

MRA::team_formation_e StringToFormation(const string& formation_string) {
    MRA::team_formation_e formation = team_formation_e::FORMATION_013;
    if (formation_string == "FORMATION_013") {
        formation = team_formation_e::FORMATION_013;
    } else if (formation_string == "FORMATION_112") {
        formation = team_formation_e::FORMATION_112;
    } else if (formation_string == "FORMATION_211") {
        formation = team_formation_e::FORMATION_211;
    } else if (formation_string == "FORMATION_310") {
        formation = team_formation_e::FORMATION_310;
    } else if (formation_string == "FORMATION_ATTACK_SUPPORT_ONLY") {
        formation = team_formation_e::FORMATION_ATTACK_SUPPORT_ONLY;
    } else if (formation_string == "FORMATION_DEFENDER_ONLY") {
        formation = team_formation_e::FORMATION_DEFENDER_ONLY;
    } else if (formation_string == "FORMATION_INTERCEPTOR_ONLY") {
        formation = team_formation_e::FORMATION_INTERCEPTOR_ONLY;
    } else if (formation_string == "FORMATION_SWEEPER_ONLY") {
        formation = team_formation_e::FORMATION_SWEEPER_ONLY;
    } else if (formation_string == "FORMATION_SETPLAY_RECEIVER_ONLY") {
        formation = team_formation_e::FORMATION_SETPLAY_RECEIVER_ONLY;
    } else if (formation_string == "FORMATION_SETPLAY_KICKER_ONLY") {
        formation = team_formation_e::FORMATION_SETPLAY_KICKER_ONLY;
    } else if (formation_string == "FORMATION_BALLPLAYER_ONLY") {
        formation = team_formation_e::FORMATION_BALLPLAYER_ONLY;
    } else if (formation_string == "FORMATION_SEARCHFORBALL_ONLY") {
        formation = team_formation_e::FORMATION_SEARCHFORBALL_ONLY;
    } else if (formation_string == "FORMATION_BEGINPOSITION_ONLY") {
        formation = team_formation_e::FORMATION_BEGINPOSITION_ONLY;
    } else if (formation_string == "FORMATION_PARKING_ONLY") {
        formation = team_formation_e::FORMATION_PARKING_ONLY;
    } else if (formation_string == "FORMATION_PENALTYKICKER_ONLY") {
        formation = team_formation_e::FORMATION_PENALTYKICKER_ONLY;
    } else {
        cerr << "UNKNOWN FORMATION xmlTeamPlanner.cpp: " << formation_string
                << endl;
    }
    return formation;
}

void getRoleAssignParameters(TeamPlannerParameters & parameters, auto_ptr<robotsports::StrategyType>& c) {
    parameters.calculateAllPaths = c->Options().calculateAllPaths();
    parameters.minimumEdgeLength = c->Options().minimumEdgeLength();
    parameters.maximumEdgeLength = c->Options().maximumEdgeLength();
    parameters.minimumDistanceToEndPoint = c->Options().minimumDistanceToEndPoint();
    parameters.nrVerticesFirstCircle = c->Options().nrVerticesFirstCircle();
    parameters.firstCircleRadius = c->Options().firstCircleRadius();
    parameters.nrVerticesSecondCircle = c->Options().nrVerticesSecondCircle();
    parameters.secondCircleRadius = c->Options().secondCircleRadius();
    parameters.safetyFactor = c->Options().safetyFactor();
    parameters.addBarierVertices = c->Options().addBarierVertices();
    parameters.addUniformVertices = c->Options().addUniformVertices();
    parameters.uniform_x_interval = c->Options().uniform_x_interval();
    parameters.uniform_y_interval = c->Options().uniform_y_interval();
    parameters.startingVelocityPenaltyFactor = c->Options().startingVelocityPenaltyFactor();
    parameters.addBallApproachVertices = c->Options().addBallApproachVertices();
    parameters.distToapplyBallApproachVertices = c->Options().distToapplyBallApproachVertices();
    parameters.ballApproachVerticesRadius = c->Options().ballApproachVerticesRadius();
    parameters.ballApproachNumberOfVertices = c->Options().ballApproachNumberOfVertices();
    parameters.manDefenseBetweenBallAndPlayer = c->Options().manDefenseBetweenBallAndPlayer();
    parameters.dist_before_penalty_area_for_sweeper = c->Options().dist_before_penalty_area_for_sweeper();
    parameters.grid_size = c->Options().grid_size();
    parameters.interceptionChanceStartDistance = c->Options().interceptionChanceStartDistance();
    parameters.interceptionChanceIncreasePerMeter = c->Options().interceptionChanceIncreasePerMeter();
    parameters.interceptionChancePenaltyFactor = c->Options().interceptionChancePenaltyFactor();
    parameters.grid_close_to_ball_normal_penalty = c->Options().grid_close_to_ball_normal_penalty();
    parameters.grid_close_to_ball_normal_radius = c->Options().grid_close_to_ball_normal_radius();
    parameters.grid_close_to_ball_restart_normal_penalty = c->Options().grid_close_to_ball_restart_normal_penalty();
    parameters.grid_close_to_ball_restart_normal_radius = c->Options().grid_close_to_ball_restart_normal_radius();
    parameters.grid_close_to_ball_restart_penalty_penalty = c->Options().grid_close_to_ball_restart_penalty_penalty();
    parameters.grid_close_to_ball_restart_penalty_radius = c->Options().grid_close_to_ball_restart_penalty_radius();
    parameters.grid_close_to_ball_restart_dropball_penalty = c->Options().grid_close_to_ball_restart_dropball_penalty();
    parameters.grid_close_to_ball_restart_dropball_radius = c->Options().grid_close_to_ball_restart_dropball_radius();
    parameters.grid_opponent_goal_clearance_x = c->Options().grid_opponent_goal_clearance_x();
    parameters.grid_opponent_goal_clearance_y = c->Options().grid_opponent_goal_clearance_y();
    parameters.grid_own_goal_clearance_x = c->Options().grid_own_goal_clearance_x();
    parameters.grid_own_goal_clearance_y = c->Options().grid_own_goal_clearance_y();
    parameters.nrDynamicPlannerIterations = c->Options().nrDynamicPlannerIterations();
    parameters.maxPossibleLinearSpeed = c->Options().maxPossibleLinearSpeed();
    parameters.maxPossibleLinearAcceleration = c->Options().maxPossibleLinearAcceleration();
    parameters.nr_robots_needed_for_pass_play = c->Options().nr_robots_needed_for_pass_play();
    parameters.nr_attack_support_during_defensive_period = c->Options().nr_attack_support_during_defensive_period();
    parameters.wait_on_non_optimal_position_during_prepare_phase = c->Options().wait_on_non_optimal_position_during_prepare_phase();
    // plannerOptions.auto_save_svg_period not handled for xml only when teamplanner skill is used
    parameters.autoAssignGoalie = c->Options().autoAssignGoalie();
    parameters.preferredSetplayKicker = c->Options().preferredSetplayKicker();
    parameters.preferredSetplayReceiver = c->Options().preferredSetplayReceiver();
    parameters.use_pass_to_position_for_attack_support = c->Options().use_pass_to_position_for_attack_support();
    parameters.man_to_man_defense_during_normal_play = c->Options().man_to_man_defense_during_normal_play();
    parameters.man_to_man_defense_during_setplay_against = c->Options().man_to_man_defense_during_setplay_against();
    parameters.interceptor_assign_use_ball_velocity = c->Options().interceptor_assign_use_ball_velocity();
    parameters.interceptor_assign_min_velocity_for_calculate_interception_position =  c->Options().interceptor_assign_min_velocity_for_calculate_interception_position();
    parameters.dist_to_goal_to_mark_opponent_as_goalie = c->Options().dist_to_goal_to_mark_opponent_as_goalie();
    parameters.setplay_against_dist_to_opponent = c->Options().setplay_against_dist_to_opponent();
    parameters.move_to_ball_left_field_position = c->Options().move_to_ball_left_field_position();

    parameters.svgDrawVelocity = c->Options().svgDrawVelocity();
    parameters.svgDrawEdges = c->Options().svgDrawEdges();
    parameters.svgDefaultTargetColor =  c->Options().svgDefaultTargetColor();
    parameters.svgBallColor = c->Options().svgBallColor();
    parameters.svgOriginalTargetColor = c->Options().svgOriginalTargetColor();
    parameters.svgTeamColor = c->Options().svgTeamColor();
    parameters.svgOpponentColor = c->Options().svgOpponentColor();
    parameters.svgOutputFileName = c->Options().svgOutputFileName();
    parameters.saveGridDataToFile = c->Options().saveGridDataToFile();
    parameters.svgRobotPlanner = c->Options().svgRobotPlanner();
    parameters.previous_role_end_pos_threshold  = c->Options().previous_role_end_pos_threshold();
    parameters.previous_role_bonus_end_pos_radius = c->Options().previous_role_bonus_end_pos_radius();
    parameters.priority_block_min_distance = c->Options().priority_block_min_distance();
    parameters.priority_block_max_distance = c->Options().priority_block_max_distance();
}

game_state_e gamestate_string_to_enum(std::string& gs) {
    game_state_e gameState  = game_state_e::NONE;
    if (gs == "Normal") {
        gameState = game_state_e::NORMAL;
    } else if (gs == "Parking") {
        gameState = game_state_e::PARKING;
    } else if (gs == "Begin Position") {
        gameState = game_state_e::BEGIN_POSITION;
    } else if (gs == "Kickoff") {
        gameState = game_state_e::KICKOFF;
    } else if (gs == "Kickoff Against") {
        gameState = game_state_e::KICKOFF_AGAINST;
    } else if (gs == "Freekick") {
        gameState = game_state_e::FREEKICK;
    } else if (gs == "Freekick Against") {
        gameState = game_state_e::FREEKICK_AGAINST;
    } else if (gs == "Goalkick") {
        gameState = game_state_e::GOALKICK;
    } else if (gs == "Goalkick Against") {
        gameState = game_state_e::GOALKICK_AGAINST;
    } else if (gs == "Throwin") {
        gameState = game_state_e::THROWIN;
    } else if (gs == "Throwin Against") {
        gameState = game_state_e::THROWIN_AGAINST;
    } else if (gs == "Corner") {
        gameState = game_state_e::CORNER;
    } else if (gs == "Corner Against") {
        gameState = game_state_e::CORNER_AGAINST;
    } else if (gs == "Penalty") {
        gameState = game_state_e::PENALTY;
    } else if (gs == "Penalty Against") {
        gameState = game_state_e::PENALTY_AGAINST;
    } else if (gs == "Penalty Shootout") {
        gameState = game_state_e::PENALTY_SHOOTOUT;
    } else if (gs == "Penalty Shootout Against") {
        gameState = game_state_e::PENALTY_SHOOTOUT_AGAINST;
    } else if (gs == "Dropped Ball") {
        gameState = game_state_e::DROPPED_BALL;
    } else if (gs == "Yellow Card Against") {
        gameState = game_state_e::YELLOW_CARD_AGAINST;
    } else if (gs == "Red Card Against") {
        gameState = game_state_e::RED_CARD_AGAINST;
    } else if (gs == "Goal") {
        gameState = game_state_e::GOAL;
    } else if (gs == "Goal Against") {
        gameState = game_state_e::GOAL_AGAINST;
    } else {
        cerr << "Unknown game state in xml file: " << gs << endl;
        exit(-1);
    }
    return gameState;
}

ball_status_e ball_status_to_enum(const xsd::cxx::tree::optional<robotsports::BallStatusType>& ball_status)
{
    if (!ball_status) {
        return ball_status_e::FREE;
    }
    switch (ball_status.get()) {
        case BallStatusType::value::FREE:              return ball_status_e::FREE;
        case BallStatusType::value::OWNED_BY_PLAYER:   return ball_status_e::OWNED_BY_PLAYER;
        case BallStatusType::value::OWNED_BY_TEAMMATE: return ball_status_e::OWNED_BY_TEAMMATE;
        case BallStatusType::value::OWNED_BY_TEAM:     return ball_status_e::OWNED_BY_TEAM;
        case BallStatusType::value::OWNED_BY_OPPONENT: return ball_status_e::OWNED_BY_OPPONENT;
        default:
            cerr << "Unknown ball_status in xml file: " << ball_status.get() << endl;
            exit(-1);
    }
    return ball_status_e::FREE;
}

void fillFieldConfig(FieldConfig& fieldConfig, auto_ptr<robotsports::StrategyType>& c)
{
    if (c->Field() != 0) {
        // If field info is present then overwrite the defaults with values from the xml file
        FieldConfig fc = FillDefaultFieldConfig();
        fc.setConfig(c->Field()->field_length(),
                     c->Field()->field_width(),
                     c->Field()->field_margin(),
                     c->Field()->goal_width(),
                     c->Field()->goal_length(),
                     c->Field()->center_circle_diameter(),
                    c->Field()->goal_area_width(),
                    c->Field()->goal_area_length(),
                    c->Field()->penalty_area_present(),
                    c->Field()->penalty_area_width(),
                    c->Field()->penalty_area_length(),
                    c->Field()->parking_area_width(),
                    c->Field()->parking_area_length(),
                    c->Field()->parking_distance_between_robots(),
                    c->Field()->parking_distance_to_line(),
                    c->Field()->robot_size(),
                    c->Field()->ball_radius(),
                    c->Field()->field_markings_width(),
                    c->Field()->corner_circle_diameter(),
                    c->Field()->penalty_spot_to_backline());
        fieldConfig = fc;
    }
}

void fillTeam(std::vector<TeamPlannerRobot>& Team, bool& r_playerPassedBall, bool& r_team_has_ball, auto_ptr<robotsports::StrategyType>& c)
{
    long playerId = 0;
    r_playerPassedBall = false;
    for (StrategyType::Team_const_iterator team_iter = c->Team().begin(); team_iter != c->Team().end(); ++team_iter) {
        playerId++;
        TeamPlannerRobot P;
        final_planner_result_t previous_result = { 0 };
        previous_result.previous_result_present =
                (*team_iter).previous_result_present();
        previous_result.dynamic_role = StringToDynamicRole((*team_iter).previous_result_dynamic_role());
        previous_result.end_position.x = (*team_iter).previous_result_x();
        previous_result.end_position.y = (*team_iter).previous_result_y();
        previous_result.ts = (*team_iter).previous_result_ts();

        if ((*team_iter).passedBall()) {
            r_playerPassedBall = true;
        }

        if ((*team_iter).isGoalie()) {
            P.player_type = player_type_e::GOALIE;
        } else {
            P.player_type = player_type_e::FIELD_PLAYER;
        }


        P.position = Geometry::Position(*team_iter->x(), *team_iter->y(), 0.0, 0.0, 0.0, team_iter->rz());
        P.velocity= Geometry::Position(team_iter->velx(), team_iter->vely(), 0.0, 0.0, 0.0, team_iter->velrz());
        
        // id of robot must be defined in xml-file
        if (not (*team_iter).id()) {
            cout << "No id defined for robot" << endl;
            exit(1);
        }
         P.robotId = (*team_iter).id().get();

        // label of robot must be defined in xml-file
        if (not (*team_iter).label()) {
            cout << "label-id not defined for an own player" << endl;
            exit(1);
        }
        P.labelId = (*team_iter).label().get();


        P.controlBall = (*team_iter).hasBall();
        if (P.controlBall) {
            cout << "player with ball is  " << P.robotId << endl;
            r_team_has_ball = true;
        }
        P.time_in_own_penalty_area = (*team_iter).time_in_own_penalty_area();
        P.time_in_opponent_penalty_area =  (*team_iter).time_in_enemy_penalty_area();
        P.passBall = (*team_iter).passedBall();
        P.assigned = false;
        P.result = {};
        P.previous_result = previous_result;

        Team.push_back(P);
    }
}



void fillOpponents(std::vector<TeamPlannerOpponent>& Opponents, auto_ptr<robotsports::StrategyType>& c)
{
    // const int ENEMY_LABEL_OFFSET = 10;
    long playerId = 0;
    for (StrategyType::Opponent_const_iterator opponent_iter =
            c->Opponent().begin(); opponent_iter != c->Opponent().end();
            ++opponent_iter) {
        playerId++;
        if (not (*opponent_iter).label()) {
            cout << "label-id not defined for a player of the opponents" << endl;
            exit(1);
        }
        long labelId = (*opponent_iter).label().get();
        if (labelId < 10) {
            labelId = playerId + 10;
            cout <<  " labe-id must be above 10 for an opponent." << endl;
            exit(1);
        }
        TeamPlannerOpponent opponent;
        opponent.position = Geometry::Position(*opponent_iter->x(), *opponent_iter->y(), 0.0, 0.0, 0.0, opponent_iter->rz());
        opponent.velocity= Geometry::Position(opponent_iter->velx(), opponent_iter->vely(), 0.0, 0.0, 0.0, opponent_iter->velrz());
        opponent.label = playerId;
        Opponents.push_back(opponent);
    }

}

void fillTeamPlannerData(TeamPlannerData& tdp, game_state_e gamestate,
        const Geometry::Position& ball_position,
        const Geometry::Position& ball_velocity,
        ball_status_e ball_status,
        const std::vector<Geometry::Position>& myTeam,
        const std::vector<Geometry::Position>& myTeam_vel,
        const std::vector<int>& myTeam_labels,
        const std::vector<Geometry::Position>& opponents,
        const std::vector<Geometry::Position>& opponents_vel,
        const std::vector<int>& opponents_labels,
        bool team_controls_ball, long controlBallByPlayerId,
        const std::vector<player_type_e>& teamTypes, const std::vector<long>& robotIds,
        const TeamPlannerParameters& options,
        const std::vector<Geometry::Point>& parking_positions,
        const previous_used_ball_by_planner_t& previous_ball,
        const std::vector<final_planner_result_t>& previous_planner_results,
        const ball_pickup_position_t& ball_pickup_position, bool passIsRequired,
        long passBallByPlayerId, const pass_data_t& pass_data,
        const std::vector<double>& time_in_own_penalty_area, const std::vector<double>& time_in_opponent_penalty_area)
{
    tdp.previous_ball = previous_ball;
    // calculate assignment for the given situation
    vector<TeamPlannerRobot> Team = vector<TeamPlannerRobot>();
    for (unsigned idx = 0; idx < myTeam.size(); idx++) {
        TeamPlannerRobot robot = {};
        robot.robotId = robotIds[idx];
        robot.labelId = myTeam_labels[idx];
        robot.position = myTeam[idx];
        robot.velocity = myTeam_vel[idx];
        if (controlBallByPlayerId >= 0 && robotIds[idx] == static_cast<unsigned>(controlBallByPlayerId)) {
            robot.controlBall = true;
        }
        else {
            robot.controlBall = false;
        }
        robot.passBall = robot.robotId == static_cast<unsigned>(passBallByPlayerId);
        robot.player_type = teamTypes[idx];
        robot.assigned = false;
        robot.result = {};
        if (idx < previous_planner_results.size()) {
            robot.previous_result = previous_planner_results[idx];
        }
        if (idx < time_in_own_penalty_area.size()) {
            robot.time_in_own_penalty_area = time_in_own_penalty_area[idx];
        }
        if (idx < time_in_opponent_penalty_area.size()) {
            robot.time_in_opponent_penalty_area = time_in_opponent_penalty_area[idx];
        }
        Team.push_back(robot);
    }
    std::vector<TeamPlannerOpponent> Opponents = std::vector<TeamPlannerOpponent>();
    for (unsigned idx = 0; idx < opponents.size(); idx++) {
        TeamPlannerOpponent opponent = {};
        opponent.position = opponents[idx];
        opponent.label = opponents_labels[idx];
        opponent.assigned = false;
        Opponents.push_back(opponent);
    }

    tdp.gamestate = gamestate;
    tdp.ball.position = ball_position;
    tdp.ball.velocity = ball_velocity;
    tdp.ball.status = ball_status;
    tdp.team = Team;
    tdp.opponents = Opponents;
    tdp.parameters = options;
    tdp.parking_positions = parking_positions;
    tdp.ball_pickup_position = ball_pickup_position;
    tdp.passIsRequired = passIsRequired;
    tdp.pass_data = pass_data;
};


void xmlplanner(string input_filename) {
    string filename = input_filename;
    bool print_only_errors = false;
    if (not print_only_errors) {
        cout << "reading file : " << filename << endl;
    }

    bool robot_strategy_parameter_no_sweeper_during_setplay = true;
    team_formation_e robot_strategy_parameter_attack_formation = FORMATION_013;
    team_formation_e robot_strategy_parameter_defense_formation = FORMATION_013;


    TeamPlannerParameters parameters = {};
    std::vector<Geometry::Position> myTeam = std::vector<Geometry::Position>();
    std::vector<Geometry::Position> myTeam_vel = std::vector<Geometry::Position>();
    std::vector<int> myTeam_labels = std::vector<int>();
    std::vector<MRA::player_type_e> teamTypes = std::vector<MRA::player_type_e>();
    std::vector<MRA::player_type_e> opponentTypes = std::vector<MRA::player_type_e>();
    Geometry::Position ball = Geometry::Position();
    Geometry::Position ball_vel = Geometry::Position();
    std::vector<Geometry::Position> opponents = std::vector<Geometry::Position>();
    std::vector<Geometry::Position> opponents_vel = std::vector<Geometry::Position>();
    std::vector<int> opponents_labels = std::vector<int>();
    game_state_e gameState;
    std::string description = "";
    MRA::FieldConfig fieldConfig = FillDefaultFieldConfig();
    ball_pickup_position_t pickup_pos = {};
    bool pickup_pos_set = false;

    std::vector<double> time_in_own_penalty_area = std::vector<double>();
    std::vector<double> time_in_opponent_penalty_area = std::vector<double>();

    std::vector<long> robotIds = std::vector<long>();
    std::vector<long> opponentIds = std::vector<long>();
    std::vector<final_planner_result_t> previous_planner_results = std::vector<final_planner_result_t>();


    std::vector<TeamPlannerRobot> Team = {};
    std::vector<TeamPlannerOpponent> Opponents = {};
    vector<Geometry::Point> parking_positions = {};

    bool passIsRequired = false;
    bool team_controls_ball = false;
    long passBallByPlayer = -1; // no pass by any player is on its way
    long ownPlayerWithBall = -1;
    bool playerPassedBall = false;
    bool team_has_ball = false;
    bool ball_is_valid = false;
    pass_data_t pass_data = {.target_id=-1};
    previous_used_ball_by_planner_t previous_ball = {};
    ball_status_e ball_status = ball_status_e::FREE;
    try {
        auto_ptr<robotsports::StrategyType> c(robotsports::Situation(filename));
        description = c->Description();
        if (c->Ball() != 0) {
            ball_is_valid = true;
            ball.x = *c->Ball()->x();
            ball.y = *c->Ball()->y();
            ball_vel.x = c->Ball()->velx();
            ball_vel.y = c->Ball()->vely();
        }

        fillTeam(Team, playerPassedBall, team_has_ball, c);

        for (auto idx = 0u; idx< Team.size(); idx++) {
            myTeam.push_back(Team[idx].position);
            myTeam_vel.push_back(Team[idx].velocity);
            myTeam_labels.push_back(Team[idx].labelId);

            previous_planner_results.push_back(Team[idx].previous_result);
            teamTypes.push_back(Team[idx].player_type);
            time_in_own_penalty_area.push_back(Team[idx].time_in_own_penalty_area);
            time_in_opponent_penalty_area.push_back(Team[idx].time_in_opponent_penalty_area);
            if  (Team[idx].passBall) {
                passBallByPlayer = Team[idx].robotId;
            }
            if  (Team[idx].controlBall) {
                ownPlayerWithBall  = Team[idx].robotId;
            }
            robotIds.push_back(Team[idx].robotId); // start player-id with 1, loop starts with 0.
        }

        fillOpponents(Opponents, c);
        for (auto idx = 0u; idx< Opponents.size(); idx++) {
            opponents.push_back(Opponents[idx].position);
            opponents_vel.push_back(Opponents[idx].velocity);
            opponents_labels.push_back(Opponents[idx].label);
            opponentTypes.push_back(player_type_e::FIELD_PLAYER);
        }

        gameState = gamestate_string_to_enum(c->GameState());
        fillFieldConfig(fieldConfig, c);
        getRoleAssignParameters(parameters, c);
        robot_strategy_parameter_no_sweeper_during_setplay = c->Options().no_sweeper_during_setplay();
        robot_strategy_parameter_attack_formation =  StringToFormation(c->AttackFormation());
        robot_strategy_parameter_defense_formation = StringToFormation(c->DefenseFormation());



        string svgOutputFileName = parameters.svgOutputFileName;
        std::size_t found = svgOutputFileName.find_last_of("/");
        if (found != string::npos) {
            // svgOutputFileName contains /,the sub directory relative to current directory should exists
            auto svg_output_dir = svgOutputFileName.substr(0,found);
            if (not boost::filesystem::exists(svg_output_dir)) {
                auto created_new_directory = boost::filesystem::create_directory(svg_output_dir);
                if (not created_new_directory) {
                    // Either creation failed or the directory was already present.
                    cout << "FAILED to create directory: " << svg_output_dir << endl;
                }
                else {
                    cout << "Successful created directory: " << svg_output_dir << endl;
                }
            }
        }

        if (c->PickupPosition()) {
            pickup_pos.x = c->PickupPosition()->x();
            pickup_pos.y = c->PickupPosition()->y();
            pickup_pos.valid = c->PickupPosition()->valid();
            pickup_pos.ts = c->PickupPosition()->ts();
            pickup_pos_set = true;
        }
        if (c->SituationInfo()) {
            if (not c->SituationInfo()->passing_required()) {
                cout << "SituationInfo must have attribute \"passing_required\"" << endl;
                exit(1);
            }
            passIsRequired = c->SituationInfo()->passing_required().get();
            ball_status = ball_status_to_enum(c->SituationInfo()->ball_status());
            if (c->SituationInfo()->PassData() != 0) {
                pass_data.ts = c->SituationInfo()->PassData()->ts();
                pass_data.origin_pos.x = c->SituationInfo()->PassData()->origin_x();
                pass_data.origin_pos.y = c->SituationInfo()->PassData()->origin_y();
                pass_data.target_pos.x = c->SituationInfo()->PassData()->target_x();
                pass_data.target_pos.y = c->SituationInfo()->PassData()->target_y();
                pass_data.valid = c->SituationInfo()->PassData()->valid();
                pass_data.target_id = c->SituationInfo()->PassData()->target_id();
            }
        }
        if (c->PreviousBall()) {
            previous_ball.previous_ball_present = true;
            previous_ball.x = c->PreviousBall()->x();
            previous_ball.y = c->PreviousBall()->y();
        }

        if (c->ParkingInfo()) {
            auto parkpos_list = c->ParkingInfo().get().ParkingPosition();
            for (auto parkpos_iter = parkpos_list.begin(); parkpos_iter != parkpos_list.end(); ++parkpos_iter) {
                parking_positions.push_back(Geometry::Point((*parkpos_iter).x().get(), (*parkpos_iter).y().get()));
            }
        }
        else {
            // default parking positions
            parking_positions.push_back(Geometry::Point(-6.375, -0.25));
            parking_positions.push_back(Geometry::Point(-6.375, -1.25));
            parking_positions.push_back(Geometry::Point(-6.375, -2.25));
            parking_positions.push_back(Geometry::Point(-6.375, -3.25));
            parking_positions.push_back(Geometry::Point(-6.375, -4.25));
        }

    } catch (const xml_schema::exception& e) {
        cerr << e << endl;
        return;
    } catch (const xml_schema::properties::argument&) {
        cerr << "invalid property argument (empty namespace or location)"
                << endl;
        return;
    } catch (const xsd::cxx::xml::invalid_utf16_string&) {
        cerr << "invalid UTF-16 text in DOM model" << endl;
        return;
    } catch (const xsd::cxx::xml::invalid_utf8_string&) {
        cerr << "invalid UTF-8 text in object model" << endl;
        return;
    }
    if (not print_only_errors) {
        cerr << "start planner" << endl;
        cout << "running : " << endl << description << endl;
    }

    string orginal_svgOutputFileName = parameters.svgOutputFileName;
    auto start = std::chrono::system_clock::now();

    std::vector<PlayerPlannerResult> player_paths = {};

    char buffer[250];
    sprintf(buffer, ".svg");
    filename = orginal_svgOutputFileName;
    if (filename.size() > 4) {
        filename.replace(filename.end() - 4, filename.end(), buffer);
    }
    parameters.svgOutputFileName = filename;
    if (not print_only_errors) {
        cerr << ">>>> Assign roles" << endl << flush;
    }

    if (!pickup_pos_set) {
        pickup_pos.x = ball.x;
        pickup_pos.y = ball.y;
        pickup_pos.valid = ownPlayerWithBall >= 0;
        pickup_pos.ts = 0.0;
    }

    std::vector<RunData> run_results = {};
    string run_filename = parameters.svgOutputFileName;
    if (not print_only_errors) {
        cout <<" xmlPlanner Fill DATA" << endl;
    }
    TeamPlannerData teamplannerData = {};
    teamplannerData.ball.is_valid = ball_is_valid;
    teamplannerData.fieldConfig = fieldConfig;


    fillTeamPlannerData(teamplannerData, gameState,
            ball,
            ball_vel,
            ball_status,
            myTeam,
            myTeam_vel,
            myTeam_labels,
            opponents,
            opponents_vel,
            opponents_labels,
            team_controls_ball, ownPlayerWithBall,
            teamTypes, robotIds, parameters, parking_positions, previous_ball, previous_planner_results,
            pickup_pos, passIsRequired, passBallByPlayer, pass_data, time_in_own_penalty_area, time_in_opponent_penalty_area);

    teamplannerData.ball.is_valid = ball_is_valid;
    teamplannerData.fieldConfig = fieldConfig;

    teamplannerData.input_formation = getListWithRoles(gameState, ball_status,
                                                       robot_strategy_parameter_no_sweeper_during_setplay,
                                                       robot_strategy_parameter_attack_formation,
                                                       robot_strategy_parameter_defense_formation);


    TeamPlay teamplay = TeamPlay();
    player_paths = teamplay.assign(teamplannerData);

    RunData run_data (teamplannerData, player_paths);
    run_results.push_back(run_data);

    if (player_paths.size() > 0) {
        if (not print_only_errors) {
            cerr << "<< XML: print received path " << endl << flush;
            cerr << TeamPlannerResultToString(player_paths, teamplannerData.team) << endl << flush;
        }
        SvgUtils::plannerdata_to_svg(player_paths, teamplannerData, fieldConfig, run_filename);

    } else {
        cerr << "<< XML: no path received" << endl << flush;
    }
    if (not print_only_errors) {
        cerr << "<< Assign roles" << endl << flush;
    }

    auto finish = std::chrono::system_clock::now();
    double elapsed_seconds = std::chrono::duration_cast< std::chrono::duration<double> > (finish - start).count();
    // test start position assignments
    if (not print_only_errors) {
        std::cout << "elapsed time: " << elapsed_seconds * 1000 << " [ms]" << std::endl;
    }
}
