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
//#include "TeamPlannerThread.h"

#include "StrategyTester_generated.h" // generated
//#include "MovingObject.h"
#include "FieldConfig.h"
#include "SvgUtils.hpp"
#include "GlobalPathPlanner.hpp" // for print path
//#include "TeamPlannerRobot.hpp"
#include "TeamPlay.hpp"
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

team_formation_e StringToFormation(const string& formation_string) {
	team_formation_e formation = team_formation_e::FORMATION_013;
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



void getPlannerOptions(TeamPlannerParameters & options, auto_ptr<robotsports::StrategyType>& c) {
    options.calculateAllPaths = c->Options().calculateAllPaths();
    options.minimumEdgeLength = c->Options().minimumEdgeLength();
    options.maximumEdgeLength = c->Options().maximumEdgeLength();
    options.minimumDistanceToEndPoint = c->Options().minimumDistanceToEndPoint();
    options.nrVerticesFirstCircle = c->Options().nrVerticesFirstCircle();
    options.firstCircleRadius = c->Options().firstCircleRadius();
    options.nrVerticesSecondCircle = c->Options().nrVerticesSecondCircle();
    options.secondCircleRadius = c->Options().secondCircleRadius();
    options.safetyFactor = c->Options().safetyFactor();
    options.addBarierVertices = c->Options().addBarierVertices();
    options.addUniformVertices = c->Options().addUniformVertices();
    options.uniform_x_interval = c->Options().uniform_x_interval();
    options.uniform_y_interval = c->Options().uniform_y_interval();
    options.startingVelocityPenaltyFactor = c->Options().startingVelocityPenaltyFactor();
    options.addBallApproachVertices = c->Options().addBallApproachVertices();
    options.distToapplyBallApproachVertices = c->Options().distToapplyBallApproachVertices();
    options.ballApproachVerticesRadius = c->Options().ballApproachVerticesRadius();
    options.ballApproachNumberOfVertices = c->Options().ballApproachNumberOfVertices();
    options.manDefenseBetweenBallAndPlayer = c->Options().manDefenseBetweenBallAndPlayer();
    options.dist_before_penalty_area_for_sweeper = c->Options().dist_before_penalty_area_for_sweeper();
    options.grid_size = c->Options().grid_size();
    options.interceptionChanceStartDistance = c->Options().interceptionChanceStartDistance();
    options.interceptionChanceIncreasePerMeter = c->Options().interceptionChanceIncreasePerMeter();
    options.interceptionChancePenaltyFactor = c->Options().interceptionChancePenaltyFactor();
    options.grid_close_to_ball_normal_penalty = c->Options().grid_close_to_ball_normal_penalty();
    options.grid_close_to_ball_normal_radius = c->Options().grid_close_to_ball_normal_radius();
    options.grid_close_to_ball_restart_normal_penalty = c->Options().grid_close_to_ball_restart_normal_penalty();
    options.grid_close_to_ball_restart_normal_radius = c->Options().grid_close_to_ball_restart_normal_radius();
    options.grid_close_to_ball_restart_penalty_penalty = c->Options().grid_close_to_ball_restart_penalty_penalty();
    options.grid_close_to_ball_restart_penalty_radius = c->Options().grid_close_to_ball_restart_penalty_radius();
    options.grid_close_to_ball_restart_dropball_penalty = c->Options().grid_close_to_ball_restart_dropball_penalty();
    options.grid_close_to_ball_restart_dropball_radius = c->Options().grid_close_to_ball_restart_dropball_radius();
    options.grid_opponent_goal_clearance_x = c->Options().grid_opponent_goal_clearance_x();
    options.grid_opponent_goal_clearance_y = c->Options().grid_opponent_goal_clearance_y();
    options.grid_own_goal_clearance_x = c->Options().grid_own_goal_clearance_x();
    options.grid_own_goal_clearance_y = c->Options().grid_own_goal_clearance_y();
    options.nrDynamicPlannerIterations = c->Options().nrDynamicPlannerIterations();
//TODO    options.attack_formation = StringToFormation(c->AttackFormation());
//TODO    options.defense_formation = StringToFormation(c->DefenseFormation());
    options.maxPossibleLinearSpeed = c->Options().maxPossibleLinearSpeed();
    options.maxPossibleLinearAcceleration = c->Options().maxPossibleLinearAcceleration();
    options.nr_robots_needed_for_pass_play = c->Options().nr_robots_needed_for_pass_play();
    options.nr_attack_support_during_defensive_period = c->Options().nr_attack_support_during_defensive_period();
    options.wait_on_non_optimal_position_during_prepare_phase = c->Options().wait_on_non_optimal_position_during_prepare_phase();
    // plannerOptions.auto_save_svg_period not handled for xml only when teamplanner skill is used
    options.autoAssignGoalie = c->Options().autoAssignGoalie();
    options.preferredSetplayKicker = c->Options().preferredSetplayKicker();
    options.preferredSetplayReceiver = c->Options().preferredSetplayReceiver();
    options.use_pass_to_position_for_attack_support = c->Options().use_pass_to_position_for_attack_support();
    options.man_to_man_defense_during_normal_play = c->Options().man_to_man_defense_during_normal_play();
    options.man_to_man_defense_during_setplay_against = c->Options().man_to_man_defense_during_setplay_against();
    options.no_sweeper_during_setplay = c->Options().no_sweeper_during_setplay();
    options.interceptor_assign_use_ball_velocity = c->Options().interceptor_assign_use_ball_velocity();
    options.interceptor_assign_min_velocity_for_calculate_interception_position =  c->Options().interceptor_assign_min_velocity_for_calculate_interception_position();
    options.dist_to_goal_to_mark_opponent_as_goalie = c->Options().dist_to_goal_to_mark_opponent_as_goalie();
    options.setplay_against_dist_to_opponent = c->Options().setplay_against_dist_to_opponent();
    options.move_to_ball_left_field_position = c->Options().move_to_ball_left_field_position();

    options.svgDrawVelocity = c->Options().svgDrawVelocity();
    options.svgDrawEdges = c->Options().svgDrawEdges();
    options.svgDefaultTargetColor =  c->Options().svgDefaultTargetColor();
    options.svgBallColor = c->Options().svgBallColor();
    options.svgOriginalTargetColor = c->Options().svgOriginalTargetColor();
    options.svgTeamColor = c->Options().svgTeamColor();
    options.svgOpponentColor = c->Options().svgOpponentColor();
    options.svgOutputFileName = c->Options().svgOutputFileName();
    options.saveGridDataToFile = c->Options().saveGridDataToFile();
    options.svgRobotPlanner = c->Options().svgRobotPlanner();
//TODO new / added ? removed?    options.previous_role_bonus_end_pos_radius = c->Options().previous_role_bonus_end_pos_radius();
    options.priority_block_max_distance = c->Options().priority_block_max_distance();
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

void fillFieldConfig(FieldConfig& fieldConfig, auto_ptr<robotsports::StrategyType>& c)
{
    if (c->Field() != 0) {
        // If field info is present then overwrite the defaults with values from the xml file
    	fieldConfig = FillDefaultFieldConfig();
    	// TODO used setting functions
    	fieldConfig.FIELD_WIDTH = c->Field()->field_width();
        fieldConfig.FIELD_LENGTH = c->Field()->field_length();
        fieldConfig.FIELD_MARGIN = c->Field()->field_margin();
        fieldConfig.FIELD_MARKINGS_WIDTH = c->Field()->field_markings_width();
        fieldConfig.CENTER_CIRCLE_DIAMETER = c->Field()->center_circle_diameter();

        fieldConfig.GOAL_WIDTH = c->Field()->goal_width();
        fieldConfig.GOAL_LENGTH = c->Field()->goal_length();

        fieldConfig.GOAL_AREA_WIDTH = c->Field()->goal_area_width();
        fieldConfig.GOAL_AREA_LENGTH = c->Field()->goal_area_length();

        fieldConfig.PENALTY_AREA_PRESENT = c->Field()->penalty_area_present();
        fieldConfig.PENALTY_AREA_WIDTH = c->Field()->penalty_area_width();
        fieldConfig.PENALTY_AREA_LENGTH = c->Field()->penalty_area_length();

        fieldConfig.ROBOTSIZE = c->Field()->robot_size();
        fieldConfig.BALL_RADIUS = c->Field()->ball_radius();

        fieldConfig.PARKING_AREA_WIDTH = c->Field()->parking_area_width();
        fieldConfig.PARKING_AREA_LENGTH = c->Field()->parking_area_length();
        fieldConfig.PARKING_DISTANCE_BETWEEN_ROBOTS = c->Field()->parking_distance_between_robots();
        fieldConfig.PARKING_DISTANCE_TO_LINE = c->Field()->parking_distance_to_line();
//TODO        fieldConfig.CORNER_CIRCLE_DIAMETER  = c->Field()->corner_circle_diameter();
//TODO        fieldConfig.PENALTY_SPOT_TO_BACKLINE  = c->Field()->penalty_spot_to_backline();
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
        if ((*team_iter).hasBall()) {
            cout << "player with ball is  " << playerId << endl;
            r_team_has_ball = true;
        }
        P.position = Geometry::Position(*team_iter->x(), *team_iter->y(), team_iter->rz());
        P.velocity= Geometry::Position(team_iter->velx(), team_iter->vely(), team_iter->velrz());
        P.labelId = (*team_iter).label();
        P.controlBall = (*team_iter).hasBall();
        P.robotId = playerId;
        if ((*team_iter).isGoalie()) {
            P.player_type = player_type_e::GOALIE;
        }
        else{
            P.player_type = player_type_e::FIELD_PLAYER;
        }
        P.time_in_own_penalty_area = (*team_iter).time_in_own_penalty_area();
        P.time_in_opponent_penalty_area =  (*team_iter).time_in_enemy_penalty_area();
        P.passBall = (*team_iter).passedBall();
        P.dynamic_role = dynamic_role_e::dr_NONE;
        P.assigned = false;
        P.result = PlayerPlannerResult();
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
        long labelId = (*opponent_iter).label();
        if (labelId < 10) {
            labelId = playerId + 10;
        }
        TeamPlannerOpponent opponent;
        opponent.position = Geometry::Position(*opponent_iter->x(), *opponent_iter->y(), opponent_iter->rz());
        opponent.velocity= Geometry::Position(opponent_iter->velx(), opponent_iter->vely(), opponent_iter->velrz());
        opponent.label = playerId;
        Opponents.push_back(opponent);
    }

}

void fillTeamPlannerData(TeamPlannerData& tdp, game_state_e gamestate,
		const Geometry::Position& ball_position,
		const Geometry::Position& ball_velocity,
        const std::vector<Geometry::Position>& myTeam,
        const std::vector<Geometry::Position>& myTeam_vel,
		const std::vector<int>& myTeam_labels,
		const std::vector<Geometry::Position>& opponents,
		const std::vector<Geometry::Position>& opponents_vel,
		const std::vector<int>& opponents_labels,
        bool team_controls_ball, long controlBallByPlayerId,
        const std::vector<player_type_e>& teamTypes, const std::vector<long>& robotIds,
        const TeamPlannerParameters& options,
        const std::vector<Geometry::Point>& parking_postions,
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
        if (controlBallByPlayerId >= 0 && robotIds[idx] == static_cast<unsigned>(controlBallByPlayerId)) {
            robot.controlBall = true;
        }
        else {
            robot.controlBall = false;
        }
        robot.passBall = robot.robotId == static_cast<unsigned>(passBallByPlayerId);
        robot.player_type = teamTypes[idx];
        robot.dynamic_role = dynamic_role_e::dr_NONE;
        robot.assigned = false;
        robot.result = PlayerPlannerResult();
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
    tdp.team = Team;
    tdp.opponents = Opponents;
    tdp.parameters = options;
    tdp.parking_positions = parking_postions;
    tdp.ball_pickup_position = ball_pickup_position;
    tdp.passIsRequired = passIsRequired;
    tdp.pass_data = pass_data;
    tdp.team_controls_ball = team_controls_ball;
};


void xmlplanner(string input_filename) {
    string filename = input_filename;
    bool print_only_errors = false;
    if (not print_only_errors) {
        cout << "reading file : " << filename << endl;
    }

    TeamPlannerParameters parameters = {};
	std::vector<Geometry::Position> myTeam = std::vector<Geometry::Position>(); // TODO handle velocity
	std::vector<Geometry::Position> myTeam_vel = std::vector<Geometry::Position>(); // TODO handle velocity
	std::vector<int> myTeam_labels = {};
	std::vector<MRA::player_type_e> teamTypes = std::vector<MRA::player_type_e>();
	std::vector<MRA::player_type_e> opponentTypes = std::vector<MRA::player_type_e>();
	Geometry::Position ball = Geometry::Position();
	Geometry::Position ball_vel = Geometry::Position();
	std::vector<Geometry::Position> opponents = std::vector<Geometry::Position>();
	std::vector<Geometry::Position> opponents_vel = std::vector<Geometry::Position>();
	std::vector<int> opponents_labels = {};
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

	bool passIsRequired = false;
	bool team_controls_ball = false;
	long passBallByPlayer = -1; // no pass by any player is on its way
	long ownPlayerWithBall = -1;
	bool playerPassedBall = false;
	bool team_has_ball = false;
	pass_data_t pass_data = {.target_id=-1};
    previous_used_ball_by_planner_t previous_ball = {};
	try {
		auto_ptr<robotsports::StrategyType> c(robotsports::Situation(filename));
		description = c->Description();
		if (c->Ball() != 0) {
			ball.x = *c->Ball()->x();
			ball.y = *c->Ball()->y();
			ball_vel.x = c->Ball()->velx();
			ball_vel.y = c->Ball()->vely();
		}

        fillTeam(Team, playerPassedBall, team_has_ball, c);
        long playerId = 0;
        for (auto idx = 0u; idx< Team.size(); idx++) {
            playerId++;
            myTeam.push_back(Team[idx].position);
            myTeam_vel.push_back(Team[idx].velocity);
            myTeam_labels.push_back(Team[idx].labelId);

            previous_planner_results.push_back(Team[idx].previous_result);
            teamTypes.push_back(Team[idx].player_type);
            time_in_own_penalty_area.push_back(Team[idx].time_in_own_penalty_area);
            time_in_opponent_penalty_area.push_back(Team[idx].time_in_opponent_penalty_area);
            if  (Team[idx].passBall) {
                passBallByPlayer = playerId;
            }
            if  (Team[idx].controlBall) {
                ownPlayerWithBall  = playerId;
            }
            robotIds.push_back(Team[idx].robotId); // start player-id with 1, loop starts with 0.
        }

        fillOpponents(Opponents, c);
        for (auto idx = 0u; idx< Opponents.size(); idx++) {
            opponents.push_back(Opponents[idx].position);
            opponentTypes.push_back(player_type_e::FIELD_PLAYER);
        }

		gameState = gamestate_string_to_enum(c->GameState());
        fillFieldConfig(fieldConfig, c);
        getPlannerOptions(parameters, c);

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
			passIsRequired = c->SituationInfo()->passing_required();
			team_controls_ball = c->SituationInfo()->team_controls_ball();
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
    //cerr << "\t >>>> Options" << plannerOptions.toString() << endl << flush;
    std::vector<Geometry::Point> parking_postions = std::vector<Geometry::Point>();
    parking_postions.push_back(Geometry::Point(-6.375, -0.25));
    parking_postions.push_back(Geometry::Point(-6.375, -1.25));
    parking_postions.push_back(Geometry::Point(-6.375, -2.25));
    parking_postions.push_back(Geometry::Point(-6.375, -3.25));
    parking_postions.push_back(Geometry::Point(-6.375, -4.25));

    if (!pickup_pos_set) {
        pickup_pos.x = ball.x;
        pickup_pos.y = ball.y;
        pickup_pos.valid = ownPlayerWithBall >= 0;
        pickup_pos.ts = 0.0;
    }

    unsigned current_run = 1;
    std::vector<RunData> run_results = {};
	string run_filename = parameters.svgOutputFileName;
	if (not print_only_errors) {
		cout <<" current_run = " << current_run << endl;
		cout <<" xmlPlanner Fill DATA" << endl;
	}
	TeamPlannerData teamplannerData = {};
	// TODO teamplannerData.fieldConfig = fieldConfig;

//        if (current_run > 1) {
//            ball = run_results[0].teamplanner_data.ball;
//            previous_ball.previous_ball_present = 1;
//            previous_ball.x = run_results[0].teamplanner_data.ball.position.x;
//            previous_ball.y = run_results[0].teamplanner_data.ball.position.y;
//            auto const DELTA = 0.25;
//            auto xd = 0.0;
//            auto yd = 0.0;
//            if (current_run == 2) {
//                xd = -DELTA;
//            }
//            if (current_run == 3) {
//                xd = +DELTA;
//            }
//            if (current_run == 4) {
//                yd = -DELTA;
//            }
//            if (current_run == 5) {
//                yd = DELTA;
//            }
//            auto deltaPos = Geometry::Point(xd, yd);
//            ball.move(deltaPos, 0.0);
//            parameters.svgOutputFileName = orginal_svgOutputFileName;
//            char buffer[250];
//            sprintf(buffer, "_move_%02d.svg", current_run);
//            string filename = orginal_svgOutputFileName;
//            if (filename.size() > 4) {
//                filename.replace(filename.end() - 4, filename.end(), buffer);
//            }
//            run_filename = filename;
//            parameters.svgOutputFileName = ""; //run_filename;
//            previous_planner_results = std::vector<final_planner_result_t>();
//
//            team_planner_result_t org_result_paths = run_results[0].player_paths;
//            for (unsigned player_idx = 0; player_idx != org_result_paths.size(); player_idx++) {
//                std::vector<planner_piece_t> path = org_result_paths[player_idx].path;
//                Geometry::Point end_pos = Geometry::Point(path[path.size()-1].x, path[path.size()-1].y);
//                final_planner_result_t previous_result = {};
//                previous_result.previous_result_present = 1;
//                previous_result.dynamic_role = org_result_paths[player_idx].dynamic_role;
//                previous_result.end_position.x = end_pos.x;
//                previous_result.end_position.y = end_pos.y;
//                previous_result.ts = 0.125; // previous result 125 ms again (previous planner run)
//                previous_planner_results.push_back(previous_result);
//            }
//        }
	fillTeamPlannerData(teamplannerData, gameState,
			ball,
			ball_vel,
			myTeam,
			myTeam_vel,
			myTeam_labels,
			opponents,
			opponents_vel,
			opponents_labels,
			team_controls_ball, ownPlayerWithBall,
			teamTypes, robotIds, parameters, parking_postions, previous_ball, previous_planner_results,
			pickup_pos, passIsRequired, passBallByPlayer, pass_data, time_in_own_penalty_area, time_in_opponent_penalty_area);

	TeamPlay teamplay = TeamPlay();
	player_paths = teamplay.assign(teamplannerData);


	RunData run_data (teamplannerData, player_paths);
	run_results.push_back(run_data);

	if (player_paths.size() > 0) {
		if (not print_only_errors) {
			cerr << "<< XML: print received path " << endl << flush;
			cerr << TeamPlannerResultToString(player_paths, teamplannerData.team) << endl << flush;
		}

		if (current_run == 1) {
			SvgUtils::plannerdata_to_svg(player_paths, teamplannerData, fieldConfig, run_filename);
		}
		else {
			if (not print_only_errors) {
				cout  << "CHECK for run :" << current_run << endl;
			}
			auto too_large_delta_pos = false;
			team_planner_result_t org_result_paths = run_results[0].player_paths;
			team_planner_result_t result_paths = run_results[current_run-1].player_paths;
			for (unsigned player_idx = 0; player_idx != result_paths.size(); player_idx++) {
				std::vector<planner_piece_t> path = result_paths[player_idx].path;
				auto end_pos = Geometry::Point(path[path.size()-1].x, path[path.size()-1].y);
				std::vector<planner_piece_t> org_path = org_result_paths[player_idx].path;
				auto end_org_pos = Geometry::Point(org_path[org_path.size()-1].x, org_path[org_path.size()-1].y);
				if (end_pos.distanceTo(end_org_pos) > 1.0) {
					too_large_delta_pos = true;
					cout << "Run: " << current_run << " p[" << player_idx <<  "] id: " << run_results[current_run-1].teamplanner_data.team[player_idx].robotId
							<< "pos: " <<  end_pos.toString() << "ORG pos = " << end_org_pos.toString() << " diff " << end_pos.distanceTo(end_org_pos) << " ";
				}
				if ((result_paths[player_idx].dynamic_role != org_result_paths[player_idx].dynamic_role)) {
					if (!(result_paths[player_idx].dynamic_role == dr_DEFENDER && org_result_paths[player_idx].dynamic_role == dr_SWEEPER)) {
						if (!(result_paths[player_idx].dynamic_role == dr_SWEEPER && org_result_paths[player_idx].dynamic_role == dr_DEFENDER)) {
							too_large_delta_pos = true;
							cout << "Run: " << current_run << " role " << DynamicRoleAsString(result_paths[player_idx].dynamic_role) <<  " was: " << DynamicRoleAsString(org_result_paths[player_idx].dynamic_role) << " ";
						}
					}
				}
			}
			if (too_large_delta_pos) {
				auto comparing_player_paths = run_results[0].player_paths;
				SvgUtils::plannerdata_to_svg(run_results[current_run-1].player_paths, teamplannerData, fieldConfig, run_filename, comparing_player_paths);
			}
			if (not print_only_errors) {
				cout  << "FINISHED CHECK for run :" << current_run << endl;
			}
		}

	} else {
		cerr << "<< XML: no path received" << endl << flush;
	}
	if (not print_only_errors) {
		cerr << "<< Assign roles" << endl << flush;
	}

    auto finish = std::chrono::system_clock::now();
	double elapsed_seconds = std::chrono::duration_cast
			< std::chrono::duration<double> > (finish - start).count();
	// test start position assignments
    if (not print_only_errors) {
        std::cout << "elapsed time: " << elapsed_seconds * 1000 << " [ms]" << std::endl;
    }
}
