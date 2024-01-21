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
#include "../../robot_strategy/RobotsportsRobotStrategy.hpp"

#include "StrategyTester_generated.h" // generated
#include "FieldConfig.h"
#include "SvgUtils.hpp"
#include "GlobalPathPlanner.hpp" // for print path
#include "TeamPlay.hpp"
#include "WmTypes.h"



using namespace MRA;
using namespace std;
using namespace robotsports;

game_state_e getOpponentGameState(game_state_e gameState) {
	game_state_e opponentGameState = gameState;
	if (opponentGameState == game_state_e::CORNER) {
		opponentGameState = game_state_e::CORNER_AGAINST;
	} else if (opponentGameState == game_state_e::CORNER_AGAINST) {
		opponentGameState = game_state_e::CORNER;
	} else if (opponentGameState == game_state_e::KICKOFF) {
		opponentGameState = game_state_e::KICKOFF_AGAINST;
	} else if (opponentGameState == game_state_e::KICKOFF_AGAINST) {
		opponentGameState = game_state_e::KICKOFF;
	} else if (opponentGameState == game_state_e::FREEKICK) {
		opponentGameState = game_state_e::FREEKICK_AGAINST;
	} else if (opponentGameState == game_state_e::FREEKICK_AGAINST) {
		opponentGameState = game_state_e::FREEKICK;
	} else if (opponentGameState == game_state_e::GOALKICK) {
		opponentGameState = game_state_e::GOALKICK_AGAINST;
	} else if (opponentGameState == game_state_e::GOALKICK_AGAINST) {
		opponentGameState = game_state_e::GOALKICK;
	} else if (opponentGameState == game_state_e::THROWIN) {
		opponentGameState = game_state_e::THROWIN_AGAINST;
	} else if (opponentGameState == game_state_e::THROWIN_AGAINST) {
		opponentGameState = game_state_e::THROWIN;
	} else if (opponentGameState == game_state_e::PENALTY) {
		opponentGameState = game_state_e::PENALTY_AGAINST;
	} else if (opponentGameState == game_state_e::PENALTY_AGAINST) {
		opponentGameState = game_state_e::PENALTY;
	} else if (opponentGameState == game_state_e::PENALTY_SHOOTOUT) {
		opponentGameState = game_state_e::PENALTY_SHOOTOUT_AGAINST;
	} else if (opponentGameState == game_state_e::PENALTY_SHOOTOUT_AGAINST) {
		opponentGameState = game_state_e::PENALTY_SHOOTOUT;
	} else if (opponentGameState == game_state_e::GOAL) {
		opponentGameState = game_state_e::GOAL_AGAINST;
	} else if (opponentGameState == game_state_e::GOAL_AGAINST) {
		opponentGameState = game_state_e::GOAL;
	}
	return opponentGameState;
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

class TeamPlannerRun {
public:
    TeamPlannerInput input;
    TeamPlannerOutput output;
    TeamPlannerParameters parameters;
};

void fillPlannerOptions(TeamPlannerParameters& plannerOptions, auto_ptr<robotsports::StrategyType>& c)
{
    // Copy all input
    plannerOptions.calculateAllPaths = c->Options().calculateAllPaths();
    plannerOptions.minimumEdgeLength = c->Options().minimumEdgeLength();
    plannerOptions.maximumEdgeLength = c->Options().maximumEdgeLength();
    plannerOptions.maximumEdgeLength = 4.0;
    plannerOptions.minimumDistanceToEndPoint =
            c->Options().minimumDistanceToEndPoint();
    plannerOptions.nrVerticesFirstCircle =
            c->Options().nrVerticesFirstCircle();
    plannerOptions.firstCircleRadius = c->Options().firstCircleRadius();
    plannerOptions.nrVerticesSecondCircle =
            c->Options().nrVerticesSecondCircle();
    plannerOptions.secondCircleRadius = c->Options().secondCircleRadius();
    plannerOptions.safetyFactor = c->Options().safetyFactor();
    plannerOptions.addBarierVertices = c->Options().addBarierVertices();
    plannerOptions.addUniformVertices = c->Options().addUniformVertices();
    plannerOptions.uniform_x_interval = c->Options().uniform_x_interval();
    plannerOptions.uniform_y_interval = c->Options().uniform_y_interval();
    plannerOptions.startingVelocityPenaltyFactor =
            c->Options().startingVelocityPenaltyFactor();
    plannerOptions.addBallApproachVertices =
            c->Options().addBallApproachVertices();
    plannerOptions.distToapplyBallApproachVertices =
            c->Options().distToapplyBallApproachVertices();
    plannerOptions.ballApproachVerticesRadius =
            c->Options().ballApproachVerticesRadius();
    plannerOptions.ballApproachNumberOfVertices =
            c->Options().ballApproachNumberOfVertices();
    plannerOptions.manDefenseBetweenBallAndPlayer =
            c->Options().manDefenseBetweenBallAndPlayer();
    plannerOptions.dist_before_penalty_area_for_sweeper =
            c->Options().dist_before_penalty_area_for_sweeper();
    plannerOptions.grid_size = c->Options().grid_size();
    plannerOptions.interceptionChanceStartDistance =
            c->Options().interceptionChanceStartDistance();
    plannerOptions.interceptionChanceIncreasePerMeter =
            c->Options().interceptionChanceIncreasePerMeter();
    plannerOptions.interceptionChancePenaltyFactor =
            c->Options().interceptionChancePenaltyFactor();

    plannerOptions.grid_close_to_ball_normal_penalty =
            c->Options().grid_close_to_ball_normal_penalty();
    plannerOptions.grid_close_to_ball_normal_radius =
            c->Options().grid_close_to_ball_normal_radius();
    plannerOptions.grid_close_to_ball_restart_normal_penalty =
            c->Options().grid_close_to_ball_restart_normal_penalty();
    plannerOptions.grid_close_to_ball_restart_normal_radius =
            c->Options().grid_close_to_ball_restart_normal_radius();
    plannerOptions.grid_close_to_ball_restart_penalty_penalty =
            c->Options().grid_close_to_ball_restart_penalty_penalty();
    plannerOptions.grid_close_to_ball_restart_penalty_radius =
            c->Options().grid_close_to_ball_restart_penalty_radius();
    plannerOptions.grid_close_to_ball_restart_dropball_penalty =
            c->Options().grid_close_to_ball_restart_dropball_penalty();
    plannerOptions.grid_close_to_ball_restart_dropball_radius =
            c->Options().grid_close_to_ball_restart_dropball_radius();
    plannerOptions.grid_opponent_goal_clearance_x =
            c->Options().grid_opponent_goal_clearance_x();
    plannerOptions.grid_opponent_goal_clearance_y =
            c->Options().grid_opponent_goal_clearance_y();
    plannerOptions.grid_own_goal_clearance_x =
            c->Options().grid_own_goal_clearance_x();
    plannerOptions.grid_own_goal_clearance_y =
            c->Options().grid_own_goal_clearance_y();
    plannerOptions.nrDynamicPlannerIterations =
            c->Options().nrDynamicPlannerIterations();
// TODO
//      plannerOptions.attack_formation = StringToFormation(
//              c->AttackFormation());
//      plannerOptions.defense_formation = StringToFormation(
//              c->DefenseFormation());
    plannerOptions.maxPossibleLinearSpeed =
            c->Options().maxPossibleLinearSpeed();
    plannerOptions.maxPossibleLinearAcceleration =
            c->Options().maxPossibleLinearAcceleration();
    plannerOptions.nr_robots_needed_for_pass_play =
            c->Options().nr_robots_needed_for_pass_play();
    plannerOptions.nr_attack_support_during_defensive_period =
            c->Options().nr_attack_support_during_defensive_period();
    plannerOptions.wait_on_non_optimal_position_during_prepare_phase =
            c->Options().wait_on_non_optimal_position_during_prepare_phase();
    // plannerOptions.auto_save_svg_period not handled for xml only when teamplanner skill is used
    plannerOptions.autoAssignGoalie = c->Options().autoAssignGoalie();
    plannerOptions.preferredSetplayKicker =
            c->Options().preferredSetplayKicker();
    plannerOptions.preferredSetplayReceiver =
            c->Options().preferredSetplayReceiver();
    plannerOptions.use_pass_to_position_for_attack_support =
            c->Options().use_pass_to_position_for_attack_support();
    plannerOptions.man_to_man_defense_during_normal_play =
            c->Options().man_to_man_defense_during_normal_play();
    plannerOptions.man_to_man_defense_during_setplay_against =
            c->Options().man_to_man_defense_during_setplay_against();
    plannerOptions.no_sweeper_during_setplay =
            c->Options().no_sweeper_during_setplay();
    plannerOptions.interceptor_assign_use_ball_velocity =
            c->Options().interceptor_assign_use_ball_velocity();
    plannerOptions.interceptor_assign_min_velocity_for_calculate_interception_position =
            c->Options().interceptor_assign_min_velocity_for_calculate_interception_position();
    plannerOptions.dist_to_goal_to_mark_opponent_as_goalie =
            c->Options().dist_to_goal_to_mark_opponent_as_goalie();
    plannerOptions.setplay_against_dist_to_opponent =
            c->Options().setplay_against_dist_to_opponent();

    plannerOptions.move_to_ball_left_field_position =
            c->Options().move_to_ball_left_field_position();

    plannerOptions.svgDrawVelocity = c->Options().svgDrawVelocity();
    plannerOptions.svgDrawEdges = c->Options().svgDrawEdges();
    plannerOptions.svgDefaultTargetColor =
            c->Options().svgDefaultTargetColor();
    plannerOptions.svgBallColor = c->Options().svgBallColor();
    plannerOptions.svgOriginalTargetColor =
            c->Options().svgOriginalTargetColor();
    plannerOptions.svgTeamColor = c->Options().svgTeamColor();
    plannerOptions.svgOpponentColor = c->Options().svgOpponentColor();
    plannerOptions.svgOutputFileName = c->Options().svgOutputFileName();
    plannerOptions.saveGridDataToFile = c->Options().saveGridDataToFile();
    plannerOptions.svgRobotPlanner = c->Options().svgRobotPlanner();
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

void printInputs(const TeamPlannerInput& input)
{
    cerr << __func__ << "inputs:" << endl << flush;
    cerr << "game_state_e: " << input.gamestate << " (" << GameStateAsString(input.gamestate) << " )"<< endl << flush;
    cerr << "ball x: " << input.ball.x  << " y: " << input.ball.y << " z: " << input.ball.z << endl << flush;
//  cerr << "ball x: " << input.ball.position.x  << " y: " << input.ball.position.y << " z: " << input.ball.position.z
//                   <<" vx: " << input.ball.velocity.x  << " vy: " << input.ball.velocity.y << " vz: " << input.ball.velocity.z<< endl << flush;
    for (unsigned idx = 0; idx < input.team.size(); idx++) {
        cerr << "Robot [" << idx << "] =" << endl << input.team[idx].toString() << endl;
    }
    cerr << "Opponents size: " << input.opponents.size()  << endl << flush;
    for (unsigned int i = 0; i < input.opponents.size(); i++) {
        cerr << "Opponents[" << i << "].position: x: " << input.opponents[i].position.x  << " y: " << input.opponents[i].position.y << " z: " << input.opponents[i].position.z
                         <<" vx: " << input.opponents[i].velocity.x  << " vy: " << input.opponents[i].velocity.y << " vz: " << input.opponents[i].velocity.z<< endl << flush;

    }
//  cerr << "plannerOptions: " << input.parameters.toString() << endl << flush;
    cerr << "parking positions size: " << input.parking_positions.size() << endl << flush;
    for (unsigned int idx = 0; idx < input.parking_positions.size(); idx++) {
        cerr << input.parking_positions[idx].toString() << endl << flush;
    }

    if (input.ball_pickup_position.valid)
        cerr << "pickup: valid, x:" << std::setprecision(2) << input.ball_pickup_position.x << " y: " << input.ball_pickup_position.y << " ts:" << input.ball_pickup_position.ts << endl << flush;
    else{
        cerr << "pickup: invalid " << endl << flush;
    }

    cerr << "passIsRequired: " << (input.passIsRequired ? "true" : "false") << endl << flush;

    if (input.pass_data.valid) {
        cerr << "pass_data: valid target_id: " << input.pass_data.target_id << endl << flush;
        //      pass_data.target_id
        //      pass_data.kicked
        //      pass_data.origin_pos
        //      pass_data.target_pos
        //      pass_data.velocity
        //      pass_data.ts
        //      pass_data.eta
        //      pass_data.angle

    }
    else {
        cerr << "pass_data: invalid " << endl << flush;
    }
}

void fillOpponents(std::vector<MRA::TeamPlannerOpponent>& opponents, auto_ptr<robotsports::StrategyType>& c)
{
    int playerId = 0;
    const int ENEMY_LABEL_OFFSET = 10;
    for (StrategyType::Opponent_const_iterator opponent_iter =
            c->Opponent().begin(); opponent_iter != c->Opponent().end();
            ++opponent_iter) {
        playerId++;
        TeamPlannerOpponent op;
        op.position.x  = *(*opponent_iter).x();
        op.position.y  = *(*opponent_iter).y();
        op.position.rz = (*opponent_iter).rz();
        op.velocity.x  = (*opponent_iter).velx();
        op.velocity.y  = (*opponent_iter).vely();
        op.velocity.rz  = (*opponent_iter).velrz();
        op.label = ENEMY_LABEL_OFFSET + playerId;
        opponents.push_back(op);
    }
}

void fillTeam(std::vector<MRA::TeamPlannerRobot>& myTeam, bool& r_playerPassedBall, bool& r_team_has_ball, auto_ptr<robotsports::StrategyType>& c)
{
    long playerId = 0;
    r_playerPassedBall = false;
    for (StrategyType::Team_const_iterator team_iter = c->Team().begin();
            team_iter != c->Team().end(); ++team_iter) {
        playerId++;
        TeamPlannerRobot P;
        final_planner_result_t previous_result = { 0 };
        previous_result.previous_result_present =
                (*team_iter).previous_result_present();
        previous_result.dynamic_role = MRA::StringToDynamicRole(
                (*team_iter).previous_result_dynamic_role());
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

        P.position.x = *(*team_iter).x();
        P.position.y = *(*team_iter).y();
        P.position.rz = (*team_iter).rz();
        P.velocity.x = (*team_iter).velx();
        P.velocity.y = (*team_iter).vely();
        P.velocity.rz = (*team_iter).velrz();
        P.controlBall = (*team_iter).hasBall();
        P.robotId = playerId;
        P.is_keeper = (*team_iter).isGoalie();
        double time_in_penalty_area = (*team_iter).time_in_penalty_area();
        if (*(*team_iter).y() > 0) {
            // player at opponent half
            P.time_in_own_penalty_area = 0.0;
            P.time_in_opponent_penalty_area = time_in_penalty_area;
        } else {
            // player at own half
            P.time_in_own_penalty_area = time_in_penalty_area;
            P.time_in_opponent_penalty_area = 0.0;
        }
        P.passBall = (*team_iter).passedBall();
        P.dynamic_role = dynamic_role_e::dr_NONE;
        P.assigned = false;
        P.result = PlayerPlannerResult();
        P.previous_result = previous_result;

        myTeam.push_back(P);
    }
}



void fillFieldConfig(FieldConfig& fieldConfig, auto_ptr<robotsports::StrategyType>& c)
{
    if (c->Field() != 0) {
        // If field info is present then overwrite the defaults with values from the xml file
        fieldConfig.FIELD_LENGTH = c->Field()->field_length();
        fieldConfig.FIELD_WIDTH = c->Field()->field_width();
        fieldConfig.FIELD_MARGIN = c->Field()->field_margin();
        fieldConfig.GOAL_WIDTH = c->Field()->goal_width();
        fieldConfig.GOAL_LENGTH = c->Field()->goal_length();
        fieldConfig.CENTER_CIRCLE_DIAMETER = c->Field()->center_circle_diameter();
        fieldConfig.GOAL_AREA_WIDTH = c->Field()->goal_area_width();
        fieldConfig.GOAL_AREA_LENGTH = c->Field()->goal_area_length();
        fieldConfig.PENALTY_AREA_PRESENT = c->Field()->penalty_area_present();
        fieldConfig.PENALTY_AREA_WIDTH = c->Field()->penalty_area_width();
        fieldConfig.PENALTY_AREA_LENGTH = c->Field()->penalty_area_length();
        fieldConfig.ROBOTSIZE = c->Field()->robot_size();
        fieldConfig.BALL_RADIUS = c->Field()->ball_radius();
        fieldConfig.FIELD_MARKINGS_WIDTH = c->Field()->field_markings_width();
        fieldConfig.FIELD_MARKINGS_WIDTH_INTERNAL = c->Field()->field_markings_width_internal();
        fieldConfig.PARKING_AREA_WIDTH = c->Field()->parking_area_width();
        fieldConfig.PARKING_AREA_LENGTH = c->Field()->parking_area_length();
        fieldConfig.PARKING_DISTANCE_BETWEEN_ROBOTS = c->Field()->parking_distance_between_robots();
        fieldConfig.PARKING_DISTANCE_TO_LINE = c->Field()->parking_distance_to_line();
    }

}

void xmlplanner(string input_filename) {
	TeamPlannerParameters plannerOptions = TeamPlannerParameters();

	std::vector<MRA::TeamPlannerRobot> myTeam = std::vector<MRA::TeamPlannerRobot>();
	std::vector<MRA::player_type_e> teamTypes = std::vector<MRA::player_type_e>();
	MRA::TeamPlannerBall ball = MRA::TeamPlannerBall();
	std::vector<MRA::TeamPlannerOpponent> opponents = std::vector<MRA::TeamPlannerOpponent>();
	game_state_e gameState = game_state_e::NORMAL;
	std::string description = "";
	ball_pickup_position_t pickup_pos = { 0 };
	bool pickup_pos_set = false;

	int numberOfIterations = 1;
	std::vector<double> time_in_own_penalty_area = std::vector<double>();
	std::vector<double> time_in_opponent_penalty_area = std::vector<double>();

	string filename = input_filename;
    bool print_only_errors = false;

    if (not print_only_errors) {
        cout << "reading file : " << filename << endl;
    }
    cerr << __func__ << " : " << __LINE__ << endl;
	std::vector<long> robotIds = std::vector<long>();
	std::vector<final_planner_result_t> previous_planner_results = std::vector<
			final_planner_result_t>();
	bool passIsRequired = false;
    cerr << __func__ << " : " << __LINE__ << endl;
	bool ball_present = false;
	pass_data_t pass_data = { 0 };
	try {
	    cerr << __func__ << " : " << __LINE__ << endl;
		auto_ptr<robotsports::StrategyType> c(robotsports::Situation(filename));
		description = c->Description();
		if (c->Ball() != 0) {
			ball.position.x = *(c->Ball()->x());
			ball.position.y = *(c->Ball()->y());
			ball.velocity.x = c->Ball()->velx();
			ball.velocity.y = c->Ball()->vely();
			ball_present = true;
		}

		bool playerPassedBall = false;
		bool team_has_ball = false;
		fillTeam(myTeam, playerPassedBall, team_has_ball, c);
        cerr << __func__ << " : " << __LINE__ << endl;

	    fillOpponents(opponents, c);
	    cerr << __func__ << " : " << __LINE__ << endl;

		string game_state_str = c->GameState();
        cerr << __func__ << " : " << __LINE__ << endl;
		gameState = gamestate_string_to_enum(game_state_str);
        cerr << __func__ << " : " << __LINE__ << endl;

        numberOfIterations = c->Simulation().numberOfIterations();

        FieldConfig fieldConfig = FillDefaultFieldConfig();
        fillFieldConfig(fieldConfig, c);

		fillPlannerOptions(plannerOptions, c);


		string svgOutputFileName = plannerOptions.svgOutputFileName;
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
			if (c->SituationInfo()->PassData() != 0) {
				pass_data.ts = c->SituationInfo()->PassData()->ts();
				;
				pass_data.origin_pos.x =
						c->SituationInfo()->PassData()->origin_x();
				pass_data.origin_pos.y =
						c->SituationInfo()->PassData()->origin_y();
				pass_data.target_pos.x =
						c->SituationInfo()->PassData()->target_x();
				pass_data.target_pos.y =
						c->SituationInfo()->PassData()->target_y();
				pass_data.valid = c->SituationInfo()->PassData()->valid();
				pass_data.target_id = c->SituationInfo()->PassData()->target_id();
				cerr << "xml: pass_data.target_id =  " << c->SituationInfo()->PassData()->target_id() << endl;
			}
		}

	string orginal_svgOutputFileName = plannerOptions.svgOutputFileName;
	//auto start = std::chrono::system_clock::now();
	bool updates_not_done = true;
	for (int i = 1; i <= numberOfIterations && updates_not_done; i++) {
		char buffer[250];
		if (numberOfIterations > 1) {
			sprintf(buffer, "_%02d.svg", i);
		} else {
			sprintf(buffer, ".svg"); // single iteration
		}
		string filename = orginal_svgOutputFileName;
		if (filename.size() > 4) {
			filename.replace(filename.end() - 4, filename.end(), buffer);
		}
		plannerOptions.svgOutputFileName = filename;
		if (not print_only_errors) {
            cerr << ">>>> Assign roles" << endl << flush;
		}
		//cerr << "\t >>>> Options" << plannerOptions.toString() << endl << flush;
		vector<Geometry::Point> parking_postions = vector<Geometry::Point>();
		parking_postions.push_back(Geometry::Point(-6.375, -0.25));
		parking_postions.push_back(Geometry::Point(-6.375, -1.25));
		parking_postions.push_back(Geometry::Point(-6.375, -2.25));
		parking_postions.push_back(Geometry::Point(-6.375, -3.25));
		parking_postions.push_back(Geometry::Point(-6.375, -4.25));

		if (!pickup_pos_set) {
			pickup_pos.x = ball.position.x;
			pickup_pos.y = ball.position.y;
			pickup_pos.valid = team_has_ball;
			pickup_pos.ts = 0.0;
		}

		unsigned current_run = 1;
		auto run_results = std::vector<TeamPlannerRun>();


		auto strategy = MRA::RobotsportsRobotStrategy::RobotsportsRobotStrategy();
		MRA::RobotsportsRobotStrategy::Input strategy_input = MRA::RobotsportsRobotStrategy::Input();
		strategy_input.set_gamestate(MRA::RobotsportsRobotStrategy::Input_GameState_NORMAL);
		strategy_input.set_team_control_ball(false);
		strategy_input.set_ball_passed(false);

		MRA::RobotsportsRobotStrategy::Params strategy_params = MRA::RobotsportsRobotStrategy::Params();
		strategy_params.set_no_sweeper_during_setplay(true);
		strategy_params.set_attack_formation(MRA::RobotsportsRobotStrategy::Params_TeamFormation_FORMATION_112);
		strategy_params.set_defense_formation(MRA::RobotsportsRobotStrategy::Params_TeamFormation_FORMATION_112);
		auto strategy_output = MRA::RobotsportsRobotStrategy::Output();
		std::cout << __FILE__ << " input: " << convert_proto_to_json_str(strategy_input) << std::endl << std::flush;
        std::cerr << __FILE__ << " input: " << convert_proto_to_json_str(strategy_input) << std::endl << std::flush;
		std::cout << __FILE__ << " params: " << convert_proto_to_json_str(strategy_params) << std::endl;
		strategy.tick(strategy_input, strategy_params, strategy_output);
		std::cout << __FILE__ << " output: " << convert_proto_to_json_str(strategy_output) << std::endl;

		TeamPlannerInput tp_input = {};
		tp_input.gamestate = gameState;
		tp_input.fieldConfig = fieldConfig;
		tp_input.ball_pickup_position = pickup_pos;
		tp_input.passIsRequired = passIsRequired;
		tp_input.ball_present = ball_present;
		tp_input.ball = ball.position;
		tp_input.pass_data = pass_data;
		tp_input.playerPassedBall = playerPassedBall;
		tp_input.teamControlBall = team_has_ball;
		for (auto idx = 0; idx < strategy_output.dynamic_roles_size(); idx++) {
		    tp_input.teamFormation.push_back((dynamic_role_e) strategy_output.dynamic_roles(idx));
		}
		tp_input.parking_positions = parking_postions;
		tp_input.team = myTeam;
		tp_input.opponents = opponents;

        std::cout << "gameState: " << gameState << " (" << GameStateAsString(gameState) << " )"<< endl << flush;
        std::cout << "tp_input.game_state_e: " << tp_input.gamestate << " (" << GameStateAsString(tp_input.gamestate) << " )"<< endl << flush;
        printInputs(tp_input);

		TeamPlannerState tp_state;
		TeamPlannerOutput tp_output;
		TeamPlannerParameters tp_parameters;
		TeamPlay teamplay = TeamPlay();

		teamplay.assign(tp_input, tp_state, tp_output, tp_parameters);

#if 0

			auto run_data = TeamPlannerRun();
            run_data.input = tp_input;
            run_data.output = tp_output;
			run_data.parameters = tp_parameters;

			run_results.push_back(run_data);

			if (player_paths != 0) {
		        if (not print_only_errors) {
		            cerr << "<< XML: print received path " << endl << flush;
		            cerr << tp_output.pathToString() << endl << flush;
		        }
//				bool hasTeamPlannerInputInfo = true;
//				class TeamPlannerInputInfo inputInfo;
//				inputInfo.passIsRequired = passIsRequired;
//				inputInfo.playerWhoIsPassing = passBallByPlayer;
//				inputInfo.pass_data = pass_data;
//				inputInfo.ball_pickup_position = pickup_pos;
//				inputInfo.previous_results = previous_planner_results;

				if (current_run == 1) {
//					SvgUtils::save_graph_as_svg(ball, myTeam,
//							opponents, *player_paths, plannerOptions,
//							std::vector<Vertex*>(), gameState, ownPlayerWithBall,
//							teamTypes, robotIds, "red", fieldConfig,
//							hasTeamPlannerInputInfo, inputInfo);
				}
				else {
//	                if (not print_only_errors) {
//	                    cout  << "CHECK for run :" << current_run << endl;
//	                }
//					auto too_large_delta_pos = false;
//					vector<team_planner_result_t> org_result_paths = vector<team_planner_result_t>();
////					*(run_results[0].team);
//					team_planner_result_t result_paths = *(run_results[current_run-1].player_paths);
//					for (unsigned player_idx = 0; player_idx != result_paths.size(); player_idx++) {
//						std::vector<planner_piece_t> path = result_paths[player_idx].path;
//						Geometry::Point end_pos = Geometry::Point(path[path.size()-1].x, path[path.size()-1].y);
//						std::vector<planner_piece_t> org_path = org_result_paths[player_idx].path;
//						Geometry::Point end_org_pos = Geometry::Point(org_path[org_path.size()-1].x, org_path[org_path.size()-1].y);
//						if (end_pos.distanceTo(end_org_pos) > 1.0) {
//							too_large_delta_pos = true;
//                            cout << "Run: " << current_run << " p[" << player_idx <<  "] id: " << run_results[current_run-1].team[player_idx].robotId
//                                    << "pos: " <<  end_pos.toString() << "ORG pos = " << end_org_pos.toString() << " diff " << end_pos.distanceTo(end_org_pos) << " ";
//						}
//						if ((result_paths[player_idx].dynamic_role != org_result_paths[player_idx].dynamic_role)) {
//							if (!(result_paths[player_idx].dynamic_role == dr_DEFENDER && org_result_paths[player_idx].dynamic_role == dr_SWEEPER)) {
//								if (!(result_paths[player_idx].dynamic_role == dr_SWEEPER && org_result_paths[player_idx].dynamic_role == dr_DEFENDER)) {
//									too_large_delta_pos = true;
//									cout << "Run: " << current_run << " role " << DynamicRoleAsString(result_paths[player_idx].dynamic_role) <<  " was: " << DynamicRoleAsString(org_result_paths[player_idx].dynamic_role) << " ";
//								}
//							}
//						}
//						cout  << endl;
//					}
//					if (too_large_delta_pos) {
//						plannerOptions.svgOutputFileName = run_filename;
//						auto comparing_player_paths = run_results[0].player_paths;
//						SvgUtils::save_graph_as_svg(ball, myTeam,
//								opponents, *player_paths, *comparing_player_paths, plannerOptions,
//								std::vector<Vertex*>(), gameState, ownPlayerWithBall,
//								teamTypes, robotIds, "red", fieldConfig,
//								hasTeamPlannerInputInfo, inputInfo);
//					}
				}

			} else {
				cerr << "<< XML: no path received" << endl << flush;
			}
#endif
			current_run++;
            if (not print_only_errors) {
                cerr << "<< Assign roles" << endl << flush;
            }
		}

#if 0
	    // show delta results
        auto org_ball = run_results[0].input.ball;
        team_planner_result_t org_result_paths = *(run_results[0].output.player_paths);
        for (unsigned run_idx = 2; run_idx <= run_results.size(); ++run_idx) {
            auto result = run_results[run_idx-1];
            cout << "run[" << run_idx << "] ball: x:" << result.input.ball.x << " y: "<< result.input.ball.y
                                      << " org ball: x:" << org_ball.x << " y: " << org_ball.y << endl;
            team_planner_result_t result_paths = *(result.output.player_paths);
            for (unsigned player_idx = 0; player_idx != result_paths.size(); player_idx++) {
                std::vector<planner_piece_t> path = result_paths[player_idx].path;
                Geometry::Point end_pos = Geometry::Point(path[path.size()-1].x, path[path.size()-1].y);
                std::vector<planner_piece_t> org_path = org_result_paths[player_idx].path;
                Geometry::Point end_org_pos = Geometry::Point(org_path[org_path.size()-1].x, org_path[org_path.size()-1].y);
                cout << "p[" << player_idx <<  "] id: " << result.input.team[player_idx].robotId
                     << "pos: " <<  end_pos.toString() << " ";
                bool print_role = false;
                if (end_pos.distanceTo(end_org_pos) > 1.0) {
                    cout << " ORG pos = " << end_org_pos.toString() << " diff " << end_pos.distanceTo(end_org_pos) << " ";
                    print_role = true;
                }
                if (print_role || (result_paths[player_idx].dynamic_role != org_result_paths[player_idx].dynamic_role)) {
                    cout << DynamicRoleAsString(result_paths[player_idx].dynamic_role) <<  " was: " << DynamicRoleAsString(org_result_paths[player_idx].dynamic_role) << " ";

                }
                cout  << endl;
            }
        }

		if (player_paths != 0) {
			delete player_paths;
			player_paths = 0;
		}
	}
	auto finish = std::chrono::system_clock::now();
	double elapsed_seconds = std::chrono::duration_cast
			< std::chrono::duration<double> > (finish - start).count();
	// test start position assignments
    if (not print_only_errors) {
        std::cout << "elapsed time: " << elapsed_seconds * 1000 << " [ms]" << std::endl;
    }
#endif
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

}
