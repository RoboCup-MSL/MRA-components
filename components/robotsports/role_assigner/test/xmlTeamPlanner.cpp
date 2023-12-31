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

#include "StrategyTester_generated.h" // generated
#include "MovingObject.h"
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

bool updateObjects(const team_planner_result_t& player_paths,
		int ownPlayerWithBall, const team_planner_result_t& opponent_paths,
		int opponentWithBall, MRA::MovingObject& ball,
		vector<MRA::MovingObject>& myTeam, vector<MRA::MovingObject>& opponents,
		const FieldConfig& fieldConfig) {
	// update objects with 1.0 m per interation. returns false if no new updates can be done (ball out of bounds) or all paths have been executed.

	cout << "-----------------------------" << endl << flush;
	bool ret = true;

	double max_dist_per_update = 1.0;
	const double ballToRobotDist = (fieldConfig.getRobotRadius())
			+ (fieldConfig.getBallRadius());
	cout << "ballToRobotDist: " << ballToRobotDist << endl << flush;

	for (std::vector<unsigned int>::size_type player_idx = 0;
			player_idx != player_paths.size(); player_idx++) {
		double player_updated_dist = 0;

		std::vector<planner_piece_t> path = player_paths[player_idx].path;
		std::vector<unsigned int>::size_type path_idx = 1;
		while (path_idx < path.size()
				&& player_updated_dist < max_dist_per_update) {
			double x1 = path[path_idx - 1].x;
			double y1 = path[path_idx - 1].y;
			double x2 = path[path_idx].x;
			double y2 = path[path_idx].y;
			Geometry::Point offset(x2 - x1, y2 - y1);
			if (offset.size() > max_dist_per_update) {
				double factor = max_dist_per_update / offset.size();
				offset *= factor;
			}
			myTeam[player_idx].move(offset,
					offset.size() / max_dist_per_update);
			player_updated_dist += offset.size();
			cout << "Updated pos: [" << player_idx << "] = "
					<< myTeam[player_idx].toString() << endl << flush;
			;
			if (ownPlayerWithBall == static_cast<int>(player_idx)) {
			    Geometry::Point lastMoveDirection(offset);
				double ballFactor = ballToRobotDist / lastMoveDirection.size();
				cout << "ballFactor: " << ballFactor << endl << flush;
				lastMoveDirection *= ballFactor;

				cout << "lastMoveDirection: " << lastMoveDirection.toString() << endl << flush;
				Geometry::Point newBallVect = myTeam[player_idx].getPosition().getPoint();
				newBallVect += lastMoveDirection;
				ball.set(newBallVect.x, newBallVect.y, 0.0, 0.0, 0.0, 0.0, -1);
			}
			path_idx++;
		}
	}

	for (std::vector<unsigned int>::size_type player_idx = 0;
			player_idx != opponent_paths.size(); player_idx++) {
		double player_updated_dist = 0;
		std::vector<planner_piece_t> path = opponent_paths[player_idx].path;
		std::vector<unsigned int>::size_type path_idx = 1;
		while (path_idx < path.size()
				&& player_updated_dist < max_dist_per_update) {
			double x1 = path[path_idx - 1].x;
			double y1 = path[path_idx - 1].y;
			double x2 = path[path_idx].x;
			double y2 = path[path_idx].y;
			Geometry::Point offset(x2 - x1, y2 - y1);
			if (offset.size() > max_dist_per_update) {
				double factor = max_dist_per_update / offset.size();
				offset *= factor;
			}
			opponents[player_idx].move(offset,
					offset.size() / max_dist_per_update);
			player_updated_dist += offset.size();
			cout << "Updated pos: [" << player_idx << "] = "
					<< opponents[player_idx].toString() << endl << flush;
			;
			if (opponentWithBall == static_cast<int>(player_idx)) {
			    Geometry::Point lastMoveDirection(offset);
				double ballFactor = ballToRobotDist / lastMoveDirection.size();
				lastMoveDirection *= ballFactor;
				Geometry::Point newBallVect = opponents[player_idx].getPosition().getPoint();
                newBallVect += lastMoveDirection;
				ball.set(newBallVect.x, newBallVect.y, 0.0, 0.0, 0.0, 0.0, -1);
			}
			path_idx++;
		}
	}

	if ((ownPlayerWithBall == -1) && (opponentWithBall == -1)) {
		double timespan = 1.0;
		ball.move(timespan);
	}

	Geometry::Point newBallPos = ball.getPosition().getPoint();
	double sim_margin = 0.10; // simulation margin
	if (fabs(newBallPos.x) > (fieldConfig.getMaxFieldX() + sim_margin)) {
		ret = false;
	} else if (fabs(newBallPos.y)
			> (fieldConfig.getMaxFieldY() + sim_margin)) {
		ret = false;
	}

	return ret;
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

void xmlplanner(string input_filename, unsigned runs_needed) {
	TeamPlannerParameters plannerOptions = TeamPlannerParameters();

	std::vector<MRA::MovingObject> myTeam = std::vector<MRA::MovingObject>();
	std::vector<MRA::player_type_e> teamTypes =
			std::vector<MRA::player_type_e>();
	std::vector<MRA::player_type_e> opponentTypes = std::vector<
			MRA::player_type_e>();
	MRA::MovingObject ball = MRA::MovingObject();
	std::vector<MRA::MovingObject> opponents = std::vector<MRA::MovingObject>();
	game_state_e gameState;
	std::string description = "";
	FieldConfig fieldConfig = FillDefaultFieldConfig();
	ball_pickup_position_t pickup_pos = { 0 };
	bool pickup_pos_set = false;

	int numberOfIterations = 1;
	long ownPlayerWithBall = -1;
//	long opponentWithBall = -1;
	std::vector<double> time_in_own_penalty_area = std::vector<double>();
	std::vector<double> time_in_opponent_penalty_area = std::vector<double>();

	string filename = input_filename;
    bool print_only_errors = false;

    if (not print_only_errors) {
        cout << "reading file : " << filename << endl;
    }
	std::vector<long> robotIds = std::vector<long>();
	std::vector<long> opponentIds = std::vector<long>();
	std::vector<final_planner_result_t> previous_planner_results = std::vector<
			final_planner_result_t>();
	bool passIsRequired = false;
	long passBallByPlayer = -1; // no pass by any player is on its way
	bool ball_present = false;
	pass_data_t pass_data = { 0 };
	try {
		int ownGoalieId = -1;
		int opponnentGoalieId = -1;

		auto_ptr<robotsports::StrategyType> c(robotsports::Situation(filename));
		description = c->Description();
		if (c->Ball() != 0) {
			ball.set(*(c->Ball()->x()), *(c->Ball()->y()), 0.0, (c->Ball()->velx()), (c->Ball()->vely()), 0.0, 0);
			ball_present = true;
		}

		long playerId = 0;
		for (StrategyType::Team_const_iterator team_iter = c->Team().begin();
				team_iter != c->Team().end(); ++team_iter) {
			playerId++;
			MovingObject P = MRA::MovingObject(*(*team_iter).x(),
					*(*team_iter).y(), (*team_iter).rz(), (*team_iter).velx(),
					(*team_iter).vely(), (*team_iter).velrz(), playerId);
			myTeam.push_back(P);
			final_planner_result_t previous_result = { 0 };
			previous_result.previous_result_present =
					(*team_iter).previous_result_present();
			previous_result.dynamic_role = MRA::StringToDynamicRole(
					(*team_iter).previous_result_dynamic_role());
			previous_result.end_position.x = (*team_iter).previous_result_x();
			previous_result.end_position.y = (*team_iter).previous_result_y();
			previous_result.ts = (*team_iter).previous_result_ts();
			previous_planner_results.push_back(previous_result);

			if ((*team_iter).passedBall()) {
				passBallByPlayer = playerId;
				cout <<" XML teamplanner: pass by robot with id: " << playerId << endl;
			}

			if ((*team_iter).isGoalie()) {
				ownGoalieId = playerId;
				teamTypes.push_back(player_type_e::GOALIE);
			} else {
				teamTypes.push_back(player_type_e::FIELD_PLAYER);
			}
			if ((*team_iter).hasBall()) {
				cout << "player with ball is  " << playerId << endl;
				ownPlayerWithBall = playerId;
			}
			robotIds.push_back(playerId); // start player-id with 1, loop starts with 0.
			double time_in_penalty_area = (*team_iter).time_in_penalty_area();
			if (*(*team_iter).y() > 0) {
				// player at opponent half
				time_in_own_penalty_area.push_back(0.0);
				time_in_opponent_penalty_area.push_back(time_in_penalty_area);
			} else {
				// player at own half
				time_in_own_penalty_area.push_back(time_in_penalty_area);
				time_in_opponent_penalty_area.push_back(0.0);
			}
		}

		playerId = 0;
		for (StrategyType::Opponent_const_iterator opponent_iter =
				c->Opponent().begin(); opponent_iter != c->Opponent().end();
				++opponent_iter) {
			playerId++;
			opponents.push_back(
					MRA::MovingObject(*(*opponent_iter).x(),
							*(*opponent_iter).y(), (*opponent_iter).rz(),
							(*opponent_iter).velx(), (*opponent_iter).vely(),
							(*opponent_iter).velrz(), 10+playerId));
			opponentTypes.push_back(player_type_e::FIELD_PLAYER);
			if ((*opponent_iter).isGoalie()) {
				opponnentGoalieId = playerId;
			}
			if ((*opponent_iter).hasBall()) {
//				opponentWithBall = playerId;
			}
			opponentIds.push_back(playerId);
		}

		string gs = c->GameState();
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

		if (ownGoalieId > -1) {
			teamTypes[ownGoalieId-1] = player_type_e::GOALIE; /* id starts with 1. array with 0 */
		}
		if (opponnentGoalieId > -1) {
			opponentTypes[opponnentGoalieId-1] = player_type_e::GOALIE; /* id starts with 1. array with 0 */
		}

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

		numberOfIterations = c->Simulation().numberOfIterations();

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
//		plannerOptions.attack_formation = StringToFormation(
//				c->AttackFormation());
//		plannerOptions.defense_formation = StringToFormation(
//				c->DefenseFormation());
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

	string orginal_svgOutputFileName = plannerOptions.svgOutputFileName;
	auto start = std::chrono::system_clock::now();
	bool updates_not_done = true;
	for (int i = 1; i <= numberOfIterations && updates_not_done; i++) {
		team_planner_result_t* player_paths = 0;

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
			Position ballPos = ball.getPosition();
			pickup_pos.x = ballPos.getPoint().x;
			pickup_pos.y = ballPos.getPoint().y;
			pickup_pos.valid = ownPlayerWithBall >= 0;
			pickup_pos.ts = 0.0;
		}

		unsigned current_run = 1;
		auto run_results = std::vector<TeamPlannerRun>();
		while (current_run <= runs_needed) {
			string run_filename = "";
	        if (not print_only_errors) {
	            cout <<" current_run = " << current_run << endl;
	            cout <<" xmlPlanner Fill DATA" << endl;
	        }
//            TeamPlannerData teamplannerData = TeamPlannerData();
//			previous_used_ball_by_planner_t previous_ball;
//			previous_ball.previous_ball_present = 0;

// TODO add support for multiple runs where ball is moved a bit
//			if (current_run > 1) {
//				ball = run_results[0].ball;
//				previous_ball.previous_ball_present = 1;
//				previous_ball.x = run_results[0].ball.getXYlocation().x;
//				previous_ball.y = run_results[0].ball.getXYlocation().y;
//				auto const DELTA = 0.25;
//				auto xd = 0.0;
//				auto yd = 0.0;
//				if (current_run == 2) {
//					xd = -DELTA;
//				}
//				if (current_run == 3) {
//					xd = +DELTA;
//				}
//				if (current_run == 4) {
//					yd = -DELTA;
//				}
//				if (current_run == 5) {
//					yd = DELTA;
//				}
//				Geometry::Point deltaPos = Geometry::Point(xd, yd);
//				ball.move(deltaPos, 0.0);
//				plannerOptions.svgOutputFileName = orginal_svgOutputFileName;
//				char buffer[250];
//				sprintf(buffer, "_move_%02d.svg", current_run);
//				string filename = orginal_svgOutputFileName;
//				if (filename.size() > 4) {
//					filename.replace(filename.end() - 4, filename.end(), buffer);
//				}
//				run_filename = filename;
//				plannerOptions.svgOutputFileName = ""; //run_filename;
//				previous_planner_results = std::vector<final_planner_result_t>();
//
//				team_planner_result_t org_result_paths = *(run_results[0].result);
//
//
//				for (unsigned player_idx = 0; player_idx != org_result_paths.size(); player_idx++) {
//					std::vector<planner_piece_t> path = org_result_paths[player_idx].path;
//					Geometry::Point end_pos = Geometry::Point(path[path.size()-1].x, path[path.size()-1].y);
//					final_planner_result_t previous_result = { 0 };
//					previous_result.previous_result_present = 1;
//					previous_result.dynamic_role = org_result_paths[player_idx].dynamic_role;
//					previous_result.end_position.x = end_pos.x;
//					previous_result.end_position.y = end_pos.y;
//					previous_result.ts = 0.125; // previous result 125 ms again (previous planner run)
//					previous_planner_results.push_back(previous_result);
//				}
//			}
//			teamplannerData.gamestate = gameState;
//			teamplannerData.ball = ball;
//			teamplannerData.parking_positions = parking_postions;
//            teamplannerData.ball_present = ball_present;
//            teamplannerData.ball_pickup_position = pickup_pos;
//            teamplannerData.passIsRequired = passIsRequired;
//            teamplannerData.pass_data = pass_data;
//            teamplannerData.fieldConfig = fieldConfig;
//            teamplannerData.parameters = plannerOptions;
//            teamplannerData.playerPassedBall = passBallByPlayer;
//            teamplannerData.opponents = opponents;
//            teamplannerData.teamFormation;
//            teamplannerData.teamControlBall;
//            teamplannerData.ballIsObstacle;
//            teamplannerData.searchForBall;
//            teamplannerData.playerWhoIsPassing;
//            teamplannerData.defend_info;
//            teamplannerData.team;

//			fillData(myTeam,
//					, ownPlayerWithBall, teamTypes, robotIds,
//					previous_ball, previous_planner_results,
//					time_in_own_penalty_area, time_in_opponent_penalty_area);


            TeamPlannerInput tp_input;
            tp_input.gamestate = gameState;
            tp_input.fieldConfig = fieldConfig;
            tp_input.ball_pickup_position = pickup_pos;
            tp_input.passIsRequired = passIsRequired;
            tp_input.ball_present = ball_present;
            tp_input.playerPassedBall = passBallByPlayer;
            TeamPlannerState tp_state;
            TeamPlannerOutput tp_output;
            TeamPlannerParameters tp_parameters;
			TeamPlay teamplay = TeamPlay();

			teamplay.assign(tp_input, tp_state, tp_output, tp_parameters);

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
			current_run++;
            if (not print_only_errors) {
                cerr << "<< Assign roles" << endl << flush;
            }
		}

		// show delta results
		if (runs_needed > 1) {
			auto org_ball = run_results[0].input.ball;
			team_planner_result_t org_result_paths = *(run_results[0].output.player_paths);
			for (unsigned run_idx = 2; run_idx <= run_results.size(); ++run_idx) {
				auto result = run_results[run_idx-1];
				cout << "run[" << run_idx << "] ball: " << result.input.ball.toString(false) << " org ball:" << org_ball.toString(false) << endl;
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
}
