/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */
#ifndef TEAM_PLANNER_DATA_H
#define TEAM_PLANNER_DATA_H 1

#include "MovingObject.h"
#include "WmTypes.h"
#include "FieldConfig.h"
#include <vector>

#include "PlannerOptions.hpp"
#include "TeamPlannerOpponent.hpp"
#include "TeamPlannerResult.hpp"

typedef struct pass_data_s {
	long   valid; // 1: if data is valid; 0 otherwise
	long   kicked; // 1: if pass/shot has been made; 0: otherwise
	long   target_id; // destination of kick, where 0 is goal
	double velocity; // [m/s]
	double angle; // upwards angle
	MRA::Geometry::Point  origin_pos; // field coordinates of origin
	MRA::Geometry::Point  target_pos; // field coordinates of target
	double ts; // timestamp of update
	double eta; // estimated time of arrival at target as liveseconds [s]
} pass_data_t;

namespace trs {



class TeamPlannerRobot {
public:
	long robotId;
	MovingObject position;
	bool controlBall;
	bool passBall; // indicator whether a pass by this player is still on its way
	player_type_e player_type;
	bool assigned;
	PlayerPlannerResult result;
	final_planner_result_t previous_result;
	dynamic_role_e dynamic_role;
	double time_in_own_penalty_area;
	double time_in_opponent_penalty_area;

	std::string toString();

};

class TeamPlannerInputInfo {
public:
	int playerWhoIsPassing;
	std::vector<final_planner_result_t> previous_results;
	ball_pickup_position_t ball_pickup_position;
	bool passIsRequired;
	pass_data_t pass_data;
};
class TeamPlannerData {
public:
	team_planner_result_t* player_paths;
	game_state_e gamestate;
	MovingObject globalBall;
	MovingObject localBall;
	previous_used_ball_by_planner_t previous_global_ball;
	std::vector<TeamPlannerRobot> Team;
	std::vector<TeamPlannerOpponent> Opponents;
	PlannerOptions plannerOptions;
	std::vector<MRA::Geometry::Point> parking_positions;
	ball_pickup_position_t ball_pickup_position;
	bool passIsRequired;
	pass_data_t pass_data;

	std::string toCSVlinestring(bool printHeader = false, bool inputOnly = false);
	void fromCSVstring(std::string&);
	void fillData(game_state_e gamestate, const MovingObject& globalBall, const MovingObject& localBall,
			const std::vector<MovingObject>& myTeam, const std::vector<MovingObject>& opponents,
			long controlBallByPlayerId, const std::vector<player_type_e>& teamTypes, const std::vector<long>& robotIds,
			const PlannerOptions& plannerOptions,
			const std::vector<MRA::Geometry::Point>& parking_postions, const FieldConfig& fieldConfig,
			const previous_used_ball_by_planner_t& previous_global_ball,
			const std::vector<final_planner_result_t>& previous_planner_results,
			const ball_pickup_position_t& ball_pickup_position, bool passIsRequired,
			long passBallByPlayerId, const pass_data_t& pass_data,
			const std::vector<double>& time_in_own_penalty_area, const std::vector<double>& time_in_opponent_penalty_area);

	std::string pathToString();
};

} // namespace

#endif /* TEAM_PLANNER_DATA_H */
