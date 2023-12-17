/**
 *  @file
 *  @brief   Class for team planning 2017
 *  @curator Jürge van Eijck
 */
#ifndef ROLE_POSITION_H
#define ROLE_POSITION_H 1

#include "MovingObject.h"
#include "WmTypes.h"
#include "FieldConfig.h"
#include <vector>
#include "GlobalPathPlanner.hpp"
#include "planner_types.hpp"
#include "PlannerOptions.hpp"
#include "TeamPlannerData.hpp"
#include "TeamPlannerOpponent.hpp"


namespace trs {


class RolePosition {
public:
	static MRA::Geometry::Point determineDynamicRolePosition(defend_info_t& rDefend_info, planner_target_e& planner_target, int& r_gridFileNumber,
			dynamic_role_e dynamic_role, game_state_e gamestate,
			const MovingObject& globalBall, const previous_used_ball_by_planner_t& previous_global_ball,
			std::vector<TeamPlannerRobot>& Team, std::vector<TeamPlannerOpponent>& Opponents,
			const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig,
			const ball_pickup_position_t& ball_pickup_position, bool passIsRequired, bool teamControlBall,
			bool playerPassedBall, const pass_data_t& pass_data, bool& r_role_position_is_end_position_of_pass);

	static void GetFixedPositions(vector<MRA::Geometry::Point>& playerPositions, game_state_e gamestate,
			const MovingObject& globalBall, bool searchForBall,
			const std::vector<MRA::Geometry::Point>& parking_positions, const FieldConfig& fieldConfig, const PlannerOptions& plannerOptions);

	static MRA::Geometry::Point closestTo(const MRA::Geometry::Point& reference_point, const std::vector<MRA::Geometry::Point>& positions);

private:
	static int FindOpponentClostestToPositionAndNotAssigned(
			const MRA::Geometry::Point& targetPos, const FieldConfig& fieldConfig,
			const PlannerOptions& plannerOptions,
			const std::vector<TeamPlannerOpponent>& Opponents);

	static int FindMostDangerousOpponentAndNotAssigned(
			const MovingObject& globalBall, const FieldConfig& fieldConfig,
			const PlannerOptions& plannerOptions,
			const std::vector<TeamPlannerOpponent>& Opponents);

	static void getSearchForBallPositions(vector<MRA::Geometry::Point>& playerPositions, game_state_e gamestate, const FieldConfig& fieldConfig);

	static void print_provided_position(game_state_e gamestate, const vector<vector<MRA::Geometry::Point>>& positions);

	static void calculateSetPlayPosition(MRA::Geometry::Point& shooterPosition, MRA::Geometry::Point& receiverPosition,
			const std::vector<TeamPlannerRobot>& Team, const MRA::Geometry::Point& ballPosition, const previous_used_ball_by_planner_t& previous_global_ball,
			game_state_e gamestate, const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig);

	static MRA::Geometry::Point calculateSetPlayReceiverPosition(const std::vector<TeamPlannerRobot>& Team,
			                                    const MRA::Geometry::Point& globalBallPosition,
												const previous_used_ball_by_planner_t& previous_global_ball,
												game_state_e gamestate,
												const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig);

	static MRA::Geometry::Point InterceptorNormalPlayPosition(planner_target_e& planner_target, const MovingObject& globalBall,
			const std::vector<TeamPlannerRobot>& Team, const std::vector<TeamPlannerOpponent>& Opponents,
			const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig);
	static MRA::Geometry::Point setplay_receiver_position_90deg_to_ball_goal(const MRA::Geometry::Point& globalBallPosition, const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig);

	static bool calculateSetPlayReceiverMinTurnPosition(const FieldConfig& fieldConfig, const MRA::Geometry::Point& globalBallPosition,
			const PlannerOptions& plannerOptions, MRA::Geometry::Point& receiverPosition);

	static bool calculateSetPlayReceiverOnLobShotLinePosition(const FieldConfig& fieldConfig, const MRA::Geometry::Point& globalBallPosition,
			const PlannerOptions& plannerOptions, MRA::Geometry::Point& receiverPosition);

	static bool calculateSetPlayReceiverConservativePosition(const FieldConfig& fieldConfig, const MRA::Geometry::Point& globalBallPosition,
				const PlannerOptions& plannerOptions, MRA::Geometry::Point& receiverPosition);

	static MRA::Geometry::Point calculateManToManDefensePosition(defend_info_t& rDefend_info, dynamic_role_e dynamic_role,
			const MovingObject& globalBall, const FieldConfig& fieldConfig,
			game_state_e gamestate, const PlannerOptions& plannerOptions,
			std::vector<TeamPlannerOpponent>& Opponents, std::vector<TeamPlannerRobot>& Team, int& r_gridFileNumber,
			bool setPlayActive, bool teamControlBall);
};

} // namespace

#endif /* TEAM_PLANNER_H */
