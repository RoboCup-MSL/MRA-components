/**
 *  @file
 *  @brief   Class for team planning 2017
 *  @curator Jürge van Eijck
 */
#ifndef ROLE_POSITION_H
#define ROLE_POSITION_H 1

#include "MovingObject.h"
#include "PlannerOptions.h"
#include "GlobalPathPlanner.h"
#include "WmTypes.h"
#include "planner_types.h"
#include "FieldConfig.h"
#include "TeamPlannerData.h"
#include "TeamPlannerOpponent.h"
#include <vector>


namespace trs {


class RolePosition {
public:
	static Vector2D determineDynamicRolePosition(defend_info_t& rDefend_info, planner_target_e& planner_target, int& r_gridFileNumber,
			dynamic_role_e dynamic_role, game_state_e gamestate,
			const MovingObject& globalBall, const MovingObject& localBall, const previous_used_ball_by_planner_t& previous_global_ball,
			std::vector<TeamPlannerRobot>& Team, std::vector<TeamPlannerOpponent>& Opponents,
			const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig,
			const ball_pickup_position_t& ball_pickup_position, bool passIsRequired, bool teamControlBall,
			bool playerPassedBall, const pass_data_t& pass_data, bool& r_role_position_is_end_position_of_pass);

	static void GetFixedPositions(vector<Vector2D>& playerPositions, game_state_e gamestate,
			const MovingObject& globalBall, bool searchForBall,
			const std::vector<Vector2D>& parking_positions, const FieldConfig& fieldConfig, const PlannerOptions& plannerOptions);

private:
	static int FindOpponentClostestToPositionAndNotAssigned(
			const Vector2D& targetPos, const FieldConfig& fieldConfig,
			const PlannerOptions& plannerOptions,
			const std::vector<TeamPlannerOpponent>& Opponents);

	static int FindMostDangerousOpponentAndNotAssigned(
			const MovingObject& globalBall, const FieldConfig& fieldConfig,
			const PlannerOptions& plannerOptions,
			const std::vector<TeamPlannerOpponent>& Opponents);

	static void getSearchForBallPositions(vector<Vector2D>& playerPositions, game_state_e gamestate, const FieldConfig& fieldConfig);

	static void print_provided_position(game_state_e gamestate, const vector<vector<Vector2D>>& positions);

	static void calculateSetPlayPosition(Vector2D& shooterPosition, Vector2D& receiverPosition,
			const std::vector<TeamPlannerRobot>& Team, const Vector2D& ballPosition, const previous_used_ball_by_planner_t& previous_global_ball,
			game_state_e gamestate, const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig);

	static Vector2D calculateSetPlayReceiverPosition(const std::vector<TeamPlannerRobot>& Team,
			                                    const Vector2D& globalBallPosition,
												const previous_used_ball_by_planner_t& previous_global_ball,
												game_state_e gamestate,
												const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig);

	static Vector2D InterceptorNormalPlayPosition(planner_target_e& planner_target, const MovingObject& globalBall,
			const std::vector<TeamPlannerRobot>& Team, const std::vector<TeamPlannerOpponent>& Opponents,
			const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig);
	static Vector2D setplay_receiver_position_90deg_to_ball_goal(const Vector2D& globalBallPosition, const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig);

	static bool calculateSetPlayReceiverMinTurnPosition(const FieldConfig& fieldConfig, const Vector2D& globalBallPosition,
			const PlannerOptions& plannerOptions, Vector2D& receiverPosition);

	static bool calculateSetPlayReceiverOnLobShotLinePosition(const FieldConfig& fieldConfig, const Vector2D& globalBallPosition,
			const PlannerOptions& plannerOptions, Vector2D& receiverPosition);

	static bool calculateSetPlayReceiverConservativePosition(const FieldConfig& fieldConfig, const Vector2D& globalBallPosition,
				const PlannerOptions& plannerOptions, Vector2D& receiverPosition);

	static Vector2D calculateManToManDefensePosition(defend_info_t& rDefend_info, dynamic_role_e dynamic_role,
			const MovingObject& globalBall, const FieldConfig& fieldConfig,
			game_state_e gamestate, const PlannerOptions& plannerOptions,
			std::vector<TeamPlannerOpponent>& Opponents, std::vector<TeamPlannerRobot>& Team, int& r_gridFileNumber,
			bool setPlayActive, bool teamControlBall);
};

} // namespace

#endif /* TEAM_PLANNER_H */
