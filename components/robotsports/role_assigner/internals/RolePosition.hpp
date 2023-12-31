/**
 *  @file
 *  @brief   Class for team planning 2017
 *  @curator Jürge van Eijck
 */
#ifndef ROLE_POSITION_H
#define ROLE_POSITION_H 1

#include "WmTypes.h"
#include "FieldConfig.h"
#include <vector>
#include "GlobalPathPlanner.hpp"
#include "planner_types.hpp"
#include "TeamPlannerData.hpp"
#include "TeamPlannerOpponent.hpp"
#include "TeamPlannerParameters.hpp"


namespace MRA {


class RolePosition {
public:
	static MRA::Geometry::Point determineDynamicRolePosition(defend_info_t& rDefend_info, planner_target_e& planner_target, int& r_gridFileNumber,
			dynamic_role_e dynamic_role, game_state_e gamestate,
			const TeamPlannerBall& ball, const TeamPlannerState& r_state,
			std::vector<TeamPlannerRobot>& Team, std::vector<TeamPlannerOpponent>& Opponents,
			const TeamPlannerParameters& plannerOptions, const FieldConfig& fieldConfig,
			const ball_pickup_position_t& ball_pickup_position, bool passIsRequired, bool teamControlBall,
			bool playerPassedBall, const pass_data_t& pass_data, bool& r_role_position_is_end_position_of_pass);

	static void GetFixedPositions(std::vector<MRA::Geometry::Point>& playerPositions, game_state_e gamestate,
			const TeamPlannerBall& ball, bool searchForBall,
			const std::vector<MRA::Geometry::Point>& parking_positions, const FieldConfig& fieldConfig, const TeamPlannerParameters& plannerOptions);

	static MRA::Geometry::Point closestTo(const MRA::Geometry::Point& reference_point, const std::vector<MRA::Geometry::Point>& positions);

private:
	static int FindOpponentClostestToPositionAndNotAssigned(
			const MRA::Geometry::Point& targetPos, const FieldConfig& fieldConfig,
			const TeamPlannerParameters& plannerOptions,
			const std::vector<TeamPlannerOpponent>& Opponents);

	static int FindMostDangerousOpponentAndNotAssigned(
			const MRA::Geometry::Pose& globalBall, const FieldConfig& fieldConfig,
			const TeamPlannerParameters& plannerOptions,
			const std::vector<TeamPlannerOpponent>& Opponents);

	static void getSearchForBallPositions(std::vector<MRA::Geometry::Point>& playerPositions, game_state_e gamestate, const FieldConfig& fieldConfig);

	static void print_provided_position(game_state_e gamestate, const std::vector<std::vector<MRA::Geometry::Point>>& positions);

	static void calculateSetPlayPosition(MRA::Geometry::Point& shooterPosition, MRA::Geometry::Point& receiverPosition,
			const std::vector<TeamPlannerRobot>& Team, const MRA::Geometry::Point& ballPosition, const TeamPlannerState& r_state,
			game_state_e gamestate, const TeamPlannerParameters& plannerOptions, const FieldConfig& fieldConfig);

	static MRA::Geometry::Point calculateSetPlayReceiverPosition(const std::vector<TeamPlannerRobot>& Team,
			                                    const MRA::Geometry::Point& globalBallPosition,
			                                    const TeamPlannerState& r_state,
												game_state_e gamestate,
												const TeamPlannerParameters& plannerOptions, const FieldConfig& fieldConfig);

	static MRA::Geometry::Point InterceptorNormalPlayPosition(planner_target_e& planner_target, const MRA::Geometry::Pose& globalBall,
			const std::vector<TeamPlannerRobot>& Team, const std::vector<TeamPlannerOpponent>& Opponents,
			const TeamPlannerParameters& plannerOptions, const FieldConfig& fieldConfig);
	static MRA::Geometry::Point setplay_receiver_position_90deg_to_ball_goal(const MRA::Geometry::Point& globalBallPosition, const TeamPlannerParameters& plannerOptions, const FieldConfig& fieldConfig);

	static bool calculateSetPlayReceiverMinTurnPosition(const FieldConfig& fieldConfig, const MRA::Geometry::Point& globalBallPosition,
			const TeamPlannerParameters& plannerOptions, MRA::Geometry::Point& receiverPosition);

	static bool calculateSetPlayReceiverOnLobShotLinePosition(const FieldConfig& fieldConfig, const MRA::Geometry::Point& globalBallPosition,
			const TeamPlannerParameters& plannerOptions, MRA::Geometry::Point& receiverPosition);

	static bool calculateSetPlayReceiverConservativePosition(const FieldConfig& fieldConfig, const MRA::Geometry::Point& globalBallPosition,
				const TeamPlannerParameters& plannerOptions, MRA::Geometry::Point& receiverPosition);

	static MRA::Geometry::Point calculateManToManDefensePosition(defend_info_t& rDefend_info, dynamic_role_e dynamic_role,
			const MRA::Geometry::Pose& ball, const FieldConfig& fieldConfig,
			game_state_e gamestate, const TeamPlannerParameters& plannerOptions,
			std::vector<TeamPlannerOpponent>& Opponents, std::vector<TeamPlannerRobot>& Team, int& r_gridFileNumber,
			bool setPlayActive, bool teamControlBall);
};

} // namespace

#endif /* TEAM_PLANNER_H */
