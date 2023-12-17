/**
 *  @file
 *  @brief   Class for team planning 2017
 *  @curator Jürge van Eijck
 */
#ifndef TEAMPLAY_H
#define TEAMPLAY_H 1

#include "MovingObject.h"
#include "WmTypes.h"
#include "FieldConfig.h"
#include <vector>
#include "GlobalPathPlanner.hpp"
#include "PlannerOptions.hpp"
#include "TeamPlannerData.hpp"
#include "TeamPlannerOpponent.hpp"


namespace trs {


class TeamPlay {
public:
	TeamPlay();
	void assign(const TeamPlannerInput& input,
	            TeamPlannerState& r_state,
	            TeamPlannerOutput& r_output,
	            const PlannerOptions& plannerOptions);

private:
	class AssignToTargetData {
	public:
		unsigned team_idx;
		long robotId;
		double distToTarget;
		bool hasPreviousTarget;
		double distToPreviousTarget;
	};

	vector<dynamic_role_e> selectTeamFormation(game_state_e gamestate);

	bool assignAnyToPosition(int role_idx, dynamic_role_e dr_role, game_state_e gamestate,
			const MovingObject& globalBall,
			std::vector<TeamPlannerRobot>& Team,
			const std::vector<TeamPlannerOpponent>& Opponents,
			const MRA::Geometry::Point& target, bool ballIsObstacle, planner_target_e planner_target,
			const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig, const defend_info_t& Defend_infob,
			bool role_position_is_end_position_of_pass, const pass_data_t& pass_data);

	vector<MovingObject> getOpponents(const std::vector<TeamPlannerOpponent>&  Opponents);

	bool check_better_path_found(double& lowest_pathcost, double newPathCost, double fastestPathCost,
			 	 	 	 	 	 const PlayerPlannerResult& new_path, const PlayerPlannerResult& fastest_path, 	double equality_cost_threshold );

	void assignGoalie(game_state_e gamestate, std::vector<TeamPlannerRobot>& Team, bool ballIsObstacle,
			const MovingObject& globalBall, const std::vector<TeamPlannerOpponent>& Opponents, const PlannerOptions& plannerOptions,
			const FieldConfig& m_fieldConfig, const std::vector<MRA::Geometry::Point>& parking_positions);

	void assignTooLongInPenaltyAreaPlayers(game_state_e gamestate, std::vector<TeamPlannerRobot>& Team,
			bool ballIsObstacle, const MovingObject& globalBall, const std::vector<TeamPlannerOpponent>& Opponents,
			const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig);

	planner_target_e determine_planner_target(dynamic_role_e dynamic_role, game_state_e gamestate);

	void assignToFixedPositions(unsigned playerlist_idx, dynamic_role_e dynamic_role, game_state_e gamestate, std::vector<TeamPlannerRobot>& Team, const std::vector<TeamPlannerOpponent>& Opponents,
			const MovingObject& globalBall, bool ballIsObstacle, const std::vector<MRA::Geometry::Point>& parking_positions,
			const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig, bool searchForBall,
			const defend_info_t& Defend_info, const pass_data_t& pass_data);

	bool searchForBallBehaviorNeeded(game_state_e gamestate, const MovingObject& globalBall, const FieldConfig& fieldConfig);

	void print_provided_position(game_state_e gamestate, const vector<vector<MRA::Geometry::Point>>& positions);

	void fillProvidePositions(vector<vector<MRA::Geometry::Point>>& playerPositions, game_state_e gamestate, const MovingObject& globalBall,
			bool searchForBall,const std::vector<MRA::Geometry::Point>& parking_positions);

	vector<MovingObject> getTeamMates(const std::vector<TeamPlannerRobot>& Team, unsigned meIdx, bool addAssignedTargetAsTeamPosition);

	template<class T> bool safeErase(std::vector<T>& myvector, unsigned int indexElementToErase, int callingFromLine);

	bool stayPathWithinBoundaries(const FieldConfig& fieldConfig, const PlayerPlannerResult& result);

	void printAssignOutputs(const std::vector<TeamPlannerRobot>& Team, team_planner_result_t&  player_paths);

	void printAssignInputs(game_state_e gamestate, const MovingObject& globalBall, std::vector<TeamPlannerRobot>& Team, const std::vector<TeamPlannerOpponent>& Opponents,
			const PlannerOptions& plannerOptions, const std::vector<MRA::Geometry::Point>& parking_positions,
			const ball_pickup_position_t& ball_pickup_position, bool passIsRequired, const pass_data_t& pass_data);

	double calculateShortestDistanceObjectsToTarget(const std::vector<MovingObject>& objects, const MovingObject& targetObject);

	void ReplanInterceptor(const MovingObject& Ball,
			unsigned interceptorIdx,
			std::vector<TeamPlannerRobot>& Team, const std::vector<TeamPlannerOpponent>& Opponents,
			const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig, bool ballIsObstacle);

	bool AssignAnyRobotPreferedSetPlayer(dynamic_role_e dr_role,
			const PlannerOptions& plannerOptions, game_state_e game_state,
			const MovingObject& globalBall,
			const std::vector<TeamPlannerOpponent>& Opponents,
			planner_target_e planner_target, bool ballIsObstacle,
			const MRA::Geometry::Point& targetPos, std::vector<TeamPlannerRobot>& Team);

	int m_gridFileNumber;
};
} // namespace

#endif /* TEAM_PLANNER_H */
