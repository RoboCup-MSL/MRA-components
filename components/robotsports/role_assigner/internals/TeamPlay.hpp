/**
 *  @file
 *  @brief   Class for team planning 2017
 *  @curator Jürge van Eijck
 */
#ifndef TEAMPLAY_H
#define TEAMPLAY_H 1

#include "planner_types.hpp"
#include "FieldConfig.h"
#include "geometry.hpp"
#include "GlobalPathPlanner.hpp"
#include "TeamPlannerData.hpp"
#include "TeamPlannerOpponent.hpp"
#include "TeamPlannerParameters.hpp"
#include <vector>
#include "geometry.hpp"

namespace MRA {


class TeamPlay {
public:
	TeamPlay();
	void assign(const TeamPlannerInput& input,
	            TeamPlannerState& r_state,
	            TeamPlannerOutput& r_output,
	            const TeamPlannerParameters& parameters);

	std::vector<PlayerPlannerResult> assign(TeamPlannerData& teamplannerData); // TODO make private and use other assign

private:
	// std::vector<PlayerPlannerResult> assign(TeamPlannerData& teamplannerData);

	class AssignToTargetData {
	public:
		bool available;
		double totalCost;
		double distToTarget;
		bool hasPreviousTarget;
		double distToPreviousTarget;
	};

	bool assignAnyToPosition(TeamPlannerData&  teamplanner_data, int role_idx, dynamic_role_e dr_role,
			const MRA::Geometry::Point& target, planner_target_e planner_target, bool role_position_is_end_position_of_pass);

	std::vector<MRA::Geometry::Position> getOpponents(const std::vector<TeamPlannerOpponent>&  Opponents);

	bool check_better_path_found(double& lowest_pathcost, double newPathCost, double fastestPathCost,
			 	 	 	 	 	 const PlayerPlannerResult& new_path, const PlayerPlannerResult& fastest_path, 	double equality_cost_threshold );

	void assignGoalie(TeamPlannerData& teamplanner_data);

	void assignTooLongInPenaltyAreaPlayers(TeamPlannerData&  teamplanner_data );

	planner_target_e determine_planner_target(dynamic_role_e dynamic_role, game_state_e gamestate);

	void assignToFixedPositions(TeamPlannerData& teamplanner_data);

	bool searchForBallBehaviorNeeded(TeamPlannerData& teamplanner_data);

	std::vector<MRA::Geometry::Position> getTeamMates(const std::vector<TeamPlannerRobot>& Team, unsigned meIdx, bool addAssignedTargetAsTeamPosition);

	bool stayPathWithinBoundaries(const FieldConfig& fieldConfig, const PlayerPlannerResult& result);

	void printAssignOutputs(const std::vector<TeamPlannerRobot>& Team, const team_planner_result_t&  player_paths);

	void printAssignInputs(const TeamPlannerInput& input, const TeamPlannerParameters& parameters);

	double calculateShortestDistanceObjectsToTarget(const std::vector<MRA::Geometry::Position>& objects, const MRA::Geometry::Position& targetObject);

	void ReplanInterceptor(unsigned interceptorIdx, TeamPlannerData&  teamplanner_data);

	bool AssignAnyRobotPreferedSetPlayer(TeamPlannerData&  teamplanner_data, dynamic_role_e dr_role, planner_target_e planner_target, const MRA::Geometry::Point& targetPos, int role_idx);

	void assignParkingPositions(TeamPlannerData& teamplanner_data);

    void assignBeginPositions(TeamPlannerData& teamplanner_data);


	Geometry::Point updatePositionIfNotAllowed(const Geometry::Point& playerPosition, dynamic_role_e dr_role, const Geometry::Point& original_target_position, const FieldConfig& fieldConfig);

	void calculatePathForRobot(TeamPlannerData& r_teamplannerData, unsigned idx);

	int m_gridFileNumber;
};
} // namespace

#endif /* TEAM_PLANNER_H */
