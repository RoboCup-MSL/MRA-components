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
#include "TeamPlannerData.hpp"
#include "TeamPlannerOpponent.hpp"
#include "TeamPlannerParameters.hpp"


namespace MRA {


class TeamPlay {
public:
	TeamPlay();
	void assign(const TeamPlannerInput& input,
	            TeamPlannerState& r_state,
	            TeamPlannerOutput& r_output,
	            const TeamPlannerParameters& plannerOptions);

private:
	class AssignToTargetData {
	public:
		unsigned team_idx;
		long robotId;
		double distToTarget;
		bool hasPreviousTarget;
		double distToPreviousTarget;
	};

	bool assignAnyToPosition(TeamPlannerData&  teamplanner_data, int role_idx, dynamic_role_e dr_role,
			const MRA::Geometry::Point& target, planner_target_e planner_target, bool role_position_is_end_position_of_pass);

	vector<MovingObject> getOpponents(const std::vector<TeamPlannerOpponent>&  Opponents);

	bool check_better_path_found(double& lowest_pathcost, double newPathCost, double fastestPathCost,
			 	 	 	 	 	 const PlayerPlannerResult& new_path, const PlayerPlannerResult& fastest_path, 	double equality_cost_threshold );

	void assignGoalie(TeamPlannerData& teamplanner_data);

	void assignTooLongInPenaltyAreaPlayers(TeamPlannerData&  teamplanner_data );

	planner_target_e determine_planner_target(dynamic_role_e dynamic_role, game_state_e gamestate);

	void assignToFixedPositions(TeamPlannerData&  teamplanner_data, unsigned playerlist_idx, dynamic_role_e dynamic_role);

	bool searchForBallBehaviorNeeded(TeamPlannerData& teamplanner_data);

	void print_provided_position(game_state_e gamestate, const vector<vector<MRA::Geometry::Point>>& positions);

	vector<MovingObject> getTeamMates(const std::vector<TeamPlannerRobot>& Team, unsigned meIdx, bool addAssignedTargetAsTeamPosition);

	template<class T> bool safeErase(std::vector<T>& myvector, unsigned int indexElementToErase, int callingFromLine);

	bool stayPathWithinBoundaries(const FieldConfig& fieldConfig, const PlayerPlannerResult& result);

	void printAssignOutputs(const std::vector<TeamPlannerRobot>& Team, team_planner_result_t&  player_paths);

	void printAssignInputs(TeamPlannerData&  teamplanner_data);

	double calculateShortestDistanceObjectsToTarget(const std::vector<MovingObject>& objects, const MovingObject& targetObject);

	void ReplanInterceptor(unsigned interceptorIdx, TeamPlannerData&  teamplanner_data);

	bool AssignAnyRobotPreferedSetPlayer(TeamPlannerData&  teamplanner_data, dynamic_role_e dr_role, planner_target_e planner_target, const MRA::Geometry::Point& targetPos);

	int m_gridFileNumber;
};
} // namespace

#endif /* TEAM_PLANNER_H */
