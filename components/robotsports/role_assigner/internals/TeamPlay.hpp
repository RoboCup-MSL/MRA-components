/**
 *  @file
 *  @brief   Main Class for role assigner
 *  @curator Jürge van Eijck
 */
#ifndef TEAMPLAY_HPP
#define TEAMPLAY_HPP 1

#include "FieldConfig.hpp"
#include "TeamPlannerParameters.hpp"
#include "GlobalPathPlanner.hpp"
#include "TeamPlannerData.hpp"
#include "TeamPlannerRobot.hpp"
#include "TeamPlannerOpponent.hpp"

#include <vector>

namespace MRA {

class TeamPlay {
public:
    TeamPlay();
    void assign(const TeamPlannerInput& input,
                TeamPlannerState& r_state,
                TeamPlannerOutput& r_output,
                const TeamPlannerParameters& parameters);

    std::vector<PlayerPlannerResult> assign(TeamPlannerData& teamplannerData);

private:
    class AssignToTargetData {
    public:
        bool available;
        double totalCost;
        double distToTarget;
        bool hasPreviousTarget;
        double distToPreviousTarget;
    };

    bool assignAnyToPosition(TeamPlannerData&  teamplanner_data, dynamic_role_e dr_role,
            const MRA::Geometry::Point& target, planner_target_e planner_target, bool role_position_is_end_position_of_pass);

    void calculatePathForRobot(TeamPlannerData&  r_teamplannerData, unsigned idx);

    std::vector<MRA::Geometry::Position> getOpponents(const std::vector<TeamPlannerOpponent>&  Opponents);

    bool check_better_path_found(double& lowest_pathcost, double newPathCost, double fastestPathCost,
                                      const PlayerPlannerResult& new_path, const PlayerPlannerResult& fastest_path,     double equality_cost_threshold );

    void assignGoalie(TeamPlannerData& teamplanner_data);

    void assignTooLongInPenaltyAreaPlayers(TeamPlannerData&  teamplanner_data );

    planner_target_e determine_planner_target(dynamic_role_e dynamic_role, game_state_e gamestate);

    std::vector<TeamPlannerRobot> getTeamMates(const TeamPlannerData& teamplanner_data, unsigned meIdx, bool addAssignedTargetAsTeamPosition);

    bool stayPathWithinBoundaries(const FieldConfig& fieldConfig, const PlayerPlannerResult& result);

    void printAssignInputs(const TeamPlannerData& teamplanner_data);

    double calculateShortestDistanceObjectsToTarget(const std::vector<MRA::Geometry::Position>& objects, const MRA::Geometry::Position& targetObject);

    void ReplanInterceptor(unsigned interceptorIdx, TeamPlannerData&  teamplanner_data);

    bool AssignAnyRobotPreferedSetPlayer(TeamPlannerData&  teamplanner_data, dynamic_role_e dr_role,
                                         planner_target_e planner_target, const MRA::Geometry::Point& targetPos);

    void assignParkingPositions(TeamPlannerData& teamplanner_data);

    void assignBeginPositions(TeamPlannerData& teamplanner_data);

    bool searchForBallBehaviorNeeded(TeamPlannerData& teamplanner_data);

    MRA::Geometry::Point updatePositionIfNotAllowed(const MRA::Geometry::Point& playerPosition, dynamic_role_e dr_role, const MRA::Geometry::Point& original_target_position, const FieldConfig& fieldConfig);

    bool stayInPlayingField(game_state_e gamestate) const;

    int m_gridFileNumber;

    std::vector<dynamic_role_e> getListWithRoles(TeamPlannerData& teamplannerData);
};
} // namespace

#endif // TEAMPLAY_HPP
