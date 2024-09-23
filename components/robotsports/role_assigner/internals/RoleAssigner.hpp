/**
 *  @file
 *  @brief   Main Class for role assigner
 *  @curator Jürge van Eijck
 */
#ifndef ROLEASSIGNER_HPP
#define ROLEASSIGNER_HPP 1

#include "RoleAssigner_types.hpp"
#include "geometry.hpp"
#include <vector>


namespace MRA {

class FieldConfig;
class GlobalPathPlanner;
class RoleAssignerData;

class RoleAssignerOpponent;
class RoleAssignerParameters;
class RoleAssignerRobot;
class RoleAssignerInput;
class RoleAssignerState;
class RoleAssignerOutput;
class RoleAssignerResult;

class RoleAssigner {
public:
    RoleAssigner();
    void assign(const RoleAssignerInput& input,
                RoleAssignerState& r_state,
                RoleAssignerOutput& r_output,
                const RoleAssignerParameters& parameters);

    std::vector<RoleAssignerResult> assign(RoleAssignerData& role_assigner_data);

private:
    class AssignToTargetData {
    public:
        bool available;
        double totalCost;
        double distToTarget;
        bool hasPreviousTarget;
        double distToPreviousTarget;
    };

    bool assignAnyToPosition(RoleAssignerData&  role_assigner_data, dynamic_role_e dr_role,
            const MRA::Geometry::Point& target, planner_target_e planner_target, bool role_position_is_end_position_of_pass,
            role_e org_role);

    void calculatePathForRobot(RoleAssignerData&  r_role_assigner_data, unsigned idx);

    std::vector<MRA::Geometry::Position> getOpponents(const std::vector<RoleAssignerOpponent>&  Opponents);

    bool check_better_path_found(double& lowest_pathcost, double newPathCost, double fastestPathCost,
                                      const RoleAssignerResult& new_path, const RoleAssignerResult& fastest_path,     double equality_cost_threshold );

    void assignGoalie(RoleAssignerData& role_assigner_data);

    void assignTooLongInPenaltyAreaPlayers(RoleAssignerData&  role_assigner_data );

    planner_target_e determine_planner_target(dynamic_role_e dynamic_role, game_state_e gamestate);

    std::vector<RoleAssignerRobot> getTeamMates(const RoleAssignerData& role_assigner_data, unsigned meIdx, bool addAssignedTargetAsTeamPosition);

    bool stayPathWithinBoundaries(const FieldConfig& fieldConfig, const RoleAssignerResult& result);

    void printAssignInputs(const RoleAssignerData& role_assigner_data);

    double calculateShortestDistanceObjectsToTarget(const std::vector<MRA::Geometry::Position>& objects, const MRA::Geometry::Position& targetObject);

    void ReplanInterceptor(unsigned interceptorIdx, RoleAssignerData&  role_assigner_data);

    bool AssignAnyRobotPreferedSetPlayer(RoleAssignerData&  role_assigner_data, dynamic_role_e dr_role,
                                         planner_target_e planner_target, const MRA::Geometry::Point& targetPos, role_e org_role);

    void assignParkingPositions(RoleAssignerData& role_assigner_data);

    void assignBeginPositions(RoleAssignerData& role_assigner_data);

    bool searchForBallBehaviorNeeded(RoleAssignerData& role_assigner_data);

    MRA::Geometry::Point updatePositionIfNotAllowed(const MRA::Geometry::Point& playerPosition, role_e role, const MRA::Geometry::Point& original_target_position, const FieldConfig& fieldConfig);

    bool stayInPlayingField(game_state_e gamestate) const;

    int m_gridFileNumber;

    std::vector<dynamic_role_e> getListWithRoles(RoleAssignerData& role_assigner_data);
};
} // namespace

#endif // ROLEASSIGNER_HPP
