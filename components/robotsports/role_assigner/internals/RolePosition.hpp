/**
 *  @file
 *  @brief   Class for team planning 2017
 *  @curator Jürge van Eijck
 */
#ifndef ROLE_POSITION_HPP
#define ROLE_POSITION_HPP 1

#include "RoleAssigner_types.hpp"
#include "geometry.hpp"

#include <vector>

namespace MRA {

class RoleAssignerData;

class RolePosition {
public:
    static Geometry::Point determineDynamicRolePosition(defend_info_t& rDefend_info, planner_target_e& planner_target, int& r_gridFileNumber,
            MRA::Datatypes::DynamicRole role, RoleAssignerData& r_role_assigner_data, bool playerPassedBall, bool& r_role_position_is_end_position_of_pass);

    static void GetFixedPositions(std::vector<Geometry::Point>& playerPositions, const RoleAssignerData& r_role_assigner_data);

    static MRA::Geometry::Point closestTo(const MRA::Geometry::Point& reference_point, const std::vector<MRA::Geometry::Point>& positions);

    static MRA::Geometry::Point determineSetplayRolePosition_2024(int assignment_nr, defend_info_t& rDefend_info,
                                                                  planner_target_e& planner_target, int& r_gridFileNumber,
                                                                  MRA::Datatypes::DynamicRole role,  RoleAssignerData& r_role_assigner_data,
                                                                  bool playerPassedBall, bool& r_role_position_is_end_position_of_pass);
private:
    static int FindOpponentClostestToPositionAndNotAssigned(const Geometry::Point& targetPos, const RoleAssignerData& r_role_assigner_data);

    static int FindMostDangerousOpponentAndNotAssigned(const RoleAssignerData& r_role_assigner_data);

    static void getSearchForBallPositions(std::vector<Geometry::Point>& playerPositions, const RoleAssignerData& r_role_assigner_data);

    static void print_provided_position(game_state_e gamestate, const std::vector<std::vector<Geometry::Point>>& positions);
    static void calculateSetPlayPosition(Geometry::Point& shooterPosition, Geometry::Point& receiverPosition, const RoleAssignerData& r_role_assigner_data);

    static Geometry::Point calculateSetPlayReceiverPosition(const RoleAssignerData& r_role_assigner_data);

    static Geometry::Point InterceptorNormalPlayPosition(planner_target_e& planner_target, const RoleAssignerData& r_role_assigner_data);
    static Geometry::Point setplay_receiver_position_90deg_to_ball_goal(const RoleAssignerData& r_role_assigner_data);

    static bool calculateSetPlayReceiverMinTurnPosition(const RoleAssignerData& r_role_assigner_data, Geometry::Point& receiverPosition);

    static bool calculateSetPlayReceiverOnLobShotLinePosition(const RoleAssignerData& r_role_assigner_data, Geometry::Point& receiverPosition);

    static bool calculateSetPlayReceiverConservativePosition(const RoleAssignerData& r_role_assigner_data, Geometry::Point& receiverPosition);


    static MRA::Geometry::Point calculateManToManDefensePosition(
            defend_info_t& rDefend_info, MRA::Datatypes::DynamicRole role,
            RoleAssignerData& r_role_assigner_data, int& r_gridFileNumber, bool setPlayActive);
    static Geometry::Point calculateSetPlayerKickerPosition(const Geometry::Point& receiverPosition, const RoleAssignerData& r_role_assigner_data);
};


} // namespace

#endif // ROLE_POSITION_HPP
