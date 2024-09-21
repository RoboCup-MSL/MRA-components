/**
 *  @file
 *  @brief   Class for team planning 2017
 *  @curator Jürge van Eijck
 */
#ifndef ROLE_POSITION_HPP
#define ROLE_POSITION_HPP 1

#include "FieldConfig.hpp"
#include "GlobalPathPlanner.hpp"
#include "planner_types.hpp"
#include "TeamPlannerData.hpp"
#include "TeamPlannerOpponent.hpp"
#include <vector>
#include "RoleAssignerParameters.hpp"

namespace MRA {


class RolePosition {
public:
    static Geometry::Point determineDynamicRolePosition(defend_info_t& rDefend_info, planner_target_e& planner_target, int& r_gridFileNumber,
            dynamic_role_e dynamic_role, TeamPlannerData& r_teamplannerData, bool playerPassedBall, bool& r_role_position_is_end_position_of_pass);

    static void GetFixedPositions(std::vector<Geometry::Point>& playerPositions, const TeamPlannerData& r_teamplannerData);

    static MRA::Geometry::Point closestTo(const MRA::Geometry::Point& reference_point, const std::vector<MRA::Geometry::Point>& positions);

    static MRA::Geometry::Point determineSetplayRolePosition_2024(int assignment_nr, defend_info_t& rDefend_info,
                                                                  planner_target_e& planner_target, int& r_gridFileNumber,
                                                                  dynamic_role_e dynamic_role,  TeamPlannerData& r_teamplannerData,
                                                                  bool playerPassedBall, bool& r_role_position_is_end_position_of_pass);
private:
    static int FindOpponentClostestToPositionAndNotAssigned(const Geometry::Point& targetPos, const TeamPlannerData& r_teamplannerData);

    static int FindMostDangerousOpponentAndNotAssigned(const TeamPlannerData& r_teamplannerData);

    static void getSearchForBallPositions(std::vector<Geometry::Point>& playerPositions, const TeamPlannerData& r_teamplannerData);

    static void print_provided_position(game_state_e gamestate, const std::vector<std::vector<Geometry::Point>>& positions);
    static void calculateSetPlayPosition(Geometry::Point& shooterPosition, Geometry::Point& receiverPosition, const TeamPlannerData& r_teamplannerData);

    static Geometry::Point calculateSetPlayReceiverPosition(const TeamPlannerData& r_teamplannerData);

    static Geometry::Point InterceptorNormalPlayPosition(planner_target_e& planner_target, const TeamPlannerData& r_teamplannerData);
    static Geometry::Point setplay_receiver_position_90deg_to_ball_goal(const TeamPlannerData& r_teamplannerData);

    static bool calculateSetPlayReceiverMinTurnPosition(const TeamPlannerData& r_teamplannerData, Geometry::Point& receiverPosition);

    static bool calculateSetPlayReceiverOnLobShotLinePosition(const TeamPlannerData& r_teamplannerData, Geometry::Point& receiverPosition);

    static bool calculateSetPlayReceiverConservativePosition(const TeamPlannerData& r_teamplannerData, Geometry::Point& receiverPosition);


    static MRA::Geometry::Point calculateManToManDefensePosition(
            defend_info_t& rDefend_info, dynamic_role_e dynamic_role,
            TeamPlannerData& r_teamplannerData, int& r_gridFileNumber, bool setPlayActive);
    static Geometry::Point calculateSetPlayerKickerPosition(const Geometry::Point& receiverPosition, const TeamPlannerData& r_teamplannerData);
};


} // namespace

#endif // ROLE_POSITION_HPP
