/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */
#ifndef TEAM_PLANNER_ROBOT_HPP
#define TEAM_PLANNER_ROBOT_HPP 1

#include "planner_types.hpp"
#include "geometry.hpp"
#include "TeamPlannerResult.hpp"

namespace MRA {


class RoleAssignerRobot {
public:
    bool active; // participating in the game (robot may be inactive when figuring out where it is)
    bool human;
    long robotId;
    long labelId;  // NEW
    bool controlBall;
    bool passBall; // indicator whether a pass by this player is still on its way
    player_type_e player_type;
    MRA::Geometry::Position position;
    MRA::Geometry::Position velocity;
    double time_in_own_penalty_area;
    double time_in_opponent_penalty_area;

    static bool CompareRobotId(const RoleAssignerRobot& r1, const RoleAssignerRobot&  r2);
    std::string toString() const;
};
} // namespace

#endif // TEAM_PLANNER_ROBOT_HPP
