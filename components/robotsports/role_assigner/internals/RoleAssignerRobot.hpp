/**
 *  @file
 *  @brief   Class for role assigner robot (data class)
 *  @curator Jürge van Eijck
 */
#ifndef ROLE_ASSIGNER_ROBOT_HPP
#define ROLE_ASSIGNER_ROBOT_HPP 1

#include "geometry.hpp"
#include "RoleAssigner_types.hpp"

namespace MRA {


class RoleAssignerRobot {
public:
    bool active; // participating in the game (robot may be inactive when figuring out where it is)
    bool human;
    long robotId;
    long trackingId;
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

#endif // ROLE_ASSIGNER_ROBOT_HPP