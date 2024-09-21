/**
 *  @file
 *  @brief   Class for team planning 2017
 *  @curator Jürge van Eijck
 */
#ifndef RoleAssignerOpponent_HPP
#define RoleAssignerOpponent_HPP 1

namespace MRA {

class RoleAssignerOpponent {
public:
    MRA::Geometry::Position position;
    MRA::Geometry::Position velocity;
    bool assigned = false;
    int label;
};

} // namespace

#endif // RoleAssignerOpponent_HPP
