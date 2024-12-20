/**
 *  @file
 *  @brief   Class for team planning 2017
 *  @curator Jürge van Eijck
 */
#ifndef RoleAssignerOpponent_HPP
#define RoleAssignerOpponent_HPP 1

#include "geometry.hpp"

namespace MRA {

class RoleAssignerOpponent {
public:
    MRA::Geometry::Position position = {};
    MRA::Geometry::Position velocity  = {};
    bool assigned = false;
    int trackingId  = -1;

    std::string toString() const;
};

} // namespace

#endif // RoleAssignerOpponent_HPP
