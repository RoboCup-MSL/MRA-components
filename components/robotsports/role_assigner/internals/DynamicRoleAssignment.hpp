/*
 * DynamicRoleAssignment.h
 *
 *  Created on: Sep 11, 2016
 *      Author: jurge
 */

#ifndef DYNAMICROLEASSIGNMENT_HPP
#define DYNAMICROLEASSIGNMENT_HPP 1

#include <vector>

#include "planner_types.hpp"
#include "TeamPlannerData.hpp"

class DynamicRoleAssignment {
public:
    static void assignDynamicRolesToPlayers(const std::vector<MRA::dynamic_role_e>& teamFormation, std::vector<MRA::TeamPlannerRobot>& Team);
};



#endif /* DYNAMICROLEASSIGNMENT_HPP */
