/*
 * DynamicRoleAssignment.h
 *
 *  Created on: Sep 11, 2016
 *      Author: jurge
 */

#ifndef DYNAMICROLEASSIGNMENT_H
#define DYNAMICROLEASSIGNMENT_H 1

#include <vector>

#include "planner_types.hpp"
#include "TeamPlannerData.hpp"

class DynamicRoleAssignment {
public:
	static void assignDynamicRolesToPlayers(const std::vector<dynamic_role_e>& teamFormation, std::vector<trs::TeamPlannerRobot>& Team);
};



#endif /* DYNAMICROLEASSIGNMENT_H */
