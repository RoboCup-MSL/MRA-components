#ifndef PLANNER_COMMON_H
#define PLANNER_COMMON_H

#include "geometry.hpp"
#include "MovingObject.h"
#include <vector>


double chance_of_intercept(const MRA::Geometry::Point& from, const MRA::Geometry::Point& to, const vector<trs::MovingObject>& Opponents,
			double interceptionChanceStartDistance, double interceptionChanceIncreasePerMeter,double interceptionChancePenaltyFactor);
#endif
