#ifndef PLANNER_COMMON_H
#define PLANNER_COMMON_H

#include "Vector2D.h"
#include "MovingObject.h"
#include <vector>


double chance_of_intercept(const trs::Vector2D& from, const trs::Vector2D& to, const vector<trs::MovingObject>& Opponents,
			double interceptionChanceStartDistance, double interceptionChanceIncreasePerMeter,double interceptionChancePenaltyFactor);
#endif
