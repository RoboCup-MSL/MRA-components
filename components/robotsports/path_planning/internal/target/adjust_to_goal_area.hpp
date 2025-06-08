// adjust_to_goal_area.h
#ifndef ADJUST_TO_GOAL_AREA_H
#define ADJUST_TO_GOAL_AREA_H

#include "target.hpp" // Include common DType definition

double sign(double x);
TargetType_t adjust_to_goal_area(const DType_t& d, TargetType_t target);

#endif // ADJUST_TO_GOAL_AREA_H