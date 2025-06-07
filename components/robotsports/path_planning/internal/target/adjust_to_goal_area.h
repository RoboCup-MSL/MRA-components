// adjust_to_goal_area.h
#ifndef ADJUST_TO_GOAL_AREA_H
#define ADJUST_TO_GOAL_AREA_H

#include "adjust_to_3m_rule.h" // Include common DType definition

double sign(double x);
DType::TargetType adjust_to_goal_area(const DType& d, DType::TargetType target);

#endif // ADJUST_TO_GOAL_AREA_H