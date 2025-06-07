// adjust_to_penalty_area.h
#ifndef ADJUST_TO_PENALTY_AREA_H
#define ADJUST_TO_PENALTY_AREA_H

#include "adjust_to_3m_rule.h" // Include common DType definition

double sign(double x); // Re-declare sign function or include a utility header if it's common
DType::TargetType adjust_to_penalty_area(const DType& d, DType::TargetType target);

#endif // ADJUST_TO_PENALTY_AREA_H
