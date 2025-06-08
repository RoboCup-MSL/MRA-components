// adjust_to_penalty_area.h
#ifndef ADJUST_TO_PENALTY_AREA_H
#define ADJUST_TO_PENALTY_AREA_H

#include "target.hpp" // Include common DType definition

double sign(double x); // Re-declare sign function or include a utility header if it's common
TargetType_t adjust_to_penalty_area(const DType_t& d, TargetType_t target);

#endif // ADJUST_TO_PENALTY_AREA_H
