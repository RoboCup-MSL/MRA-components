// set.h
#ifndef TARGET_H
#define TARGET_H

#include "adjust_to_3m_rule.h"
#include "adjust_to_field.h"
#include "adjust_to_goal_area.h"
#include "adjust_to_obstacles.h"
#include "adjust_to_penalty_area.h"

// The DType struct is defined in adjust_to_3m_rule.h,
// so no need to redefine it here.

DType Target(DType d);

#endif // TARGET_H