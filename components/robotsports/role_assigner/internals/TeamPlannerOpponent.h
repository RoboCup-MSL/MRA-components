/**
 *  @file
 *  @brief   Class for team planning 2017
 *  @curator Jürge van Eijck
 */
#ifndef TeamPlannerOpponent_H
#define TeamPlannerOpponent_H 1
#include "MovingObject.h"

namespace trs {

class TeamPlannerOpponent {
public:
	MovingObject position;
	bool assigned;
};

} // namespace

#endif /* TeamPlannerOpponent_H */
