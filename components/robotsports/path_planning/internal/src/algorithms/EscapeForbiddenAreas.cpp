/*
 * EscapeForbiddenAreas.cpp
 *
 *  Created on: November, 2019
 *      Author: Jan Feitsma
 */

#include "PathPlanningAlgorithms.hpp"
#include "logging.hpp"

void EscapeForbiddenAreas::execute(PathPlanningData &data)
{
    // crude but effective algorithm: in case robot is inside ANY forbidden area, just move to zero
    // (of course assuming there is no forbidden area at zero...)
    // ROADMAP: make it smarter, by choosing some point outside forbidden area, which minimizes path length to target
    MRA::Geometry::Point r(data.robot.position.x, data.robot.position.y);
    for (auto it = data.forbiddenAreas.begin(); it != data.forbiddenAreas.end(); ++it)
    {
        // ignore forbidden area if robot is inside, otherwise it cannot escape
        if (it->isPointInside(r))
        {
            data.insertSubTarget(MRA::Geometry::Position(0, 0, data.robot.position.rz));
            MRA_LOG_DEBUG("escaping forbidden area id=%d", it->id);
            break;
        }
    }
}

