/*
 * ForwardDriving.cpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */

#include "PathPlanningAlgorithms.hpp"
#include "ppgeometry.hpp"
#include "PathPlanning.hpp"
#include "logging.hpp"

void ForwardDriving::execute(PathPlanningData &data)
{
    // get applicable configuration
    ForwardDrivingConfig config = (data.robot.hasBall ? data.configPP.forwardDriving.withBall : data.configPP.forwardDriving.withoutBall);

    // disabled?
    if (!config.enabled)
    {
        return;
    }

    // if there are already sub-targets (typically calculated by obstacle avoidance), then modify their angles
    if (data.path.size() > 1)
    {
        for (int it = 0; it < (int)data.path.size() - 1; ++it)
        {
            // each one must face the next
            auto wp = data.path.at(it);
            auto wpNext = data.path.at(it+1);
            data.path[it].pos.rz = atan2(wpNext.pos.y - wp.pos.y, wpNext.pos.x - wp.pos.x);
        }
    }

    // get sub-target and current robot position
    MRA::Geometry::Position subTarget = data.getSubTarget();
    MRA::Geometry::Position robotPos(data.robot.position.x, data.robot.position.y, data.robot.position.rz);

    // check if distance is large enough
    MRA::Geometry::Pose p(subTarget - robotPos);
    if (MRA::Geometry::Point(p.x, p.y).size() < config.minimumDistance)
    {
        MRA_LOG_DEBUG("too close to sub-target");
        return;
    }

    // insert sub-targets
    MRA::Geometry::Position subTarget1 = faceTowards(robotPos, subTarget.x, subTarget.y);
    subTarget1 = addRcsToFcs(MRA::Geometry::Position(0.0, config.minimumDistance, 0.0), subTarget1);
    data.insertSubTarget(subTarget1);
    /*
    MRA::Geometry::Position subTarget2 = subTarget;
    if ((subTarget - robotPos).xy().size() > 2.0 * config.minimumDistance)
    {
        subTarget2.phi = subTarget1.phi;
        subTarget2 = addRcsToFcs(MRA::Geometry::Position(0.0, -config.minimumDistance, 0.0), subTarget2);
        data.insertSubTarget(subTarget2);
    }*/
}

