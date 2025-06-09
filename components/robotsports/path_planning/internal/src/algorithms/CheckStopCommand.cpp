/*
 * CheckStopCommand.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#include "PathPlanningAlgorithms.hpp"
#include "PathPlanning.hpp"

void CheckStopCommand::execute(PathPlanningData &data)
{
    if (data.stop)
    {
        data.resultStatus = actionResultTypeEnum::PASSED;
        data.path.clear();
        
        // create wayPoint on robot itself
        wayPoint stopPosition = wayPoint();
        stopPosition.pos = data.robot.position;
        stopPosition.vel = pose(0.0, 0.0, 0.0);
        data.path.push_back(stopPosition);

        data.done = true;
    }
}

