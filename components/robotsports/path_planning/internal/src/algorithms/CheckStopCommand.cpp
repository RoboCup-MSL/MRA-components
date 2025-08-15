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
        data.resultStatus = MRA::Datatypes::ActionResult::PASSED;
        data.path.clear();
        
        // create wayPoint on robot itself
        wayPoint_t stopPosition = {};
        stopPosition.pos = data.robot.position;
        stopPosition.vel = {};
        data.path.push_back(stopPosition);

        data.done = true;
    }
}

