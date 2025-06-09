/*
 * RequireWorldModelActive.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#include "PathPlanningAlgorithms.hpp"


void RequireWorldModelActive::execute(PathPlanningData &data)
{
    bool wmActive = (data.robot.status == robotStatusEnum::INPLAY);
    if (!wmActive)
    {
        data.resultStatus = MRA::Datatypes::ActionResult::FAILED;
        data.stop = true;
        data.done = true;
    }
}

