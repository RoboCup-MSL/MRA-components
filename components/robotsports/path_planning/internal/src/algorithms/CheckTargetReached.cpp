/*
 * CheckTargetReached.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#include "PathPlanningAlgorithms.hpp"
#include "logging.hpp"

void CheckTargetReached::execute(PathPlanningData &data)
{
    // get delta - TODO simplify
    MRA::Geometry::Position targetPos = data.getSubTarget();
    MRA::Geometry::Position robotPos(data.robot.position.x, data.robot.position.y, data.robot.position.rz);
    MRA::Geometry::Position deltaPositionFcs = targetPos - robotPos;
    deltaPositionFcs.rz = project_angle_mpi_pi(deltaPositionFcs.rz);
    // compare with tolerances
    MRA::Geometry::Point deltaPositionFcsXY(deltaPositionFcs.x, deltaPositionFcs.y);
    bool xyOk = deltaPositionFcsXY.size() < data.configPP.deadzone.toleranceXY;
    bool RzOk = fabs(deltaPositionFcs.rz) < data.configPP.deadzone.toleranceRz;
    MRA_LOG_DEBUG("xyOk=%d RzOk=%d", xyOk, RzOk);
    // convergence criterion, especially useful for testing where overshoot can cause premature 'PASSED'
    static int tickCountTargetReached = 0;
    // update data
    if (xyOk && RzOk)
    {
        tickCountTargetReached++;
        if (tickCountTargetReached >= data.configPP.numExtraSettlingTicks)
        {
            data.resultStatus = MRA::Datatypes::ActionResult::PASSED;
            data.done = true;
        }
    }
    else
    {
        tickCountTargetReached = 0;
        data.resultStatus = MRA::Datatypes::ActionResult::RUNNING;

        MRA_LOG_DEBUG("xyOK=%d -> deltaPositionFcs=%6.2f < toleranceXY=%6.2f", xyOk, data.deltaPositionFcs.xy().size(), data.configPP.deadzone.toleranceXY);
        MRA_LOG_DEBUG("RzOK=%d -> deltaPositionFcs=%8.4f < toleranceRz=%8.4f", RzOk, fabs(deltaPositionFcs.phi), data.configPP.deadzone.toleranceRz);
    }
}

