/*
 * CalculateVelocity.cpp
 * Started as copy of file with same name in the Falcons variant of Velocity Control
 */

#include "VelocityControlAlgorithms.hpp"
#include "VelocitySetpointControllers.hpp"


void CalculateVelocity::execute(VelocityControlData &data)
{
    // check that some controller is configured
    assert(data.controller != NULL);

    // call, check result
    // the controller should write to data.resultVelocityRcs
    bool success = data.controller->calculate(data);
    if (!success)
    {
        throw std::runtime_error("something went wrong in velocity controller");
    }

    // OPTIONAL workaround for non-convergence (mainly applies to simulation):
    // it can happen that Reflexxes outputs v==0 when Rz exceeds tolerance
    // (just disable this workaround and watch some test cases fail / motionPlanning move sometimes does not finish)
    // no flags or options seem to be available to fix this behavior ...
    // as workaround, final step is calculated by "perfect" linear controller
    // better ideas are welcome!
    // newer Reflexxes implementation seems inaccessible, tried to email author, no response
    if (data.config.spg().convergenceworkaround())
    {
        float tolerance = 1e-5; // TODO make configurable
        // is the delta small enough?
        Position2D deltaPosition = data.targetPositionFcs - data.currentPositionFcs;
        bool xyDeltaSmallEnough = ((abs(deltaPosition.x) < tolerance) && (abs(deltaPosition.y) < tolerance));
        bool rzDeltaSmallEnough = (abs(deltaPosition.rz) < tolerance);
        // is SPG converged?
        bool xySPGConverged = ((abs(data.result.velocityRcs.x) < tolerance) && (abs(data.result.velocityRcs.y) < tolerance));
        bool rzSPGConverged = (abs(data.result.velocityRcs.rz) < tolerance);
        // run linear controller
        VelocityControlData tmpdata = data;
        LinearVelocitySetpointController().calculate(tmpdata);
        // overrule XY?
        if (xySPGConverged && !xyDeltaSmallEnough)
        {
            data.result.velocityRcs.x = tmpdata.result.velocityRcs.x;
            data.result.velocityRcs.y = tmpdata.result.velocityRcs.y;
        }
        // overrule Rz?
        if (rzSPGConverged && !rzDeltaSmallEnough)
        {
            data.result.velocityRcs.rz = tmpdata.result.velocityRcs.rz;
        }
    }
}

