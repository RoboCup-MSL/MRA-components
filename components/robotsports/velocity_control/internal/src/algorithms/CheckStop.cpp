/*
 * CheckStop.cpp
 *
 *  Created on: May, 2023
 *      Author: Jan Feitsma
 */

#include "VelocityControlAlgorithms.hpp"


void CheckStop::execute(VelocityControlData &data)
{
    if (data.controlMode != MRA::RobotsportsVelocityControl::ControlModeEnum::VEL_ONLY)
    {
        return;
    }
    if (abs(data.targetVelocityFcs.x) < 1e-4 and
        abs(data.targetVelocityFcs.y) < 1e-4 and
        abs(data.targetVelocityFcs.rz) < 1e-4  )
    {
        // wipe state to prevent SPG continuing full throttle after stop
        data.previousPositionSetpointFcs.reset();
        data.previousVelocitySetpointFcs.reset();

        // no need to set output velocity to zero, it is already initialized as such
        data.done = true;
    }
}

