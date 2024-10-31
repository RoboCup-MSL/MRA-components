/*
 * SelectVelocityController.cpp
 * Started as copy of file with same name in the Falcons variant of Velocity Control
 */

#include "VelocityControlAlgorithms.hpp"
#include "AbstractVelocitySetpointController.hpp"
#include "VelocitySetpointControllers.hpp"


void SelectVelocityController::execute(VelocityControlData &data)
{
    data.controller = std::shared_ptr<AbstractVelocitySetpointController>(new SPGVelocitySetpointController());
}

