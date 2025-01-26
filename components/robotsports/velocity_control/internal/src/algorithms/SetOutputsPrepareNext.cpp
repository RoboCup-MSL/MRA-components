/*
 * SetOutputsPrepareNext.cpp
 * Started as copy of file with same name in the Falcons variant of Velocity Control
 */

#include "VelocityControlAlgorithms.hpp"


void SetOutputsPrepareNext::execute(VelocityControlData &data)
{
    data.output.mutable_velocity()->set_x(data.resultVelocityRcs.x);
    data.output.mutable_velocity()->set_y(data.resultVelocityRcs.y);
    data.output.mutable_velocity()->set_rz(data.resultVelocityRcs.rz);

    // set diagnostics
    data.diagnostics.set_controlmode(data.controlMode);
    data.diagnostics.set_numalgorithmsexecuted(data.num_algorithms_executed);

    data.diagnostics.mutable_newpositionrcs()->set_x(data.resultPositionRcs.x);
    data.diagnostics.mutable_newpositionrcs()->set_y(data.resultPositionRcs.y);
    data.diagnostics.mutable_newpositionrcs()->set_rz(data.resultPositionRcs.rz);

    data.diagnostics.mutable_newvelocityrcs()->set_x(data.resultVelocityRcs.x);
    data.diagnostics.mutable_newvelocityrcs()->set_y(data.resultVelocityRcs.y);
    data.diagnostics.mutable_newvelocityrcs()->set_rz(data.resultVelocityRcs.rz);
}

