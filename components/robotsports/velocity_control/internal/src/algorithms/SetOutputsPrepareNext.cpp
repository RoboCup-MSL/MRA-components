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

    data.state.mutable_positionsetpointfcs()->set_x(data.previousPositionSetpointFcs.x);
    data.state.mutable_positionsetpointfcs()->set_y(data.previousPositionSetpointFcs.y);
    data.state.mutable_positionsetpointfcs()->set_rz(data.previousPositionSetpointFcs.rz);
    data.state.mutable_velocitysetpointfcs()->set_x(data.previousVelocitySetpointFcs.x);
    data.state.mutable_velocitysetpointfcs()->set_y(data.previousVelocitySetpointFcs.y);
    data.state.mutable_velocitysetpointfcs()->set_rz(data.previousVelocitySetpointFcs.rz);

    // set diagnostics
    data.diagnostics.set_controlmode(data.controlMode);
    data.diagnostics.set_numalgorithmsexecuted(data.num_algorithms_executed);
}

