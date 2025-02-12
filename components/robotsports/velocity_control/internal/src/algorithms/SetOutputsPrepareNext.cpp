/*
 * SetOutputsPrepareNext.cpp
 * Started as copy of file with same name in the Falcons variant of Velocity Control
 */

#include "VelocityControlAlgorithms.hpp"


void SetOutputsPrepareNext::execute(VelocityControlData &data)
{
    data.output.mutable_velocity()->set_x(data.result.velocityRcs.x);
    data.output.mutable_velocity()->set_y(data.result.velocityRcs.y);
    data.output.mutable_velocity()->set_rz(data.result.velocityRcs.rz);

    data.state.set_executed_before(true);
    data.state.mutable_positionsetpointfcs()->set_x(data.previousPositionSetpointFcs.x);
    data.state.mutable_positionsetpointfcs()->set_y(data.previousPositionSetpointFcs.y);
    data.state.mutable_positionsetpointfcs()->set_rz(data.previousPositionSetpointFcs.rz);
    data.state.mutable_velocitysetpointfcs()->set_x(data.previousVelocitySetpointFcs.x);
    data.state.mutable_velocitysetpointfcs()->set_y(data.previousVelocitySetpointFcs.y);
    data.state.mutable_velocitysetpointfcs()->set_rz(data.previousVelocitySetpointFcs.rz);

    // set diagnostics
    data.diagnostics.set_controlmode(data.controlMode);
    data.diagnostics.set_numalgorithmsexecuted(data.num_algorithms_executed);
    data.diagnostics.mutable_newpositionrcs()->set_x(data.ruckig_info.positionRcs.x);
    data.diagnostics.mutable_newpositionrcs()->set_y(data.ruckig_info.positionRcs.y);
    data.diagnostics.mutable_newpositionrcs()->set_rz(data.ruckig_info.positionRcs.rz);
    data.diagnostics.mutable_newvelocityrcs()->set_x(data.ruckig_info.velocityRcs.x);
    data.diagnostics.mutable_newvelocityrcs()->set_y(data.ruckig_info.velocityRcs.y);
    data.diagnostics.mutable_newvelocityrcs()->set_rz(data.ruckig_info.velocityRcs.rz);
    data.diagnostics.mutable_newaccelerationrcs()->set_x(data.ruckig_info.acceleration.x);
    data.diagnostics.mutable_newaccelerationrcs()->set_y(data.ruckig_info.acceleration.y);
    data.diagnostics.mutable_newaccelerationrcs()->set_rz(data.ruckig_info.acceleration.rz);

}


