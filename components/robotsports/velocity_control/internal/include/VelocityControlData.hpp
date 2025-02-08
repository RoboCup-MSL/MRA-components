/*
 * VelocityControlData.hpp
 * Started as copy of file with same name in the Falcons variant of Velocity Control
 */

#ifndef ROBOTSPORTS_VELOCITYCONTROL_VELOCITYCONTROLDATA_HPP_
#define ROBOTSPORTS_VELOCITYCONTROL_VELOCITYCONTROLDATA_HPP_

// MRA libraries
#include "MRAbridge.hpp"

// forward declaration
class AbstractVelocitySetpointController;

namespace MRA::internal::RVC
{

struct VelocityControlResult {
    Velocity2D velocityRcs;
    Velocity2D positionRcs;
    Pose2D acceleration;
};

// this struct is used (r/w) by every algorithm
struct VelocityControlData
{
    // MRA interface
    MRA_timestamp      timestamp;
    MRA_InputType      input;
    MRA_ParamsType     config;
    MRA_StateType      state;
    MRA_DiagnosticsType diagnostics;
    MRA_OutputType     output;

    // while running the sequence of algorithms, this flag may be raised
    bool done;

    // which controller to use, determined by the SelectVelocityController algorithm
    std::shared_ptr<AbstractVelocitySetpointController> controller;

    // set the limits based on full configuration and input
    MRA::RobotsportsVelocityControl::Limits limits;

    // internal variables
    int num_algorithms_executed;
    MRA::RobotsportsVelocityControl::ControlModeEnum controlMode;
    Position2D currentPositionFcs;
    Velocity2D currentVelocityFcs;
    Position2D targetPositionFcs;
    Velocity2D targetVelocityFcs;
    Position2D previousPositionSetpointFcs;
    Velocity2D previousVelocitySetpointFcs;

    // result
    VelocityControlResult result;
    VelocityControlResult ruckig_info;

};

} // namespace MRA::internal::RVC

using namespace MRA::internal::RVC; //HACK

#endif

