// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "FalconsActionAimedKick.hpp"

using namespace MRA;

// custom includes, if any
#include "geometry.hpp"


int FalconsActionAimedKick::FalconsActionAimedKick::tick
(
    google::protobuf::Timestamp timestamp,   // absolute timestamp
    InputType  const           &input,       // input data, type generated from Input.proto
    ParamsType const           &params,      // configuration parameters, type generated from Params.proto
    StateType                  &state,       // state data, type generated from State.proto
    OutputType                 &output,      // output data, type generated from Output.proto
    LocalType                  &local        // local/diagnostics data, type generated from Local.proto
)
{
    int error_value = 0;
    MRA_LOG_TICK();

    // user implementation goes here

    auto const ws = input.worldstate();

    if (!ws.robot().hasball())
    {
        // TODO: robustness: use state - it can happen that the robot kicked the ball away, but it takes a tick or more for the ball to actually leave?
        // in that case, functionally the shot was a success, so we should not produce FAILED
        output.set_actionresult(MRA::Datatypes::FAILED);
        return 0;
    }

    // initialize output before failure-mode checks
    output.set_actionresult(MRA::Datatypes::RUNNING);

    // configurables
    // TODO: make configurable via Params
    float angleAccuracyThreshold = 0.01;

    // calculate rotation angle
    MRA::Geometry::Position robotPos = ws.robot().position();
    MRA::Geometry::Position ballTargetPos = input.target().position();
    ballTargetPos.faceAwayFrom(robotPos);
    MRA::Geometry::Position delta = ballTargetPos - robotPos;
    float remainingRotationAngle = delta.rz;

    // TODO: intentional overshoot?

    // set ballhandlers, always
    output.set_bhenabled(true);
    // TODO: disable just before shot, for improved accuracy/power?

    // determine when to shoot
    if (abs(remainingRotationAngle) < angleAccuracyThreshold)
    {
        // shoot, success
        // TODO: calculate timeToKick, tune it?
        output.set_dokick(true);
        output.set_actionresult(MRA::Datatypes::PASSED);
    }
    else
    {
        // prepare, aim, running
        *output.mutable_motiontarget()->mutable_position() = ws.robot().position();
        output.mutable_motiontarget()->mutable_position()->set_rz(ballTargetPos.rz);
        *output.mutable_balltarget() = input.target().position();
        output.set_actionresult(MRA::Datatypes::RUNNING);
    }

    // store diagnostics data
    local.set_remainingrotationangle(remainingRotationAngle);

    return error_value;
}

