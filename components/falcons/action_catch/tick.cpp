// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "FalconsActionCatch.hpp"

using namespace MRA;

// custom includes, if any
#include "geometry.hpp"


int FalconsActionCatch::FalconsActionCatch::tick
(
    google::protobuf::Timestamp timestamp,   // absolute timestamp
    InputType  const           &input,       // input data, type generated from Input.proto
    ParamsType const           &params,      // configuration parameters, type generated from Params.proto
    StateType                  &state,       // state data, type generated from State.proto
    OutputType                 &output,      // output data, type generated from Output.proto
    DiagnosticsType            &diagnostics  // diagnostics data, type generated from Diagnostics.proto
)
{
    int error_value = 0;
    MRA_LOG_TICK();

    output.Clear();
    diagnostics.Clear();

    // user implementation goes here

/*

these are the protobuf parameters in Params.proto


package MRA.FalconsActionCatch;

message Params
{
    // only respond to ball trajectory if it is expected to come close enough
    // if too far, then action will return FAILED
    float captureRadius = 1; // [m]

    // only respond to balls which are coming sufficiently fast
    // if too slow, then action will return FAILED
    float ballSpeedThreshold = 2; // [m/s]

    // default/legacy behavior is to move sideways ("strafe")
    // proactive mode will optimize, which leads to forward movemement when ball is approaching slow, which is useful when opponent is nearby
    bool proactive = 3;

    // assumed robot speed to use when calculation ball interception point
    // only used in proactive mode
    // (roadmap: get this configuration from some other config file or even runtime, using trajectory generator?)
    float robotCatchSpeed = 4; // [m/s]

    // extrapolate ball vector with this duration, similar to 'getBallOnVector'
    // only used in proactive mode
    float ballVectorExtrapolation = 5; // [s]
}

for now, do not implement proactive mode, just the basic functionality

*/
/* 
    fix these errors and remove them from this comment


*/
    try
    {
        // Always enable ballhandlers
        output.set_bhenabled(true);

        // Check if the ball speed is above the threshold
        MRA::Geometry::Velocity ball_velocity = input.worldstate().ball().velocity();
        float ball_speed = ball_velocity.size();
        bool ballmovingfastenough = ball_speed > params.ballspeedthreshold();
        if (!ballmovingfastenough)
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            return error_value;
        }

        // Check if the ball is moving towards the robot
        bool ballmovingtowardsrobot = input.worldstate().ball().velocity().x() < 0;
        if (!ballmovingtowardsrobot)
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            return error_value;
        }

        // Check if the ball is within capture range
        float ball_distance = sqrt(pow(input.worldstate().ball().position().x(), 2) + pow(input.worldstate().ball().position().y(), 2));
        bool ballmovingwithincapturerange = ball_distance < params.captureradius();
        if (!ballmovingwithincapturerange)
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            return error_value;
        }

        // Calculate the time to intercept based on ball speed and robot speed
        float ball_speed_norm = sqrt(pow(input.worldstate().ball().velocity().x(), 2) + pow(input.worldstate().ball().velocity().y(), 2));
        float time_to_catch = ball_distance / (ball_speed_norm + params.robotcatchspeed());

        // Extrapolate ball position to the interception point
        MRA::Datatypes::PosVel interceptionPoint;
        interceptionPoint.mutable_position()->set_x(input.worldstate().ball().position().x() + input.worldstate().ball().velocity().x() * time_to_catch);
        interceptionPoint.mutable_position()->set_y(input.worldstate().ball().position().y() + input.worldstate().ball().velocity().y() * time_to_catch);
        interceptionPoint.mutable_position()->set_z(input.worldstate().ball().position().z() + input.worldstate().ball().velocity().z() * time_to_catch);

        // Set the calculated interception point as the motion target
        output.mutable_motiontarget()->CopyFrom(interceptionPoint);
        output.set_actionresult(MRA::Datatypes::ActionResult::PASSED);

        // Set diagnostics values
        diagnostics.set_ballmovingtowardsrobot(ballmovingtowardsrobot);
        diagnostics.set_ballmovingfastenough(ballmovingfastenough);
        diagnostics.set_ballmovingwithincapturerange(ballmovingwithincapturerange);
        diagnostics.set_timetocatch(time_to_catch);
    }
    catch (const std::exception& e)
    {
        MRA_LOG_ERROR("ERROR: Caught a standard exception: %s", e.what());
        error_value = -1;
    }
    catch (...)
    {
        MRA_LOG_ERROR("ERROR: Caught an unknown exception.");
        error_value = -1;
    }

    return error_value;
}

