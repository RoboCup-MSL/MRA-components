// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "FalconsActionKeeper.hpp"

using namespace MRA;

// custom includes, if any
#include "geometry.hpp"


void checkParams(FalconsActionKeeper::ParamsType const &params)
{
    if (params.ballspeedthreshold() == 0)
    {
        throw std::runtime_error("ballspeedthreshold must not be zero");
    }
    if (params.captureradius() == 0)
    {
        throw std::runtime_error("captureradius must not be zero");
    }
    if (params.maxmovex() == 0)
    {
        throw std::runtime_error("maxmovex must not be zero");
    }
}

double clip_x(double target_x, double limit_x)
{
    if (target_x > limit_x)
    {
        return limit_x;
    }
    else if (target_x < -limit_x)
    {
        return -limit_x;
    }
    return target_x;
}

int FalconsActionKeeper::FalconsActionKeeper::tick
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

    try
    {
        double target_rz = 0.0; // FCS=0, facing forward into the field

        // Check parameters (some values must not be zero)
        checkParams(params);

        // Always return RUNNING, this action does not use PASSED or FAILED in its interaction with teamplay
        output.set_actionresult(MRA::Datatypes::ActionResult::RUNNING);

        // Homing routine
        // phase 1: from a position far away to a position in front of the goal, using homePosYoffset
        // phase 2: from general vicinity of the goal to the middle of the goalie base line
        MRA::Geometry::Position robot_position(input.worldstate().robot().position());
        MRA::Geometry::Position home_position(0, params.baseliney() + params.homeposyoffset(), 0);
        double distance_to_home = (robot_position - home_position).size();
        if (distance_to_home > params.homedistance())
        {
            output.mutable_motiontarget()->mutable_position()->set_x(home_position.x);
            output.mutable_motiontarget()->mutable_position()->set_y(home_position.y);
            output.mutable_motiontarget()->mutable_position()->set_rz(target_rz);
            // done, ignore the ball
            return error_value;
        }
        else
        {
            output.mutable_motiontarget()->mutable_position()->set_x(0);
            output.mutable_motiontarget()->mutable_position()->set_y(params.baseliney());
            output.mutable_motiontarget()->mutable_position()->set_rz(target_rz);
            // not done, consider the ball vector next
        }

        // Check if there is a ball, if not, do nothing (target is the center of the goal)
        if (!input.worldstate().has_ball())
        {
            return error_value;
        }

        // There is a ball, let's move along with it, clipping to keeper range
        double ball_x = input.worldstate().ball().position().x();
        output.mutable_motiontarget()->mutable_position()->set_x(clip_x(ball_x, params.maxmovex()));

        // TODO: port more of existing motionplanning code

        // Check if keeper needs to respond to a ball
        // * using ball speed
        // * using ball direction

        // Check if the ball speed is above the threshold
        MRA::Geometry::Velocity ball_velocity = input.worldstate().ball().velocity();
        double ball_speed = ball_velocity.size();
        bool ballmovingfastenough = ball_speed > params.ballspeedthreshold();
        diagnostics.set_ballmovingfastenough(ballmovingfastenough);
        if (!ballmovingfastenough)
        {
            return error_value;
        }

        // Check if the ball is moving towards the robot
        MRA::Geometry::Velocity ball_velocity_rcs = ball_velocity;
        ball_velocity_rcs.transformFcsToRcs(robot_position);
        bool ballmovingtowardsrobot = ball_velocity_rcs.y < 0;
        diagnostics.set_ballmovingtowardsrobot(ballmovingtowardsrobot);
        if (!ballmovingtowardsrobot)
        {
            return error_value;
        }

        // Calculate interception point
        // Use keeper horizontal line, do not rotate towards ball (as action catch does)
        MRA::Geometry::Point goalline_point_1(-params.maxmovex(), params.baseliney());
        MRA::Geometry::Point goalline_point_2(params.maxmovex(), params.baseliney());
        MRA::Geometry::Position ball_position(input.worldstate().ball().position());
        MRA::Geometry::Point ball_point(input.worldstate().ball().position().x(), input.worldstate().ball().position().y());
        MRA::Geometry::Point ball_point_inf = ball_point + ball_velocity * 1000;
        MRA::Geometry::Point intersect_point;
        int intersect_result = intersect(goalline_point_1, goalline_point_2, ball_point, ball_point_inf, false, &intersect_point);
        if (intersect_result != 1)
        {
            // No intersection found, ball out of range
            return error_value;
        }

        // Set the calculated interception point as the motion target, facing the ball
        MRA::Geometry::Position robot_target_position(intersect_point.x, intersect_point.y);
        robot_target_position.rz = target_rz;
        output.mutable_motiontarget()->mutable_position()->set_x(clip_x(robot_target_position.x, params.maxmovex()));
        output.mutable_motiontarget()->mutable_position()->set_y(robot_target_position.y);
        output.mutable_motiontarget()->mutable_position()->set_rz(robot_target_position.rz);
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
