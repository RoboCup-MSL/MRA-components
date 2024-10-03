// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "FalconsActionCatchBall.hpp"

using namespace MRA;

// custom includes, if any
#include "geometry.hpp"


int FalconsActionCatchBall::FalconsActionCatchBall::tick
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
        // Always enable ballhandlers
        output.set_bhenabled(true);

        // Check if the ball speed is above the threshold
        MRA::Geometry::Velocity ball_velocity = input.worldstate().ball().velocity();
        float ball_speed = ball_velocity.size();
        bool ballmovingfastenough = ball_speed > params.ballspeedthreshold();
        diagnostics.set_ballmovingfastenough(ballmovingfastenough);
        if (!ballmovingfastenough)
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            return error_value;
        }

        // Check if the ball is moving towards the robot
        bool ballmovingtowardsrobot = input.worldstate().ball().velocity().x() < 0;
        diagnostics.set_ballmovingtowardsrobot(ballmovingtowardsrobot);
        if (!ballmovingtowardsrobot)
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            return error_value;
        }

        // Calculate interception point
        MRA::Geometry::Position robot_position(input.worldstate().robot().position());
        MRA::Geometry::Position robot_position_side = robot_position;
        robot_position_side.addRcsToFcs(MRA::Geometry::Position(1, 0));
        MRA::Geometry::Point robot_point = robot_position;
        MRA::Geometry::Point robot_point_side = robot_position_side;
        MRA::Geometry::Position ball_position(input.worldstate().ball().position());
        MRA::Geometry::Point ball_point(input.worldstate().ball().position().x(), input.worldstate().ball().position().y());
        MRA::Geometry::Point ball_point_next = ball_point + ball_velocity * 0.1;
        MRA::Geometry::Point intersect_point;
        int intersect_result = intersect(robot_point, robot_point_side, ball_point, ball_point_next, true, &intersect_point);
        //MRA_LOG_DEBUG("robot_point=(%6.2f, %6.2f)", robot_point.x, robot_point.y);
        //MRA_LOG_DEBUG("robot_point_side=(%6.2f, %6.2f)", robot_point_side.x, robot_point_side.y);
        //MRA_LOG_DEBUG("ball_point=(%6.2f, %6.2f)", ball_point.x, ball_point.y);
        //MRA_LOG_DEBUG("ball_point_next=(%6.2f, %6.2f)", ball_point_next.x, ball_point_next.y);
        //MRA_LOG_DEBUG("intersect_result=%d", intersect_result);
        //MRA_LOG_DEBUG("intersect_point=(%6.2f, %6.2f)", intersect_point.x, intersect_point.y);
        if (intersect_result != 1)
        {
            // this should not happen, all previous checks should have guaranteed a valid intersection
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            throw std::runtime_error("intersect failed with return code " + std::to_string(intersect_result));
        }

        // Check if the ball is within capture range
        float distance_ball_to_intersect = (intersect_point - ball_point).size();
        float distance_robot_to_intersect = (intersect_point - robot_point).size();
        //MRA_LOG_DEBUG("distance_ball_to_intersect=(%6.2f)", distance_ball_to_intersect);
        //MRA_LOG_DEBUG("distance_robot_to_intersect=(%6.2f)", distance_robot_to_intersect);
        bool ballmovingwithincapturerange = distance_robot_to_intersect < params.captureradius();
        diagnostics.set_ballmovingwithincapturerange(ballmovingwithincapturerange);
        if (!ballmovingwithincapturerange)
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            return error_value;
        }

        // Calculate the time to intercept based on ball speed and robot speed
        float time_to_catch = distance_ball_to_intersect / ball_speed;
        diagnostics.set_timetocatch(time_to_catch);

        // Set the calculated interception point as the motion target, facing the ball
        MRA::Geometry::Position robot_target_position(intersect_point.x, intersect_point.y);
        robot_target_position.faceTowards(ball_position);
        output.mutable_motiontarget()->mutable_position()->set_x(robot_target_position.x);
        output.mutable_motiontarget()->mutable_position()->set_y(robot_target_position.y);
        output.mutable_motiontarget()->mutable_position()->set_rz(robot_target_position.rz);
        output.set_actionresult(MRA::Datatypes::ActionResult::RUNNING);
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

