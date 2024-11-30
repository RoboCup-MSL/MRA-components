// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "FalconsActionCatchBall.hpp"

using namespace MRA;

// custom includes, if any
#include "geometry.hpp"


bool checkParams(FalconsActionCatchBall::ParamsType const &params, std::string &verdict);
int calc_intercept_strafe(FalconsActionCatchBall::InputType const &input, FalconsActionCatchBall::ParamsType const &params, FalconsActionCatchBall::OutputType &output, FalconsActionCatchBall::DiagnosticsType &diagnostics);

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

    try
    {
        // initialize output and diagnostics
        auto const ws = input.worldstate();
        output.Clear();
        diagnostics.Clear();

        // check params
        std::string verdict;
        if (!checkParams(params, verdict))
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            diagnostics.set_verdict(verdict);
            return 0;
        }

        // always enable ballhandlers
        output.set_bhenabled(true);

        // action is successful when robot has the ball
        if (ws.robot().hasball())
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::PASSED);
            return error_value;
        }

        // fail when robot is inactive
        if (!ws.robot().active())
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            diagnostics.set_verdict("robot is inactive");
            return error_value;
        }

        // fail when there is no ball
        if (!ws.has_ball())
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            diagnostics.set_verdict("robot lost track of the ball");
            return error_value;
        }

        // fail when teammember has the ball
        for (auto const &teammember: ws.teammates())
        {
            if (teammember.hasball())
            {
                output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
                diagnostics.set_verdict("teammate got the ball");
                return error_value;
            }
        }

        // check if the ball speed is above the threshold
        MRA::Geometry::Velocity ball_velocity = input.worldstate().ball().velocity();
        double ball_speed = ball_velocity.size();
        bool ballmovingfastenough = ball_speed > params.ballspeedthreshold();
        diagnostics.set_ballmovingfastenough(ballmovingfastenough);
        if (!ballmovingfastenough)
        {
            // ball may not yet be moving, use state
            if (state.ballwasmovingfastenough())
            {
                output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
                diagnostics.set_verdict("ball not moving fast enough anymore");
            }
            else
            {
                output.set_actionresult(MRA::Datatypes::ActionResult::RUNNING);
            }
            return error_value;
        }
        state.set_ballwasmovingfastenough(true);

        // check if the ball is moving towards the robot
        MRA::Geometry::Position robot_position(input.worldstate().robot().position());
        MRA::Geometry::Velocity ball_velocity_rcs = ball_velocity;
        ball_velocity_rcs.transformFcsToRcs(robot_position);
        bool ballmovingtowardsrobot = ball_velocity_rcs.y < 0;
        diagnostics.set_ballmovingtowardsrobot(ballmovingtowardsrobot);
        if (!ballmovingtowardsrobot)
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            diagnostics.set_verdict("ball not moving towards robot");
            return error_value;
        }

        // proactive mode or traditional "strafe"?
        if (params.proactive())
        {
            //return calc_intercept_proactive(input, params, output, diagnostics);
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            diagnostics.set_verdict("proactive intercept not yet implemented");
        }
        else
        {
            return calc_intercept_strafe(input, params, output, diagnostics);
        }

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

bool checkParams(FalconsActionCatchBall::ParamsType const &params, std::string &verdict)
{
    if (params.ballspeedthreshold() == 0)
    {
        verdict = "invalid configuration parameter ballspeedthreshold: should be larger than zero";
        return false;
    }
    if (params.captureradius() == 0)
    {
        verdict = "invalid configuration parameter captureradius: should be larger than zero";
        return false;
    }
    return true;
}

int calc_intercept_strafe(
    FalconsActionCatchBall::InputType  const &input,
    FalconsActionCatchBall::ParamsType const &params,
    FalconsActionCatchBall::OutputType       &output,
    FalconsActionCatchBall::DiagnosticsType  &diagnostics
)
{
    // calculate interception point
    MRA::Geometry::Position robot_position(input.worldstate().robot().position());
    MRA::Geometry::Position robot_position_side = robot_position;
    robot_position_side.addRcsToFcs(MRA::Geometry::Position(1, 0));
    MRA::Geometry::Point robot_point = robot_position;
    MRA::Geometry::Point robot_point_side = robot_position_side;
    MRA::Geometry::Velocity ball_velocity = input.worldstate().ball().velocity();
    double ball_speed = ball_velocity.size();
    MRA::Geometry::Position ball_position(input.worldstate().ball().position());
    MRA::Geometry::Point ball_point(input.worldstate().ball().position().x(), input.worldstate().ball().position().y());
    MRA::Geometry::Point ball_point_next = ball_point + ball_velocity * 0.1;
    MRA::Geometry::Point intersect_point;
    int intersect_result = intersect(robot_point, robot_point_side, ball_point, ball_point_next, true, &intersect_point);
    if (intersect_result != 1)
    {
        // this should not happen, all previous checks should have guaranteed a valid intersection
        throw std::runtime_error("intersect failed with return code " + std::to_string(intersect_result));
    }

    // determine action radius
    double action_radius = params.captureradius();
    if (input.radius() > 0.0)
    {
        action_radius = input.radius();
    }

    // check if the ball is going to be within capture range
    double distance_ball_to_intersect = (intersect_point - ball_point).size();
    double distance_robot_to_intersect = (intersect_point - robot_point).size();
    //MRA_LOG_DEBUG("distance_ball_to_intersect=(%6.2f)", distance_ball_to_intersect);
    //MRA_LOG_DEBUG("distance_robot_to_intersect=(%6.2f)", distance_robot_to_intersect);
    bool ballmovingwithincapturerange = distance_robot_to_intersect < action_radius;
    diagnostics.set_ballmovingwithincapturerange(ballmovingwithincapturerange);
    if (!ballmovingwithincapturerange)
    {
        output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
        diagnostics.set_verdict("ball trajectory too far away");
        return 0;
    }

    // calculate the time to intercept based on ball speed and robot speed
    double time_to_catch = distance_ball_to_intersect / ball_speed;
    diagnostics.set_timetocatch(time_to_catch);

    // set the calculated interception point as the motion target, facing the ball
    MRA::Geometry::Position robot_target_position(intersect_point.x, intersect_point.y);
    robot_target_position.faceTowards(ball_position);
    output.mutable_motiontarget()->mutable_position()->set_x(robot_target_position.x);
    output.mutable_motiontarget()->mutable_position()->set_y(robot_target_position.y);
    output.mutable_motiontarget()->mutable_position()->set_rz(robot_target_position.rz);
    output.set_actionresult(MRA::Datatypes::ActionResult::RUNNING);
    return 0;
}