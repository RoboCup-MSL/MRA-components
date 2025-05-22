// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "FalconsActionFetchBall.hpp"

// dependent libraries:
#include "geometry.hpp"

using namespace MRA;

// custom includes, if any
#include <cmath>


bool checkParams(FalconsActionFetchBall::ParamsType const &params, std::string &failureReason);

int FalconsActionFetchBall::FalconsActionFetchBall::tick
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
        std::string failureReason;
        if (!checkParams(params, failureReason))
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            diagnostics.set_failurereason(failureReason);
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
            diagnostics.set_failurereason("robot is inactive");
            return error_value;
        }

        // fail when there is no ball
        if (!ws.has_ball())
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            diagnostics.set_failurereason("robot lost track of the ball");
            return error_value;
        }

        // fail when teammember has the ball
        for (auto const &teammember: ws.teammates())
        {
            if (teammember.hasball())
            {
                output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
                diagnostics.set_failurereason("teammate got the ball");
                return error_value;
            }
        }

        // fail if ball is too far away
        Geometry::Position bpos = Geometry::Position(input.worldstate().ball().position()) - Geometry::Position(input.worldstate().robot().position());
        double action_radius = params.actionradius();
        if (input.radius() > 0.0)
        {
            action_radius = input.radius();
        }
        if (bpos.size() > action_radius)
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            diagnostics.set_failurereason("ball too far away");
            return error_value;
        }

        // calculate the target position to drive to
        output.set_actionresult(MRA::Datatypes::ActionResult::RUNNING);
        MRA::Geometry::Position ball_position(ws.ball().position());
        MRA::Geometry::Velocity ball_velocity(ws.ball().velocity());
        MRA::Geometry::Position robot_position(ws.robot().position());

        // if speed is low enough, then just drive on top of the ball
        // otherwise: try to catch up, by making use of ball velocity vector
        float ball_speed = ball_velocity.size();
        float factor = params.ballspeedscaling() * (ball_speed >= params.ballspeedthreshold());

        // set target, robot facing angle towards ball
        // (by letting target ball "face away from" robot)
        MRA::Geometry::Position target = ball_position + ball_velocity * factor;
        target.faceAwayFrom(robot_position);

        // get shortest angle between current robot angle and target angle.
        auto angle_to_rotate = MRA::Geometry::min_angle(robot_position.rz, target.rz);
        if (fabs(angle_to_rotate) > MRA::Geometry::deg_to_rad(params.rotationonlyangle())
            and (robot_position - ball_position).size() < params.rotationonlydistance()) {
            // only rotate to target position, x and y position stay the same
            // write output
            output.mutable_motiontarget()->mutable_position()->set_x(robot_position.x);
            output.mutable_motiontarget()->mutable_position()->set_y(robot_position.y);
            output.mutable_motiontarget()->mutable_position()->set_rz(target.rz);
        }
        else {
            // translate and rotate to target position
            // write output
            output.mutable_motiontarget()->mutable_position()->set_x(target.x);
            output.mutable_motiontarget()->mutable_position()->set_y(target.y);
            output.mutable_motiontarget()->mutable_position()->set_rz(target.rz);
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

bool checkParams(FalconsActionFetchBall::ParamsType const &params, std::string &failureReason)
{
    // nothing to check for this action
    return true;
}

