// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "FalconsActionShield.hpp"

using namespace MRA;

// custom includes, if any
#include "geometry.hpp"

// forward declarations
bool checkParams(FalconsActionShield::ParamsType const &params, std::string &failureReason);


int FalconsActionShield::FalconsActionShield::tick
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

        // fail when robot has no ball possession
        if (!input.worldstate().robot().hasball())
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            diagnostics.set_failurereason("robot lost possession of the ball");
            return 0;
        }

        // fail when there is no obstacle closeby
        int num_obstacles = input.worldstate().obstacles_size();
        if (num_obstacles == 0)
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            diagnostics.set_failurereason("no obstacles detected anywhere");
            return 0;
        }
        MRA::Geometry::Position robot_position = input.worldstate().robot().position();
        MRA::Geometry::Position closest_obstacle = input.worldstate().obstacles(0).position();
        double closest_distance = robot_position.distanceTo(closest_obstacle);
        for (int i = 1; i < num_obstacles; i++)
        {
            MRA::Geometry::Position obstacle = input.worldstate().obstacles(i).position();
            double distance_this_obstacle = robot_position.distanceTo(obstacle);
            if (distance_this_obstacle < closest_distance)
            {
                closest_distance = distance_this_obstacle;
                closest_obstacle = obstacle;
            }
        }
        if (closest_distance > params.obstacledistance())
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            diagnostics.set_failurereason("no obstacles detected closeby");
            return 0;
        }

        // this action had no PASSED condition, only FAILED or RUNNING

        // all checks ok, calculate output pose
        MRA::Geometry::Position robot_target_position = robot_position;
        robot_target_position.faceAwayFrom(closest_obstacle);
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

bool checkParams(FalconsActionShield::ParamsType const &params, std::string &failureReason)
{
    if (params.obstacledistance() == 0)
    {
        failureReason = "invalid configuration parameter obstacledistance: should be larger than zero";
        return false;
    }
    return true;
}