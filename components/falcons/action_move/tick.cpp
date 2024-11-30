// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "FalconsActionMove.hpp"

using namespace MRA;

// custom includes, if any
#include "geometry.hpp"

bool checkParams(FalconsActionMove::ParamsType const &params, std::string &verdict);


int FalconsActionMove::FalconsActionMove::tick
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
        // user implementation goes here

        // check params
        std::string verdict;
        if (!checkParams(params, verdict))
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
            diagnostics.set_verdict(verdict);
            return 0;
        }

        // copy ballHandlers setpoint
        output.set_ballhandlersenabled(input.ballhandlersenabled());

        // if with ball, then set motiontype to 1
        // TODO: call ACTION_DRIBBLE
        output.set_motiontype(input.motiontype());
        if (input.worldstate().robot().hasball())
        {
            output.set_motiontype(1);
        }

        // check if arrived
        MRA::Geometry::Position target_pos = input.motiontarget().position();
        MRA::Geometry::Position current_pos = input.worldstate().robot().position();
        auto delta_pos = target_pos - current_pos;
        double tolerance_xy = params.tolerances().xy();
        double tolerance_rz = params.tolerances().rz();
        bool xy_ok = ((abs(delta_pos.x) < tolerance_xy) && (abs(delta_pos.y) < tolerance_xy));
        bool rz_ok = (abs(delta_pos.rz) < tolerance_rz);
        if (xy_ok && rz_ok)
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::PASSED);
            if (params.activestop())
            {
                output.set_stop(true);
            }
        }
        else
        {
            // just copy target setpoint, for further processing down the line by pathPlanning and subsequently velocityControl
            // architecture idea: should pathPlanning responsibility be moved here? (to apply obstacle avoidance and motion smoothening)
            *output.mutable_motiontarget() = input.motiontarget();
            output.set_actionresult(MRA::Datatypes::ActionResult::RUNNING);
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

bool checkParams(FalconsActionMove::ParamsType const &params, std::string &verdict)
{
    double tolerance_xy = params.tolerances().xy();
    if (tolerance_xy <= 0.0)
    {
        verdict = "invalid configuration parameter for move.tolerances.xy: should be larger than zero";
        return false;
    }
    double tolerance_rz = params.tolerances().rz();
    if (tolerance_rz <= 0.0)
    {
        verdict = "invalid configuration parameter for move.tolerances.rz: should be larger than zero";
        return false;
    }
    return true;
}
