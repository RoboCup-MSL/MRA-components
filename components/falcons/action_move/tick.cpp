// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "FalconsActionMove.hpp"

using namespace MRA;

// custom includes, if any
#include "geometry.hpp"

void checkParams(FalconsActionMove::ParamsType const &params);


int FalconsActionMove::FalconsActionMove::tick
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

    try
    {
        // user implementation goes here

        // check params
        checkParams(params);

        // copy ballHandlers setpoint
        output.set_ballhandlersenabled(input.ballhandlersenabled());

        // if with ball, then set motiontype to 1
        if (input.worldstate().robot().hasball())
        {
            output.set_motiontype(1);
        }

        // check if arrived
        MRA::Geometry::Position target_pos = input.motiontarget().position();
        MRA::Geometry::Position current_pos = input.worldstate().robot().position();
        auto delta_pos = target_pos - current_pos;
        float tolerance_xy = params.tolerances().xy();
        float tolerance_rz = params.tolerances().rz();
        bool xy_ok = ((abs(delta_pos.x) < tolerance_xy) && (abs(delta_pos.y) < tolerance_xy));
        bool rz_ok = (abs(delta_pos.rz) < tolerance_rz);
        if (xy_ok && rz_ok)
        {
            output.set_actionresult(MRA::Datatypes::PASSED);
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
            output.set_actionresult(MRA::Datatypes::RUNNING);
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

void checkParams(FalconsActionMove::ParamsType const &params)
{
    float tolerance_xy = params.tolerances().xy();
    if (tolerance_xy <= 0.0)
    {
        throw std::runtime_error("invalid configuration parameter for move.tolerances.xy: should be larger than zero");
    }
    float tolerance_rz = params.tolerances().rz();
    if (tolerance_rz <= 0.0)
    {
        throw std::runtime_error("invalid configuration parameter for move.tolerances.rz: should be larger than zero");
    }
}