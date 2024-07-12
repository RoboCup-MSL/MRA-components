// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "FalconsMotionPlanning.hpp"

using namespace MRA;

// custom includes, if any
#include "geometry.hpp"


using namespace MRA::FalconsMotionPlanning;
void checkParams(ParamsType const &params);
int handleAction(google::protobuf::Timestamp timestamp, InputType const &input, ParamsType const &params, StateType &state, OutputType &output, LocalType &local);


int FalconsMotionPlanning::FalconsMotionPlanning::tick
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

    try
    {
        checkParams(params);
        error_value = handleAction(timestamp, input, params, state, output, local);
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

void checkParams(ParamsType const &params)
{
    float tolerance_xy = params.action().move().tolerance_xy();
    if (tolerance_xy <= 0.0)
    {
        throw std::runtime_error("invalid configuration parameter for move.tolerance_xy: should be larger than zero");
    }
    float tolerance_rz = params.action().move().tolerance_rz();
    if (tolerance_rz <= 0.0)
    {
        throw std::runtime_error("invalid configuration parameter for move.tolerance_rz: should be larger than zero");
    }
}

int handleAction(google::protobuf::Timestamp timestamp, InputType const &input, ParamsType const &params, StateType &state, OutputType &output, LocalType &local)
{
    int error_value = 0;
    if (input.action().has_stop())
    {
        output.set_actionresult(MRA::Datatypes::PASSED);
        output.mutable_setpoints()->mutable_move()->set_stop(true);
        output.mutable_setpoints()->mutable_bh()->set_enabled(input.action().stop().ballhandlersenabled());
    }
    else if (input.action().has_move())
    {
        output.mutable_setpoints()->mutable_bh()->set_enabled(input.action().move().ballhandlersenabled());
        // check if arrived
        MRA::Geometry::Position target_pos = input.action().move().target().position();
        MRA::Geometry::Position current_pos = input.worldstate().robot().position();
        auto delta_pos = target_pos - current_pos;
        float tolerance_xy = params.action().move().tolerance_xy();
        float tolerance_rz = params.action().move().tolerance_rz();
        bool xy_ok = ((abs(delta_pos.x) < tolerance_xy) && (abs(delta_pos.y) < tolerance_xy));
        bool rz_ok = (abs(delta_pos.rz) < tolerance_rz);
        if (xy_ok && rz_ok)
        {
            output.set_actionresult(MRA::Datatypes::PASSED);
        }
        else
        {
            *output.mutable_setpoints()->mutable_move()->mutable_target() = input.action().move().target();
            output.mutable_setpoints()->mutable_move()->set_motiontype(input.action().move().motiontype());
            output.set_actionresult(MRA::Datatypes::RUNNING);
        }
    }
    // TODO other actions
    return error_value;
}

