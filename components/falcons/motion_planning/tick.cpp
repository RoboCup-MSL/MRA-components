// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "FalconsMotionPlanning.hpp"

using namespace MRA;

// custom includes, if any
#include "geometry.hpp"
#include "FalconsGetball.hpp"


using namespace MRA::FalconsMotionPlanning;
void checkParams(ParamsType const &params);
int dispatchAction(google::protobuf::Timestamp timestamp, InputType const &input, ParamsType const &params, StateType &state, OutputType &output, LocalType &local);


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
        error_value = dispatchAction(timestamp, input, params, state, output, local);
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
    // TODO: remove this function when creating FalconsActionMove, param checks shall be done in each nested action component
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

void outputToSetpointsGetball(MRA::FalconsGetball::OutputType const &actionOutput, Setpoints *setpoints)
{
    *setpoints->mutable_move()->mutable_target() = actionOutput.target();
}

template <typename SubcomponentType, typename OutputFunc>
int handleAction(
    const google::protobuf::Timestamp &timestamp,
    const InputType &input,
    const ParamsType &params,
    StateType &state,
    OutputType &output,
    LocalType &local,
    OutputFunc outputFunc
)
{
    // call component
    typename SubcomponentType::InputType subcomponent_input = input.action().getball(); // TODO make dynamic
    subcomponent_input.mutable_worldstate()->CopyFrom(input.worldstate());
    typename SubcomponentType::ParamsType subcomponent_params = params.action().getball(); // TODO make dynamic
    typename SubcomponentType::StateType subcomponent_state = state.action().getball();  // TODO make dynamic
    typename SubcomponentType::OutputType subcomponent_output;
    typename SubcomponentType::LocalType subcomponent_local;

    int error_value = SubcomponentType().tick(
        timestamp,
        subcomponent_input,
        subcomponent_params,
        subcomponent_state,
        subcomponent_output,
        subcomponent_local
    );

    if (error_value == 0)
    {
        // general action data handling
        output.set_actionresult(subcomponent_output.actionresult());
        state.mutable_action()->mutable_getball()->MergeFrom(subcomponent_state); // TODO make dynamic
        local.mutable_action()->mutable_getball()->MergeFrom(subcomponent_local); // TODO make dynamic

        // specific output mapping
        outputFunc(subcomponent_output, output.mutable_setpoints());
    }

    return error_value;
}

int dispatchAction(google::protobuf::Timestamp timestamp, InputType const &input, ParamsType const &params, StateType &state, OutputType &output, LocalType &local)
{
    int error_value = 0;
    if (input.action().type() == MRA::Datatypes::ACTION_INVALID)
    {
        return 1;
    }
    if (input.action().type() == MRA::Datatypes::ACTION_STOP)
    {
        output.set_actionresult(MRA::Datatypes::PASSED);
        output.mutable_setpoints()->mutable_move()->set_stop(true);
        output.mutable_setpoints()->mutable_bh()->set_enabled(input.action().stop().ballhandlersenabled());
    }
    else if (input.action().type() == MRA::Datatypes::ACTION_MOVE)
    {
        // TODO: refactor, move to component FalconsActionMove, also the param checks
        auto action_params = params.action().move();
        output.mutable_setpoints()->mutable_bh()->set_enabled(input.action().move().ballhandlersenabled());
        // check if arrived
        MRA::Geometry::Position target_pos = input.action().move().target().position();
        MRA::Geometry::Position current_pos = input.worldstate().robot().position();
        auto delta_pos = target_pos - current_pos;
        float tolerance_xy = action_params.tolerance_xy();
        float tolerance_rz = action_params.tolerance_rz();
        bool xy_ok = ((abs(delta_pos.x) < tolerance_xy) && (abs(delta_pos.y) < tolerance_xy));
        bool rz_ok = (abs(delta_pos.rz) < tolerance_rz);
        if (xy_ok && rz_ok)
        {
            output.set_actionresult(MRA::Datatypes::PASSED);
            if (action_params.active_stop())
            {
                output.mutable_setpoints()->mutable_move()->set_stop(true);
            }
        }
        else
        {
            *output.mutable_setpoints()->mutable_move()->mutable_target() = input.action().move().target();
            output.mutable_setpoints()->mutable_move()->set_motiontype(input.action().move().motiontype());
            output.set_actionresult(MRA::Datatypes::RUNNING);
        }
    }
    else if (input.action().type() == MRA::Datatypes::ACTION_GETBALL)
    {
        error_value = handleAction<MRA::FalconsGetball::FalconsGetball>(
            timestamp, input, params, state, output, local, outputToSetpointsGetball
        );
    }
    // TODO other actions
    // HACK: prevent INVALID causing crashes on client side -- TODO remove
    if (output.actionresult() == MRA::Datatypes::INVALID)
    {
        output.set_actionresult(MRA::Datatypes::FAILED);
    }
    return error_value;
}

