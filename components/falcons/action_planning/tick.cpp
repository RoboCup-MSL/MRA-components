// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "FalconsActionPlanning.hpp"

using namespace MRA;

// custom includes, if any
#include "geometry.hpp"
#include "subcomponent_template.hpp" // internal
#include "FalconsGetball.hpp"
#include "FalconsActionStop.hpp"
#include "FalconsActionMove.hpp"
#include "FalconsActionAimedKick.hpp"
#include "FalconsActionPark.hpp"
#include "FalconsActionCatchBall.hpp"
#include "FalconsActionKeeper.hpp"


using namespace MRA::FalconsActionPlanning;
void checkParams(ParamsType const &params);
int dispatchAction(google::protobuf::Timestamp timestamp, InputType const &input, ParamsType const &params, StateType &state, OutputType &output, DiagnosticsType &diagnostics);


int FalconsActionPlanning::FalconsActionPlanning::tick
(
    google::protobuf::Timestamp timestamp,   // absolute timestamp
    InputType  const           &input,       // input data, type generated from Input.proto
    ParamsType const           &params,      // configuration parameters, type generated from Params.proto
    StateType                  &state,       // state data, type generated from State.proto
    OutputType                 &output,      // output data, type generated from Output.proto
    DiagnosticsType            &diagnostics  // diagnostics/diagnostics data, type generated from Diagnostics.proto
)
{
    int error_value = 0;
    MRA_LOG_TICK();

    output.Clear();
    diagnostics.Clear();

    // user implementation goes here

    try
    {
        error_value = dispatchAction(timestamp, input, params, state, output, diagnostics);
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

// design notes:
// * each action is implemented in a dedicated subcomponent:
//   * ACTION_STOP: FalconsActionStop
//   * ACTION_SHOOT: FalconsActionAimedKick
//   * etc
// * a template function "handleAction" is used to interface with each action subcomponent
//   * making use of specific outputToSetpoints* adapters per action
// * this makes the function "dispatchAction" short and to the point
// * parameter checking is delegated to the components

void outputToSetpointsActionStop(MRA::FalconsActionStop::OutputType const &actionOutput, Setpoints *setpoints)
{
    setpoints->mutable_move()->set_stop(actionOutput.stopmoving());
    setpoints->mutable_bh()->set_enabled(actionOutput.ballhandlersenabled());
}

void outputToSetpointsActionMove(MRA::FalconsActionMove::OutputType const &actionOutput, Setpoints *setpoints)
{
    *setpoints->mutable_move()->mutable_target() = actionOutput.motiontarget();
    setpoints->mutable_move()->set_stop(actionOutput.stop());
    setpoints->mutable_move()->set_motiontype(actionOutput.motiontype());
    setpoints->mutable_bh()->set_enabled(actionOutput.ballhandlersenabled());
}

void outputToSetpointsActionGetball(MRA::FalconsGetball::OutputType const &actionOutput, Setpoints *setpoints)
{
    if (actionOutput.has_target())
    {
        *setpoints->mutable_move()->mutable_target() = actionOutput.target();
    }
    setpoints->mutable_bh()->set_enabled(true);
}

void outputToSetpointsActionPass(MRA::FalconsActionAimedKick::OutputType const &actionOutput, Setpoints *setpoints)
{
    if (actionOutput.has_motiontarget())
    {
        *setpoints->mutable_move()->mutable_target() = actionOutput.motiontarget();
    }
    setpoints->mutable_bh()->set_enabled(actionOutput.bhenabled());
    setpoints->mutable_shoot()->set_type(MRA::FalconsActionPlanning::SHOOT_TYPE_PASS);
    setpoints->mutable_shoot()->set_phase(actionOutput.phase());
    setpoints->mutable_shoot()->set_pos_x(actionOutput.balltarget().x());
    setpoints->mutable_shoot()->set_pos_y(actionOutput.balltarget().y());
}

void outputToSetpointsActionShoot(MRA::FalconsActionAimedKick::OutputType const &actionOutput, Setpoints *setpoints)
{
    if (actionOutput.has_motiontarget())
    {
        *setpoints->mutable_move()->mutable_target() = actionOutput.motiontarget();
    }
    setpoints->mutable_bh()->set_enabled(actionOutput.bhenabled());
    if (actionOutput.dokick())
    {
        setpoints->mutable_shoot()->set_type(MRA::FalconsActionPlanning::SHOOT_TYPE_SHOOT);
        setpoints->mutable_shoot()->set_phase(MRA::FalconsActionAimedKick::SHOOT_PHASE_DISCHARGE);
    }
    else // prepare & aiming phase
    {
        setpoints->mutable_shoot()->set_type(MRA::FalconsActionPlanning::SHOOT_TYPE_SHOOT);
        setpoints->mutable_shoot()->set_phase(MRA::FalconsActionAimedKick::SHOOT_PHASE_PREPARE);
        setpoints->mutable_shoot()->set_pos_x(actionOutput.balltarget().x());
        setpoints->mutable_shoot()->set_pos_y(actionOutput.balltarget().y());
        setpoints->mutable_shoot()->set_pos_z(actionOutput.balltarget().z());
    }
}

void outputToSetpointsActionPark(MRA::FalconsActionPark::OutputType const &actionOutput, Setpoints *setpoints)
{
    *setpoints->mutable_move()->mutable_target() = actionOutput.motiontarget();
    setpoints->mutable_move()->set_stop(actionOutput.stop());
    setpoints->mutable_move()->set_motiontype(actionOutput.motiontype());
    setpoints->mutable_bh()->set_enabled(false);
}

void outputToSetpointsActionCatchBall(MRA::FalconsActionCatchBall::OutputType const &actionOutput, Setpoints *setpoints)
{
    *setpoints->mutable_move()->mutable_target() = actionOutput.motiontarget();
    setpoints->mutable_bh()->set_enabled(actionOutput.bhenabled());
}

void outputToSetpointsActionKeeper(MRA::FalconsActionKeeper::OutputType const &actionOutput, Setpoints *setpoints)
{
    if (actionOutput.has_motiontarget())
    {
        *setpoints->mutable_move()->mutable_target() = actionOutput.motiontarget();
    }
    if (actionOutput.has_framesetpoint())
    {
        setpoints->mutable_keeperframe()->set_extension(actionOutput.framesetpoint().extension());
    }
}

void replaceAll(std::string &s, const std::string &search, const std::string &replace) {
    size_t pos = 0;
    while ((pos = s.find(search, pos)) != std::string::npos) {
        s.replace(pos, search.length(), replace);
        pos += replace.length();
    }
}

void checkFlushHistory(MRA::Datatypes::ActionType currentActionType, ParamsType const &params, StateType &state)
{
    int num_ticks = state.history().samples_size();
    MRA_TRACE_FUNCTION_INPUTS(state.action().type(), currentActionType, num_ticks);
    if (state.action().type() != currentActionType)
    {
        // flush history to file, if so configured
        bool do_flush = params.history().enabled() && (num_ticks >= params.history().minimumticks());
        if (do_flush)
        {
            std::string filename = params.history().logfolder() + "/" + params.history().logfilepattern();
            // replace <action> and <actionresult>, use last sample
            auto lastSample = state.history().samples(num_ticks - 1);
            replaceAll(filename, "<action>", MRA::Datatypes::ActionType_Name(lastSample.input().action().type()));
            replaceAll(filename, "<actionresult>", MRA::Datatypes::ActionResult_Name(lastSample.output().actionresult()));
            // replace <date> and <time>
            std::time_t currentTime = std::time(nullptr);
            struct std::tm *timeinfo = std::localtime(&currentTime);
            char dateBuffer[9];
            std::strftime(dateBuffer, sizeof(dateBuffer), "%Y%m%d", timeinfo);
            char timeBuffer[11];
            std::strftime(timeBuffer, sizeof(timeBuffer), "%H%M%S", timeinfo);
            replaceAll(filename, "<date>", dateBuffer);
            replaceAll(filename, "<time>", timeBuffer);
            // replace <msec> for milliseconds
            auto now = std::chrono::system_clock::now();
            auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
            int milliseconds = (microseconds / 1000) % 1000;
            sprintf(timeBuffer, "%03d", milliseconds);
            replaceAll(filename, "<msec>", timeBuffer);
            // logging
            MRA_LOG_INFO("flushing history to file: %s", filename.c_str());

            // TODO: try to open file, make sure an error occurs if configuration is empty (either folder or filename)

            // TODO: reuse logging .bin write
        }

        // reset and re-initialize history
        state.Clear();
        state.mutable_action()->set_type(currentActionType);
        state.mutable_history()->set_type(currentActionType);
        *state.mutable_history()->mutable_params() = params;
    }
}

void updateHistory(google::protobuf::Timestamp timestamp, InputType const &input, ParamsType const &params, StateType &state, OutputType const &output, DiagnosticsType const &diagnostics)
{
    MRA_TRACE_FUNCTION();
    MRA::FalconsActionPlanning::ActionSample sample;
    *sample.mutable_timestamp() = timestamp;
    *sample.mutable_input() = input;
    *sample.mutable_output() = output;
    *sample.mutable_diagnostics() = diagnostics;
    *state.mutable_history()->mutable_samples()->Add() = sample;
}

int dispatchAction(google::protobuf::Timestamp timestamp, InputType const &input, ParamsType const &params, StateType &state, OutputType &output, DiagnosticsType &diagnostics)
{
    MRA_TRACE_FUNCTION();
    int error_value = 0;
    MRA::Datatypes::ActionType currentActionType = input.action().type();
    checkFlushHistory(currentActionType, params, state);
    diagnostics.mutable_action()->set_type(currentActionType);
    if (currentActionType == MRA::Datatypes::ACTION_INVALID)
    {
        return 1;
    }
    else if (currentActionType == MRA::Datatypes::ACTION_MOVE)
    {
        error_value = handleAction<MRA::FalconsActionMove::FalconsActionMove>(
            timestamp, input, params, state, output, diagnostics, outputToSetpointsActionMove, "move"
        );
    }
    else if (currentActionType == MRA::Datatypes::ACTION_STOP)
    {
        error_value = handleAction<MRA::FalconsActionStop::FalconsActionStop>(
            timestamp, input, params, state, output, diagnostics, outputToSetpointsActionStop, "stop"
        );
    }
    else if (currentActionType == MRA::Datatypes::ACTION_PASS)
    {
        error_value = handleAction<MRA::FalconsActionAimedKick::FalconsActionAimedKick>(
            timestamp, input, params, state, output, diagnostics, outputToSetpointsActionPass, "pass"
        );
    }
    else if (currentActionType == MRA::Datatypes::ACTION_SHOOT)
    {
        error_value = handleAction<MRA::FalconsActionAimedKick::FalconsActionAimedKick>(
            timestamp, input, params, state, output, diagnostics, outputToSetpointsActionShoot, "shoot"
        );
    }
    else if (currentActionType == MRA::Datatypes::ACTION_GETBALL)
    {
        error_value = handleAction<MRA::FalconsGetball::FalconsGetball>(
            timestamp, input, params, state, output, diagnostics, outputToSetpointsActionGetball, "getball"
        );
    }
    else if (currentActionType == MRA::Datatypes::ACTION_PARK)
    {
        error_value = handleAction<MRA::FalconsActionPark::FalconsActionPark>(
            timestamp, input, params, state, output, diagnostics, outputToSetpointsActionPark, "park"
        );
    }
    else if (currentActionType == MRA::Datatypes::ACTION_CATCHBALL)
    {
        error_value = handleAction<MRA::FalconsActionCatchBall::FalconsActionCatchBall>(
            timestamp, input, params, state, output, diagnostics, outputToSetpointsActionCatchBall, "catchball"
        );
    }
    else if (currentActionType == MRA::Datatypes::ACTION_KEEPER)
    {
        error_value = handleAction<MRA::FalconsActionKeeper::FalconsActionKeeper>(
            timestamp, input, params, state, output, diagnostics, outputToSetpointsActionKeeper, "keeper"
        );
    }
    // TODO other actions
    // HACK: prevent INVALID causing crashes on client side -- TODO remove
    if (output.actionresult() == MRA::Datatypes::INVALID)
    {
        output.set_actionresult(MRA::Datatypes::FAILED);
    }
    // update history
    updateHistory(timestamp, input, params, state, output, diagnostics);
    return error_value;
}

