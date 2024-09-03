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


using namespace MRA::FalconsActionPlanning;
void checkParams(ParamsType const &params);
int dispatchAction(google::protobuf::Timestamp timestamp, InputType const &input, ParamsType const &params, StateType &state, OutputType &output, LocalType &local);


int FalconsActionPlanning::FalconsActionPlanning::tick
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

    output.Clear();
    local.Clear();

    // user implementation goes here

    try
    {
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

int dispatchAction(google::protobuf::Timestamp timestamp, InputType const &input, ParamsType const &params, StateType &state, OutputType &output, LocalType &local)
{
    // TODO: memorize previousActionType, if different, wipe state
    int error_value = 0;
    MRA::Datatypes::ActionType currentActionType = input.action().type();
    state.mutable_action()->set_type(currentActionType);
    local.mutable_action()->set_type(currentActionType);
    if (currentActionType == MRA::Datatypes::ACTION_INVALID)
    {
        return 1;
    }
    else if (currentActionType == MRA::Datatypes::ACTION_MOVE)
    {
        error_value = handleAction<MRA::FalconsActionMove::FalconsActionMove>(
            timestamp, input, params, state, output, local, outputToSetpointsActionMove, "move"
        );
    }
    else if (currentActionType == MRA::Datatypes::ACTION_STOP)
    {
        error_value = handleAction<MRA::FalconsActionStop::FalconsActionStop>(
            timestamp, input, params, state, output, local, outputToSetpointsActionStop, "stop"
        );
    }
    else if (currentActionType == MRA::Datatypes::ACTION_PASS)
    {
        error_value = handleAction<MRA::FalconsActionAimedKick::FalconsActionAimedKick>(
            timestamp, input, params, state, output, local, outputToSetpointsActionPass, "pass"
        );
    }
    else if (currentActionType == MRA::Datatypes::ACTION_SHOOT)
    {
        error_value = handleAction<MRA::FalconsActionAimedKick::FalconsActionAimedKick>(
            timestamp, input, params, state, output, local, outputToSetpointsActionShoot, "shoot"
        );
    }
    else if (currentActionType == MRA::Datatypes::ACTION_GETBALL)
    {
        error_value = handleAction<MRA::FalconsGetball::FalconsGetball>(
            timestamp, input, params, state, output, local, outputToSetpointsActionGetball, "getball"
        );
    }
    else if (currentActionType == MRA::Datatypes::ACTION_PARK)
    {
        error_value = handleAction<MRA::FalconsActionPark::FalconsActionPark>(
            timestamp, input, params, state, output, local, outputToSetpointsActionPark, "park"
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

