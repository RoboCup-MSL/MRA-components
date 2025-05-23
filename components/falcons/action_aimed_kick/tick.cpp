// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "FalconsActionAimedKick.hpp"

using namespace MRA;

// custom includes, if any
#include "geometry.hpp"
#include <google/protobuf/util/time_util.h>


class ActionAimedKick
{
public:
    ActionAimedKick(google::protobuf::Timestamp const &timestamp, FalconsActionAimedKick::InputType const &input, FalconsActionAimedKick::ParamsType const &params, FalconsActionAimedKick::StateType &state, FalconsActionAimedKick::OutputType &output, FalconsActionAimedKick::DiagnosticsType &diagnostics);
    int run();

private:
    google::protobuf::Timestamp const &_timestamp;
    FalconsActionAimedKick::InputType const &_input;
    FalconsActionAimedKick::ParamsType const &_params;
    FalconsActionAimedKick::StateType &_state;
    FalconsActionAimedKick::OutputType &_output;
    FalconsActionAimedKick::DiagnosticsType &_diagnostics;
    MRA::Geometry::Position _robotPos;
    MRA::Geometry::Position _ballCurrentPos;
    MRA::Geometry::Position _ballTargetPos;
    MRA::Geometry::Position _deltaBallTargetToRobot;
    MRA::Geometry::Position _deltaBallTargetToCurrentBall;
    MRA::Geometry::Position _deltaCurrentBallToRobot;
    float _remainingRotationAngle;
    void precalculate();
    void phasePrepare();
    void phaseDischarge();
    float calculateAimError();
    void phaseCooldown();
};

int FalconsActionAimedKick::FalconsActionAimedKick::tick
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
    // user implementation goes here
    auto a = ActionAimedKick(timestamp, input, params, state, output, diagnostics);
    error_value = a.run();
    return error_value;
}

ActionAimedKick::ActionAimedKick(google::protobuf::Timestamp const &timestamp, FalconsActionAimedKick::InputType const &input, FalconsActionAimedKick::ParamsType const &params, FalconsActionAimedKick::StateType &state, FalconsActionAimedKick::OutputType &output, FalconsActionAimedKick::DiagnosticsType &diagnostics)
:
    _timestamp(timestamp),
    _input(input),
    _params(params),
    _state(state),
    _output(output),
    _diagnostics(diagnostics)
{}

int ActionAimedKick::run()
{
    MRA_TRACE_FUNCTION();
    // fail when there is no ball
    if (!_input.worldstate().has_ball())
    {
        _output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
        _output.set_phase(MRA::FalconsActionAimedKick::SHOOT_PHASE_INVALID);
        _diagnostics.set_verdict("robot lost track of the ball");
        return 0;
    }
    // calculate angles and such
    precalculate();
    // set ballhandlers, always
    _output.set_bhenabled(true);
    // TODO: disable just before shot, for improved accuracy/power?
    // small state machine to go through the phases
    if (_state.phase() == FalconsActionAimedKick::SHOOT_PHASE_INVALID)
    {
        // first tick: advance to PREPARE
        _state.set_phase(FalconsActionAimedKick::SHOOT_PHASE_PREPARE);
    }
    if (_state.phase() == FalconsActionAimedKick::SHOOT_PHASE_PREPARE)
    {
        phasePrepare();
    }
    else if (_state.phase() == FalconsActionAimedKick::SHOOT_PHASE_DISCHARGE)
    {
        phaseDischarge();
    }
    else if (_state.phase() == FalconsActionAimedKick::SHOOT_PHASE_COOLDOWN)
    {
        phaseCooldown();
    }
    // store phase also in output
    _output.set_phase(_state.phase());
    return 0;
}

void ActionAimedKick::precalculate()
{
    MRA_TRACE_FUNCTION();
    _robotPos = _input.worldstate().robot().position();
    _ballTargetPos = _input.target().position();
    _ballCurrentPos = _input.worldstate().ball().position();
    _ballTargetPos.faceAwayFrom(_robotPos);
    _deltaBallTargetToRobot = _ballTargetPos - _robotPos;
    _deltaBallTargetToCurrentBall = _ballTargetPos - _ballCurrentPos;
    _deltaCurrentBallToRobot = _ballCurrentPos - _robotPos;
    _remainingRotationAngle = _deltaBallTargetToRobot.rz;
    // TODO: intentional overshoot?
    MRA_TRACE_FUNCTION_OUTPUTS(_remainingRotationAngle);
}

void ActionAimedKick::phasePrepare()
{
    MRA_TRACE_FUNCTION();
    if (!_input.worldstate().robot().hasball())
    {
        // TODO: robustness: use state - it can happen that the robot kicked the ball away, but it takes a tick or more for the ball to actually leave?
        // in that case, functionally the shot was a success, so we should not produce FAILED
        _output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
        _diagnostics.set_verdict("robot lost possession of the ball");
        return;
    }
    // check if phase ends
    if (abs(_remainingRotationAngle) > _params.angleaccuracythreshold())
    {
        // prepare, aim, running
        *_output.mutable_motiontarget()->mutable_position() = _input.worldstate().robot().position();
        _output.mutable_motiontarget()->mutable_position()->set_rz(_ballTargetPos.rz);
        *_output.mutable_balltarget() = _input.target().position();
        _output.set_actionresult(MRA::Datatypes::ActionResult::RUNNING);
        _diagnostics.set_remainingrotationangle(_remainingRotationAngle);
    }
    else
    {
        // shoot, advance to next phase
        // TODO: calculate timeToKick, tune it?
        _output.set_dokick(true);
        *_state.mutable_dischargetimestamp() = _timestamp;
        _output.set_actionresult(MRA::Datatypes::ActionResult::RUNNING);
        _state.set_phase(MRA::FalconsActionAimedKick::SHOOT_PHASE_DISCHARGE);
    }
}

void ActionAimedKick::phaseDischarge()
{
    MRA_TRACE_FUNCTION();
    // not much to do except advancing to next phase
    _output.set_actionresult(MRA::Datatypes::ActionResult::RUNNING);
    _state.set_phase(MRA::FalconsActionAimedKick::SHOOT_PHASE_COOLDOWN);
}

float ActionAimedKick::calculateAimError()
{
    MRA_TRACE_FUNCTION();
    // we can assume ballTargetDistance < _params.balltargetproximity()
    float angleAtTimeOfShooting = _ballTargetPos.rz; // FCS angle because of using MRA::Geometry::Position
    float evaluationAngle = MRA::Geometry::calc_facing_angle_fcs(_robotPos.x, _robotPos.y, _ballCurrentPos.x, _ballCurrentPos.y);
    float result = MRA::Geometry::wrap_pi(evaluationAngle - angleAtTimeOfShooting);
    MRA_TRACE_FUNCTION_OUTPUTS(evaluationAngle, result);
    return result;
}

void ActionAimedKick::phaseCooldown()
{
    MRA_TRACE_FUNCTION();
    // it typically takes a few ticks for the ballhandlers to signal the ball has left
    // use a little timeout to make sure this action is not "stuck" for too long
    auto elapsedDuration = _timestamp - _state.dischargetimestamp();
    float elapsedSeconds = 1e-9 * google::protobuf::util::TimeUtil::DurationToNanoseconds(elapsedDuration);
    // float ballTargetDistance = _deltaBallTargetToCurrentBall.size();
    _output.set_actionresult(MRA::Datatypes::ActionResult::RUNNING);
    if (elapsedSeconds > _params.maxcooldownduration())
    {
        _output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
    }
    else if (!_input.worldstate().robot().hasball())
    {
        // determine success
        // if ball ends up close to target, then PASSED
        float ballTargetDistance = _deltaBallTargetToCurrentBall.size();
        if (ballTargetDistance < _params.balltargetproximity())
        {
            _diagnostics.set_aimerror(calculateAimError());
            _output.set_actionresult(MRA::Datatypes::ActionResult::PASSED);
        }
    }
    MRA_TRACE_FUNCTION_OUTPUTS(elapsedSeconds, ballTargetDistance);
}
