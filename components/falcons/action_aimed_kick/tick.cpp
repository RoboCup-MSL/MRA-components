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
    void phaseAimCoarse();
    void phaseAimFine();
    void phaseDischarge();
    float calculateAimError();
    void phaseCooldown();
    bool checkParams(FalconsActionAimedKick::ParamsType const &params, std::string &failureReason);
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
    // check params
    std::string failureReason;
    if (!checkParams(_params, failureReason))
    {
        _output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
        _diagnostics.set_failurereason(failureReason);
        return 0;
    }
    // fail when there is no ball
    if (!_input.worldstate().has_ball())
    {
        _output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
        _output.set_phase(MRA::FalconsActionAimedKick::SHOOT_PHASE_INVALID);
        _diagnostics.set_failurereason("robot lost track of the ball");
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
        _state.set_phase(FalconsActionAimedKick::SHOOT_PHASE_AIM_COARSE);
    }
    if (_state.phase() == FalconsActionAimedKick::SHOOT_PHASE_AIM_COARSE)
    {
        phaseAimCoarse();
    }
    // allow to fall through coarse phase
    if (_state.phase() == FalconsActionAimedKick::SHOOT_PHASE_AIM_FINE)
    {
        phaseAimFine();
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

void ActionAimedKick::phaseAimCoarse()
{
    MRA_TRACE_FUNCTION();
    double angleAccuracyThreshold = _params.timeout().angleaccuracythreshold();
    if (!_input.worldstate().robot().hasball())
    {
        // in that case, functionally the shot was a success, so we should not produce FAILED
        _output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
        _diagnostics.set_failurereason("robot lost possession of the ball");
        return;
    }
    // prepare, aim, running
    *_output.mutable_motiontarget()->mutable_position() = _input.worldstate().robot().position();
    _output.mutable_motiontarget()->mutable_position()->set_rz(_ballTargetPos.rz);
    *_output.mutable_balltarget() = _input.target().position();
    _output.set_actionresult(MRA::Datatypes::ActionResult::RUNNING);
    _diagnostics.set_remainingrotationangle(_remainingRotationAngle);
    // advance to next phase?
    if (abs(_remainingRotationAngle) < angleAccuracyThreshold)
    {
        *_state.mutable_timeoutstarttimestamp() = _timestamp; // start the timer
        _output.set_actionresult(MRA::Datatypes::ActionResult::RUNNING);
        _state.set_phase(MRA::FalconsActionAimedKick::SHOOT_PHASE_AIM_FINE);
    }
}

void ActionAimedKick::phaseAimFine()
{
    MRA_TRACE_FUNCTION();
    double angleAccuracyThreshold = _params.angleaccuracythreshold();
    double timeoutDuration = _params.timeout().duration();
    if (!_input.worldstate().robot().hasball())
    {
        // in that case, functionally the shot was a success, so we should not produce FAILED
        _output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
        _diagnostics.set_failurereason("robot lost possession of the ball");
        return;
    }
    // check the timer
    auto elapsedDuration = _timestamp - _state.timeoutstarttimestamp();
    float elapsedSeconds = 1e-9 * google::protobuf::util::TimeUtil::DurationToNanoseconds(elapsedDuration);
    bool timerExpired = (elapsedSeconds > timeoutDuration);
    // check if phase ends
    if (abs(_remainingRotationAngle) > angleAccuracyThreshold && !timerExpired)
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
        _output.Clear();
        _output.set_bhenabled(true);
        _output.set_dokick(true);
        *_output.mutable_balltarget() = _input.target().position();
        *_state.mutable_dischargetimestamp() = _timestamp;
        _output.set_actionresult(MRA::Datatypes::ActionResult::RUNNING);
        _state.set_phase(MRA::FalconsActionAimedKick::SHOOT_PHASE_DISCHARGE);
        _state.set_aimtimerexpired(timerExpired); // store this for diagnostics, but in state so it does not get reset next tick
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
    float ballTargetDistance = _deltaBallTargetToCurrentBall.size();
    _output.set_actionresult(MRA::Datatypes::ActionResult::RUNNING);
    if (elapsedSeconds > _params.maxcooldownduration())
    {
        _output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
    }
    else if (!_input.worldstate().robot().hasball())
    {
        // determine success
        // if ball ends up close to target, then PASSED
        if (ballTargetDistance < _params.balltargetproximity())
        {
            _diagnostics.set_aimerror(calculateAimError());
            _output.set_actionresult(MRA::Datatypes::ActionResult::PASSED);
        }
    }
    MRA_TRACE_FUNCTION_OUTPUTS(elapsedSeconds, ballTargetDistance);
}

bool ActionAimedKick::checkParams(FalconsActionAimedKick::ParamsType const &params, std::string &failureReason)
{
    MRA_TRACE_FUNCTION();
    if (params.angleaccuracythreshold() <= 0.0)
    {
        failureReason = "invalid configuration parameter angleAccuracyThreshold: should be larger than zero";
        return false;
    }
    if (params.timeout().angleaccuracythreshold() <= 0.0)
    {
        failureReason = "invalid configuration parameter timeout.angleAccuracyThreshold: should be larger than zero";
        return false;
    }
    if (params.timeout().duration() <= 0.0)
    {
        failureReason = "invalid configuration parameter timeout.duration: should be larger than zero";
        return false;
    }
    if (params.precision().angleaccuracythreshold() <= 0.0)
    {
        failureReason = "invalid configuration parameter precision.angleAccuracyThreshold: should be larger than zero";
        return false;
    }
    if (params.precision().duration() <= 0.0)
    {
        failureReason = "invalid configuration parameter precision.duration: should be larger than zero";
        return false;
    }
    return true;
}