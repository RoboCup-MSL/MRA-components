/*
 * SPGVelocitySetpointController.cpp
 * Started as copy of file with same name in the Falcons variant of Velocity Control
 */

#include "SPGVelocitySetpointController.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include <logging.hpp>
#include <ruckig/ruckig.hpp>

using namespace ruckig;
using namespace MRA;

SPGVelocitySetpointController::SPGVelocitySetpointController() {}

SPGVelocitySetpointController::~SPGVelocitySetpointController() {}

bool SPGVelocitySetpointController::calculate(VelocityControlData &data) {
    // Ruckig Motion Library with jerk control support
    // in order to achieve higher deceleration, a layer is added around the SPG algorithm
    // we start with (aggressive) deceleration limits, so the robot is normally going to be in time for braking
    // and we get a good estimate of velocity and acceleration setpoint
    // then, if robot _seems_ to be accelerating, corresponding limits are used in a recalculation
    SpgLimits spgLimits;
    spgLimits.vx = data.limits.maxvel().x();
    spgLimits.vy = data.limits.maxvel().yforward();
    spgLimits.vRz = data.limits.maxvel().rz();
    spgLimits.vxy = data.limits.maxvel().xy();
    spgLimits.ax = data.limits.maxdec().x();
    spgLimits.ay = data.limits.maxdec().y();
    spgLimits.aRz = data.limits.maxdec().rz();
    spgLimits.axy = data.limits.maxdec().xy();
    spgLimits.hasJerkLimit = data.limits.has_maxjerk();
    if (spgLimits.hasJerkLimit) {
        spgLimits.jx = data.limits.maxjerk().x();
        spgLimits.jy = data.limits.maxjerk().y();
        spgLimits.jRz = data.limits.maxjerk().rz();
    }


    // For position, finding a weighted average on the Rz is not trivial when the angles are around the boundary of [0,
    // 2pi]. To solve this, rotate both currentPosFCS.Rz and previousPosSetpointFCS.Rz towards currentPosFCS.Rz. This
    // means currentPosFCS.Rz = 0, and previousPosSetpointFCS.Rz is the difference to currentPosFCS.Rz. At this point,
    // take the weighted factor from previousPosSetpointFCS.Rz -> deltaAngle projected between [-pi, pi] This weighted
    // factor can be rotated back from currentPosFCS.Rz. The final average angle between the two is then
    // currentPosFCS.Rz + deltaAngle

    float w_pos = data.config.spg().weightfactorclosedlooppos();
    float w_vel = data.config.spg().weightfactorclosedloopvel();

    double deltaRz = 0.0;
    if (data.state.executed_before()) {
        deltaRz = MRA::Geometry::wrap_pi(data.previousPositionSetpointFcs.rz - data.currentPositionFcs.rz) * w_pos;
    }
    double weightedRz = data.currentPositionFcs.rz + deltaRz;

    Position2D weightedCurrentPositionFCS = data.currentPositionFcs;
    if (data.state.executed_before()) {
        weightedCurrentPositionFCS = data.currentPositionFcs * w_pos + data.previousPositionSetpointFcs * (1.0 - w_pos);
    }

    weightedCurrentPositionFCS.rz = weightedRz;
    Velocity2D weightedCurrentVelocityFCS = data.currentVelocityFcs;
    if (data.state.executed_before()) {
        weightedCurrentVelocityFCS = data.currentVelocityFcs * w_vel + data.previousVelocitySetpointFcs * (1.0 - w_vel);
    }

    m_deltaPositionRCS = Position2D(data.targetPositionFcs).transformFcsToRcs(weightedCurrentPositionFCS);
    m_deltaPositionRCS.rz = MRA::Geometry::wrap_pi(data.targetPositionFcs.rz - weightedCurrentPositionFCS.rz);
    m_currentVelocityRCS = Velocity2D(weightedCurrentVelocityFCS).transformFcsToRcs(weightedCurrentPositionFCS);
    m_targetVelocityRCS = Velocity2D(data.targetVelocityFcs).transformFcsToRcs(weightedCurrentPositionFCS);

    VelocityControlResult result = {};
    bool res = this->calculateSPG(data, spgLimits, result);

    bool recalculate = false;

    // check if velocity of XY is below the limit
    if (std::hypot(result.velocityRcs.x, result.velocityRcs.y) > data.limits.maxvel().xy()) {
        recalculate = true;
        // clip velocity X and Y: in proporition
        auto q = std::atan2(result.velocityRcs.y,result.velocityRcs.x);
        auto max_vel_xy = data.limits.maxvel().xy();

        spgLimits.vx = max_vel_xy * cos(q);
        spgLimits.vy = max_vel_xy * sin(q);
    }

    // check if acceleration of XY is below the limit
    if (std::hypot(result.acceleration.x, result.acceleration.y) > data.limits.maxacc().xy()) {
        recalculate = true;
        // clip acceleration X and Y: in proporition
        auto q = std::atan2(result.acceleration.y,result.acceleration.x);
        auto max_acc_xy = data.limits.maxacc().xy();
        spgLimits.ax = max_acc_xy * cos(q);
        spgLimits.ay = max_acc_xy * sin(q);
    }

    if (abs(result.acceleration.x) > data.limits.accthreshold().x()) {
        recalculate = true;
        spgLimits.ax = data.limits.maxacc().x();
    }

    if (abs(result.acceleration.y) > data.limits.accthreshold().y()) {
        recalculate = true;
        spgLimits.ay = (result.velocityRcs.y < 0.0) ? data.limits.maxacc().ybackward() : data.limits.maxacc().yforward();
    }

    if (abs(result.acceleration.rz) > data.limits.accthreshold().rz()) {
        recalculate = true;
        spgLimits.aRz = data.limits.maxacc().rz();
    }

    if (result.velocityRcs.y < 0.0) {
        recalculate = true;
        spgLimits.vy = data.limits.maxvel().ybackward();
    }

    // recalculate?
    if (recalculate) {
        result = {};
        res = calculateSPG(data, spgLimits, result);
    }

    // Done -- store output and values for next iteration
    data.result.velocityRcs = result.velocityRcs;
    data.ruckig_info = result;


    // Store previousPositionSetpointFcs
    Position2D tmpPos = result.positionRcs;
    data.previousPositionSetpointFcs = tmpPos.transformRcsToFcs(weightedCurrentPositionFCS);

    // Store previousVelocitySetpointFcs
    Velocity2D tmpVel = result.velocityRcs;
    data.previousVelocitySetpointFcs = tmpVel.transformRcsToFcs(weightedCurrentPositionFCS);

    return res;
}


bool SPGVelocitySetpointController::calculateSPG(const VelocityControlData& r_data, const SpgLimits& r_spgLimits, 
                                                 VelocityControlResult& r_result) {
    bool res = false;

    if (r_data.controlMode != MRA::RobotsportsVelocityControl::ControlModeEnum::VEL_ONLY) {
        // POS_ONLY or POSVEL
        if (r_data.config.spg().synchronizerotation()) {
            res = calculatePosXYRzPhaseSynchronized(r_data, r_spgLimits, r_result);
        } else {
            res = calculatePosXYPhaseSynchronized(r_data, r_spgLimits, r_result);
            res = calculatePosRzNonSynchronized(r_data, r_spgLimits, r_result);
        }
    } else {
        // Reflexxes has a problem when velocity already reached (start with assuming that ruckig has this issue too)
        double EPSILON = 1E-4;
        if (abs(m_currentVelocityRCS.x  - std::clamp(m_targetVelocityRCS.x, (double)-r_spgLimits.vx, (double)r_spgLimits.vx)) < EPSILON &&
            abs(m_currentVelocityRCS.y  - std::clamp(m_targetVelocityRCS.y, (double)-r_spgLimits.vy, (double)r_spgLimits.vy)) < EPSILON &&
            abs(m_currentVelocityRCS.rz - std::clamp(m_targetVelocityRCS.rz, (double)-r_spgLimits.vRz, (double)r_spgLimits.vRz)) < EPSILON) {
            r_result.velocityRcs = Velocity2D(m_currentVelocityRCS);
            res = true;
        } else {
            res = calculateVelXYRzPhaseSynchronized(r_data, r_spgLimits, r_result);
        }
    }
    return res;
}

bool SPGVelocitySetpointController::calculatePosXYRzPhaseSynchronized(const VelocityControlData& r_data,
                                                                      const SpgLimits &r_spgLimits,
                                                                      VelocityControlResult& r_result) {
    const int numberOfDOFs = 3; // degrees of freedom (x,y,Rz)
    InputParameter<numberOfDOFs> input;

    // instead of steering from current to target,
    // we steer from zero to delta, so we can better configure controlling FCS or RCS
    input.current_position = {0.0, 0.0, 0.0};
    input.current_velocity = {m_currentVelocityRCS.x, m_currentVelocityRCS.y, m_currentVelocityRCS.rz};
    input.current_acceleration = {0.0, 0.0,
                                  0.0}; // not relevant, due to limitation of TypeII library  (TODO update for ruckig)

    input.max_velocity = {r_spgLimits.vx, r_spgLimits.vy, r_spgLimits.vRz};
    input.max_acceleration = {r_spgLimits.ax, r_spgLimits.ay, r_spgLimits.aRz};
    if (r_spgLimits.hasJerkLimit) {
        input.max_jerk = {r_spgLimits.jx, r_spgLimits.jy, r_spgLimits.jRz};
    }

    // steering from currentPos = 0, to targetPos = deltaPos
    input.target_position = {m_deltaPositionRCS.x, m_deltaPositionRCS.y, m_deltaPositionRCS.rz};
    input.target_velocity = {m_targetVelocityRCS.x, m_targetVelocityRCS.y, m_targetVelocityRCS.rz};
    //      input.target_acceleration = {0.0, 0.0, 0.5}; // TODO ruckig

    // Phase synchronize the DoFs when this is possible,
    // else fall back to time (default: always synchronize the DoFs to reach the target on the same time)
    input.synchronization = Synchronization::Phase;

    return ruckig_calculate<numberOfDOFs>(r_data, input, AXES::AXES_XYRZ, r_result);
}

bool SPGVelocitySetpointController::calculatePosXYPhaseSynchronized(const VelocityControlData &r_data,
                                                                    const SpgLimits& r_spgLimits,
                                                                    VelocityControlResult& r_result) {
    const int numberOfDOFs = 2; // degrees of freedom (x,y)
    InputParameter<numberOfDOFs> input;

    // set-up the input parameters
    input.current_position = {
        0.0,
        0.0}; // instead of steering from current to target, we steer from zero to delta, so we can better configure
    input.current_velocity = {m_currentVelocityRCS.x, m_currentVelocityRCS.y};
    input.current_acceleration = {0.0, 0.0}; // not relevant, due to limitation of TypeII library // TODO ruckig

    input.max_velocity = {r_spgLimits.vx, r_spgLimits.vy};
    input.max_acceleration = {r_spgLimits.ax, r_spgLimits.ay};

    if (r_spgLimits.hasJerkLimit) {
        input.max_jerk = {r_spgLimits.jx, r_spgLimits.jy};
    }

    input.target_position = {m_deltaPositionRCS.x, m_deltaPositionRCS.y}; // steering from currentPos = 0 to targetPos = deltaPos
    input.target_velocity = {m_targetVelocityRCS.x, m_targetVelocityRCS.y};

    // Phase synchronize the DoFs when this is possible,
    // else fall back to time (default: always synchronize the DoFs to reach the target on the same time)
    input.synchronization = Synchronization::Phase;

    return ruckig_calculate<numberOfDOFs>(r_data, input, AXES::AXES_XY, r_result);
}


bool SPGVelocitySetpointController::calculatePosRzNonSynchronized(const VelocityControlData &r_data, 
                                                                  const SpgLimits& r_spgLimits,
                                                                  VelocityControlResult& r_result) {
    const int numberOfDOFs = 1; // degrees of freedom (Rz)
    InputParameter<numberOfDOFs> input;

    // set-up the input parameters
    input.current_position[0] = 0.0; // controlling FCS or RCS
    input.current_velocity[0] = m_currentVelocityRCS.rz;
    input.current_acceleration[0] = 0.0;
    input.max_velocity[0] = r_spgLimits.vRz;
    input.max_acceleration[0] = r_spgLimits.aRz;
    if (r_spgLimits.hasJerkLimit) {
        input.max_jerk = {r_spgLimits.jRz};
    }
    input.target_position[0] = m_deltaPositionRCS.rz;
    input.target_velocity[0] = m_targetVelocityRCS.rz;

    // Calculate every DoF independently
    input.synchronization = Synchronization::None;

    return ruckig_calculate<numberOfDOFs>(r_data, input, AXES::AXES_RZ, r_result );
}


bool SPGVelocitySetpointController::calculateVelXYRzPhaseSynchronized(const VelocityControlData &r_data,
                                                                      const SpgLimits& r_spgLimits,
                                                                      VelocityControlResult& r_result) {
    const int numberOfDOFs = 3; // degrees of freedom (X, Y, Rz)
    InputParameter<numberOfDOFs> input = {};

    // set-up the input parameters
    input.control_interface = ControlInterface::Velocity;

    input.current_position[0] = 0.0; // instead of steering from current to target,
    input.current_position[1] = 0.0; // we steer from zero to delta, so we can better configure
    input.current_position[2] = 0.0; // controlling FCS or RCS

    input.current_velocity[0] = m_currentVelocityRCS.x;
    input.current_velocity[1] = m_currentVelocityRCS.y;
    input.current_velocity[2] = m_currentVelocityRCS.rz;

    input.current_acceleration[0] = 0.0; // not relevant, due to limitation of TypeII library
    input.current_acceleration[1] = 0.0;
    input.current_acceleration[2] = 0.0;

    input.max_velocity = {r_spgLimits.vx, r_spgLimits.vy, r_spgLimits.vRz};
    input.max_acceleration = {r_spgLimits.ax, r_spgLimits.ay, r_spgLimits.aRz};

    if (r_spgLimits.hasJerkLimit) {
        input.max_jerk = {r_spgLimits.jx, r_spgLimits.jy, r_spgLimits.jRz};
    }

    input.target_position = {};
    input.target_velocity = {m_targetVelocityRCS.x, m_targetVelocityRCS.y, m_targetVelocityRCS.rz};

    // Phase synchronize the DoFs when this is possible,
    // else fall back to time (default: always synchronize the DoFs to reach the target on the same time)
    input.synchronization = Synchronization::Phase;

    return ruckig_calculate<numberOfDOFs>(r_data, input, AXES::AXES_XYRZ, r_result);
}


template<size_t DOFs, template<class, size_t> class CustomVector>
bool SPGVelocitySetpointController::ruckig_calculate(const MRA::internal::RVC::VelocityControlData & r_data,
                                                     ruckig::InputParameter<DOFs, CustomVector>  &r_input,
                                                     AXES axes,
                                                     MRA::internal::RVC::VelocityControlResult & r_result)
{
    for (auto dof = 0u; dof < DOFs; dof++) {
        r_input.target_velocity[dof] = std::clamp(r_input.target_velocity[dof], (double) -r_input.max_velocity[dof], r_input.max_velocity[dof]);
    }

    // output parameters have been evaluated at first tick (data.config.dt())
    // latency correction: evaluate the trajectory at some offset
    double new_time = r_data.config.dt() + r_data.config.spg().latencyoffset(); // TODO why not set new_time as sample_rate?

    // std::cout << r_input.to_string() << std::endl  << std::flush;
    Ruckig<DOFs> otg(new_time);
    OutputParameter<DOFs> output = {};
    auto res = otg.update(r_input, output);
    // std::cout << output.to_string() << std::endl << std::flush;


    if (not(res == Working or res == Finished)) {
        std::string msg;
        if (res == Error) {
            msg = "ruckig.update return Error (< Unclassified error >)";
        } else if (res == ErrorInvalidInput) {
            msg = "ruckig.update return ErrorInvalidInput (< Error in the input parameter >)";
        } else if (res == ErrorTrajectoryDuration) {
            msg = "ruckig.update return ErrorTrajectoryDuration (< The trajectory duration exceeds its numerical "
                  "limits >";
        } else if (res == ErrorPositionalLimits) {
            msg = "ruckig.update return ErrorPositionalLimits (< The trajectory exceeds the given positional limits "
                  "(only in Ruckig Pro) >";
        } else if (res == ErrorZeroLimits) {
            msg = "ruckig.update return ErrorZeroLimits (< The trajectory is not valid due to a conflict with zero "
                  "limits >)";
        } else if (res == ErrorExecutionTimeCalculation) {
            msg = "ruckig.update return ErrorExecutionTimeCalculation (< Error during the extreme time calculation "
                  "(Step 1) >)";
        } else if (res == ErrorSynchronizationCalculation) {
            msg = "ruckig.update return ErrorSynchronizationCalculation (< Error during the synchronization "
                  "calculation (Step 2) >)";
        } else {
            msg = "ruckig.update return an unknown error";
        }
        MRA_LOG_ERROR(msg);
    }

    if (res == ErrorInvalidInput) {
        MRA_LOG_INFO("Invalid input for ruckig %s", input.to_string().c_str());
        return false;
    }


    if ((axes == AXES_XYRZ) or (axes == AXES_XY)) {
        r_result.velocityRcs.x = output.new_velocity[0];
        r_result.velocityRcs.y = output.new_velocity[1];

        r_result.positionRcs.x = output.new_position[0];
        r_result.positionRcs.y = output.new_position[1];

        r_result.acceleration.x = output.new_acceleration[0];
        r_result.acceleration.y = output.new_acceleration[1];
    }
    
    if (axes == AXES_XYRZ) {
        r_result.velocityRcs.rz = output.new_velocity[2];
        r_result.positionRcs.rz = output.new_position[2];
        r_result.acceleration.rz = output.new_acceleration[2];
    }
    else if (axes == AXES_RZ)  {
        r_result.velocityRcs.rz = output.new_velocity[0];
        r_result.positionRcs.rz = output.new_position[0];
        r_result.acceleration.rz = output.new_acceleration[0];
    }

    /* TODO  Diag data  (for the defined axes)
    data.spgCurrentPosition.x  = input.current_position[0];
    data.spgCurrentVelocity.x  = input.current_velocity[0];
    data.spgMaxVelocity.x  = input.max_velocity[0];
    data.spgMaxAcceleration.x  = input.max_acceleration[0];
    data.spgTargetPosition.x  = input.target_position[0];
    data.spgTargetVelocity.x  = input.target_velocity[0];
    data.spgNewPosition.x  = new_position[0];
    data.spgNewVelocity.x  = new_velocity[0];
    */

    return true;
}
