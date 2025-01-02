/*
 * SPGVelocitySetpointController.cpp
 * Started as copy of file with same name in the Falcons variant of Velocity Control
 */

// TODO: reduce excessive code duplication, improve maintainability

#include "SPGVelocitySetpointController.hpp"

//#include <algorithm>
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
    spgLimits.ax = data.limits.maxdec().x();
    spgLimits.ay = data.limits.maxdec().y();
    spgLimits.aRz = data.limits.maxdec().rz();
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
    double deltaRz = MRA::Geometry::wrap_pi(data.previousPositionSetpointFcs.rz - data.currentPositionFcs.rz) * w_pos;
    double weightedRz = data.currentPositionFcs.rz + deltaRz;

    Position2D weightedCurrentPositionFCS =
        data.currentPositionFcs * w_pos + data.previousPositionSetpointFcs * (1.0 - w_pos);
    weightedCurrentPositionFCS.rz = weightedRz;
    float w_vel = data.config.spg().weightfactorclosedloopvel();
    Velocity2D weightedCurrentVelocityFCS =
        data.currentVelocityFcs * w_vel + data.previousVelocitySetpointFcs * (1.0 - w_vel);

    m_deltaPositionRCS = Position2D(data.targetPositionFcs).transformFcsToRcs(weightedCurrentPositionFCS);
    m_deltaPositionRCS.rz = MRA::Geometry::wrap_pi(data.targetPositionFcs.rz - weightedCurrentPositionFCS.rz);
    m_currentVelocityRCS = Velocity2D(weightedCurrentVelocityFCS).transformFcsToRcs(weightedCurrentPositionFCS);
    m_targetVelocityRCS = Velocity2D(data.targetVelocityFcs).transformFcsToRcs(weightedCurrentPositionFCS);

    Position2D resultPosition;
    Velocity2D resultVelocity;
    bool result = this->calculateSPG(data, spgLimits, resultPosition, resultVelocity);

    bool recalculate = false;
    if (isDofAccelerating(data, resultVelocity, 0, data.limits.accthreshold().x())) {
        recalculate = true;
        spgLimits.ax = data.limits.maxacc().x();
    }
    if (isDofAccelerating(data, resultVelocity, 1, data.limits.accthreshold().y())) {
        recalculate = true;
        spgLimits.ay = (resultVelocity.y < 0.0) ? data.limits.maxacc().ybackward() : data.limits.maxacc().yforward();
    }
    if (isDofAccelerating(data, resultVelocity, 2, data.limits.accthreshold().rz())) {
        recalculate = true;
        spgLimits.aRz = data.limits.maxacc().rz();
    }
    if (resultVelocity.y < 0.0) {
        recalculate = true;
        spgLimits.vy = data.limits.maxvel().ybackward();
    }

    // recalculate?
    if (recalculate) {
        result = calculateSPG(data, spgLimits, resultPosition, resultVelocity);
    }

    // check if motor limits are not exceeded.
    /* disabled - in MRA context, we must not depend on vtClient
    for(int i = 0; i < 10; i++)
    {
        if (data.vtClient.exceedsMotorLimits( pose(resultVelocity.x, resultVelocity.y, resultVelocity.rz) ))
        {
            // reduce limits by 20% to see if we can find setpoints which do not exceed the motor limits
            spgLimits.vx *= 0.8;
            spgLimits.vy *= 0.8;
            spgLimits.vRz *= 0.8;
            result = calculateSPG(data, spgLimits, resultPosition, resultVelocity);
        }
        else
        {
            break;
        }
    }
    */

    // Done -- store output and values for next iteration
    data.resultVelocityRcs = resultVelocity;

    // Store previousPositionSetpointFcs for open loop control
    Position2D tmpPos = resultPosition;
    data.previousPositionSetpointFcs = tmpPos.transformRcsToFcs(weightedCurrentPositionFCS);
    // ???data.previousPositionSetpointFcs.rz += M_PI_2;
    // ??? data.previousPositionSetpointFcs.rz = project_angle_0_2pi(data.previousPositionSetpointFcs.rz);

    // Store previousVelocitySetpointFcs for open loop control
    Velocity2D tmpVel = resultVelocity;
    data.previousVelocitySetpointFcs = tmpVel.transformRcsToFcs(weightedCurrentPositionFCS);

    return result;
}

bool SPGVelocitySetpointController::isDofAccelerating(const VelocityControlData &data, const Velocity2D &resultVelocity,
                                                      int dof, float threshold) {
    // To check if a DOF is accelerating, we should look if the m_currentVelocityRCS -> resultVelocity is "moving away
    // from zero".
    std::vector<double> currentVelocity;
    currentVelocity.push_back(m_currentVelocityRCS.x);
    currentVelocity.push_back(m_currentVelocityRCS.y);
    currentVelocity.push_back(m_currentVelocityRCS.rz);

    std::vector<double> newVelocity;
    newVelocity.push_back(resultVelocity.x);
    newVelocity.push_back(resultVelocity.y);
    newVelocity.push_back(resultVelocity.rz);

    if (currentVelocity.at(dof) < 0.0) {
        // newVelocity - currentVelocity < threshold
        // e.g., -2.0 - -1.0 = -1.0 < (-threshold)
        return (newVelocity.at(dof) - currentVelocity.at(dof)) < (-threshold);
    } else if (currentVelocity.at(dof) > 0.0) {
        // newVelocity - currentVelocity > threshold
        // e.g., 2.0 - 1.0 = 1.0 > threshold
        return (newVelocity.at(dof) - currentVelocity.at(dof)) > threshold;
    } else {
        // currentVelocity == 0.0
        return abs(newVelocity.at(dof)) > threshold;
    }

    return true;
}

bool SPGVelocitySetpointController::calculateSPG(VelocityControlData &data, SpgLimits const &spgLimits,
                                                 Position2D &resultPosition, Velocity2D &resultVelocity) {
    bool result = false;

    if (data.controlMode != MRA::RobotsportsVelocityControl::ControlModeEnum::VEL_ONLY) {
        // POS_ONLY or POSVEL
        if (data.config.spg().synchronizerotation()) {
            result = calculatePosXYRzPhaseSynchronized(data, spgLimits, resultPosition, resultVelocity);
        } else {
            result = calculatePosXYPhaseSynchronized(data, spgLimits, resultPosition, resultVelocity);
            result = calculatePosRzNonSynchronized(data, spgLimits, resultPosition, resultVelocity);
        }
    } else {
        // Reflexxes has a problem when velocity already reached (start with assuming that ruckig has this issue too)
        double EPSILON = 1E-4;
        if (abs(m_currentVelocityRCS.x -
                std::clamp(m_targetVelocityRCS.x, (double)-spgLimits.vx, (double)spgLimits.vx)) < EPSILON &&
            abs(m_currentVelocityRCS.y -
                std::clamp(m_targetVelocityRCS.y, (double)-spgLimits.vy, (double)spgLimits.vy)) < EPSILON &&
            abs(m_currentVelocityRCS.rz -
                std::clamp(m_targetVelocityRCS.rz, (double)-spgLimits.vRz, (double)spgLimits.vRz)) < EPSILON) {
            resultVelocity = Velocity2D(m_currentVelocityRCS);
            return true;
        } else {
            result = calculateVelXYRzPhaseSynchronized(data, spgLimits, resultPosition, resultVelocity);
        }
    }
    return result;
}

bool SPGVelocitySetpointController::calculatePosXYRzPhaseSynchronized(VelocityControlData &data,
                                                                      const SpgLimits &spgLimits,
                                                                      Position2D &resultPosition,
                                                                      Velocity2D &resultVelocity) {
    //    Flags.SynchronizationBehavior                  = RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
    //    Flags.BehaviorAfterFinalStateOfMotionIsReached = RMLPositionFlags::RECOMPUTE_TRAJECTORY;
    //    // construct Reflexxes objects

    const int numberOfDOFs = 3; // degrees of freedom (x,y,Rz)
    Ruckig<numberOfDOFs> otg;
    Trajectory<numberOfDOFs> trajectory;

    InputParameter<numberOfDOFs> input;

    // instead of steering from current to target,
    // we steer from zero to delta, so we can better configure controlling FCS or RCS
    input.current_position = {0.0, 0.0, 0.0};
    input.current_velocity = {m_currentVelocityRCS.x, m_currentVelocityRCS.y, m_currentVelocityRCS.rz};
    input.current_acceleration = {0.0, 0.0,
                                  0.0}; // not relevant, due to limitation of TypeII library  (TODO update for ruckig)

    input.max_velocity = {spgLimits.vx, spgLimits.vy, spgLimits.vRz};
    input.max_acceleration = {spgLimits.ax, spgLimits.ay, spgLimits.aRz};
    if (spgLimits.hasJerkLimit) {
        input.max_jerk = {spgLimits.jx, spgLimits.jy, spgLimits.jRz};
    }

    // steering from currentPos = 0, to targetPos = deltaPos
    input.target_position = {m_deltaPositionRCS.x, m_deltaPositionRCS.y, m_deltaPositionRCS.rz};
    input.target_velocity = {m_targetVelocityRCS.x, m_targetVelocityRCS.y, m_targetVelocityRCS.rz};
    //      input.target_acceleration = {0.0, 0.0, 0.5}; // TODO ruckig

    auto result = otg.calculate(input, trajectory);
    if (result == ErrorInvalidInput) {
        return false;
    }

    // output parameters have been evaluated at first tick (data.config.dt())
    // latency correction: evaluate the trajectory at some offset
    double new_time = data.config.dt() + data.config.spg().latencyoffset(); // TODO why not set new_time as sample_rate?
    std::array<double, numberOfDOFs> new_position, new_velocity, new_acceleration, new_jerk;
    size_t new_section;
    trajectory.at_time(new_time, new_position, new_velocity, new_acceleration, new_jerk, new_section);

    // convert outputs
    resultPosition.x = new_position[0];
    resultPosition.y = new_position[1];
    resultPosition.rz = new_position[2];

    resultVelocity.x = new_velocity[0];
    resultVelocity.y = new_velocity[1];
    resultVelocity.rz = new_velocity[2];

    // Diag data
    /* TODO
    data.spgCurrentPosition.x  = input.current_position[0];
    data.spgCurrentPosition.y  = input.current_position[1];
    data.spgCurrentPosition.Rz = input.current_position[2];

    data.spgCurrentVelocity.x  = input.current_velocity[0];
    data.spgCurrentVelocity.y  = input.current_velocity[1];
    data.spgCurrentVelocity.Rz = input.current_velocity[2];

    data.spgMaxVelocity.x  = input.max_velocity[0];
    data.spgMaxVelocity.y  = input.max_velocity[1];
    data.spgMaxVelocity.Rz = input.max_velocity[2];

    data.spgMaxAcceleration.x  = input.max_acceleration[0];
    data.spgMaxAcceleration.y  = input.max_acceleration[1];
    data.spgMaxAcceleration.Rz = input.max_acceleration[2];

    data.spgTargetPosition.x  = input.target_position[0];
    data.spgTargetPosition.y  = input.target_position[1];
    data.spgTargetPosition.Rz = input.target_position[2];

    data.spgTargetVelocity.x  = input.target_velocity[0];
    data.spgTargetVelocity.y  = input.target_velocity[1];
    data.spgTargetVelocity.Rz = input.target_velocity[2];

    data.spgNewPosition.x  = new_position[0];
    data.spgNewPosition.y  = new_position[1];
    data.spgNewPosition.Rz = new_position[2];

    data.spgNewVelocity.x  = new_velocity[0];
    data.spgNewVelocity.y  = new_velocity[1];
    data.spgNewVelocity.Rz = new_velocity[2];
    */
    return true;
}

bool SPGVelocitySetpointController::calculatePosXYPhaseSynchronized(VelocityControlData &data,
                                                                    SpgLimits const &spgLimits,
                                                                    Position2D &resultPosition,
                                                                    Velocity2D &resultVelocity) {
    //    RMLPositionFlags            Flags;
    //    Flags.SynchronizationBehavior                  = RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
    //    Flags.BehaviorAfterFinalStateOfMotionIsReached = RMLPositionFlags::RECOMPUTE_TRAJECTORY;

    const int numberOfDOFs = 2; // degrees of freedom (x,y)
    Ruckig<numberOfDOFs> otg;
    Trajectory<numberOfDOFs> trajectory;

    InputParameter<numberOfDOFs> input;

    // set-up the input parameters
    input.current_position = {
        0.0,
        0.0}; // instead of steering from current to target, we steer from zero to delta, so we can better configure
    input.current_velocity = {m_currentVelocityRCS.x, m_currentVelocityRCS.y};
    input.current_acceleration = {0.0, 0.0}; // not relevant, due to limitation of TypeII library // TODO ruckig

    input.max_velocity = {spgLimits.vx, spgLimits.vy};
    input.max_acceleration = {spgLimits.ax, spgLimits.ay};

    if (spgLimits.hasJerkLimit) {
        input.max_jerk = {spgLimits.jx, spgLimits.jy};
    }

    input.target_position = {m_deltaPositionRCS.x,
                             m_deltaPositionRCS.y}; // steering from currentPos = 0 to targetPos = deltaPos
    input.target_velocity = {m_targetVelocityRCS.x, m_targetVelocityRCS.y};

    auto result = otg.calculate(input, trajectory);
    if (result == ErrorInvalidInput) {
        return false;
    }

    // output parameters have been evaluated at first tick (data.config.dt())
    // latency correction: evaluate the trajectory at some offset
    double new_time = data.config.dt() + data.config.spg().latencyoffset(); // TODO why not set new_time as sample_rate?
    std::array<double, numberOfDOFs> new_position, new_velocity, new_acceleration, new_jerk;
    size_t new_section;
    trajectory.at_time(new_time, new_position, new_velocity, new_acceleration, new_jerk, new_section);

    // convert outputs
    resultPosition.x = new_position[0];
    resultPosition.y = new_position[1];

    resultVelocity.x = new_velocity[0];
    resultVelocity.y = new_velocity[1];

    // Diag data
    /*TODO
    data.spgCurrentPosition.x  = input.current_position[0];
    data.spgCurrentPosition.y  = input.current_position[1];

    data.spgCurrentVelocity.x  = input.current_velocity[0];
    data.spgCurrentVelocity.y  = input.current_velocity[1];

    data.spgMaxVelocity.x  = input.max_velocity[0];
    data.spgMaxVelocity.y  = input.max_velocity[1];

    data.spgMaxAcceleration.x  = input.max_acceleration[0];
    data.spgMaxAcceleration.y  = input.max_acceleration[1];

    data.spgTargetPosition.x  = input.target_position[0];
    data.spgTargetPosition.y  = input.target_position[1];

    data.spgTargetVelocity.x  = input.target_velocity[0];
    data.spgTargetVelocity.y  = input.target_velocity[1];

    data.spgNewPosition.x  = new_position[0];
    data.spgNewPosition.y  = new_position[1];

    data.spgNewVelocity.x  = new_velocity[0];
    data.spgNewVelocity.y  = new_velocity[1];
    */

    return true;
}

bool SPGVelocitySetpointController::calculatePosRzNonSynchronized(VelocityControlData &data, SpgLimits const &spgLimits,
                                                                  Position2D &resultPosition,
                                                                  Velocity2D &resultVelocity) {
    //    RMLPositionFlags            Flags;
    //    Flags.SynchronizationBehavior                  = RMLPositionFlags::NO_SYNCHRONIZATION;
    //    Flags.BehaviorAfterFinalStateOfMotionIsReached = RMLPositionFlags::RECOMPUTE_TRAJECTORY;
    //
    const int numberOfDOFs = 1; // degrees of freedom (Rz)
    Ruckig<numberOfDOFs> otg;
    Trajectory<numberOfDOFs> trajectory;

    InputParameter<numberOfDOFs> input;

    // set-up the input parameters
    input.current_position[0] = 0.0; // controlling FCS or RCS
    input.current_velocity[0] = m_currentVelocityRCS.rz;
    input.current_acceleration[0] = 0.0;
    input.max_velocity[0] = spgLimits.vRz;
    input.max_acceleration[0] = spgLimits.aRz;
    if (spgLimits.hasJerkLimit) {
        input.max_jerk = {spgLimits.jRz};
    }
    input.target_position[0] = m_deltaPositionRCS.rz;
    input.target_velocity[0] = m_targetVelocityRCS.rz;

    auto result = otg.calculate(input, trajectory);
    if (result == ErrorInvalidInput) {
        return false;
    }

    // output parameters have been evaluated at first tick (data.config.dt())
    // latency correction: evaluate the trajectory at some offset
    double new_time = data.config.dt() + data.config.spg().latencyoffset(); // TODO why not set new_time as sample_rate?
    std::array<double, numberOfDOFs> new_position, new_velocity, new_acceleration, new_jerk;
    size_t new_section;
    trajectory.at_time(new_time, new_position, new_velocity, new_acceleration, new_jerk, new_section);
    // convert outputs
    resultPosition.rz = new_position[0];
    resultVelocity.rz = new_velocity[0];

    // Diag data
    /*TODO
    data.spgCurrentPosition.Rz = input.current_position[0];
    data.spgCurrentVelocity.Rz = input.current_velocity[0];
    data.spgMaxVelocity.Rz = input.max_velocity[0];
    data.spgMaxAcceleration.Rz = input.max_acceleration[0];
    data.spgTargetPosition.Rz = input.target_position[0];
    data.spgTargetVelocity.Rz = input.target_velocity[0];
    data.spgNewPosition.Rz = new_position[0];
    data.spgNewVelocity.Rz = new_velocity[0];
    */

    return true;
}

void SPGVelocitySetpointController::checkRuckigResult(ruckig::Result result) {
    if (not(result == Working or result == Finished)) {
        std::string msg;
        if (result == Error) {
            msg = "ruckig.update return Error (< Unclassified error >)";
        } else if (result == ErrorInvalidInput) {
            msg = "ruckig.update return ErrorInvalidInput (< Error in the input parameter >)";
        } else if (result == ErrorTrajectoryDuration) {
            msg = "ruckig.update return ErrorTrajectoryDuration (< The trajectory duration exceeds its numerical "
                  "limits >";
        } else if (result == ErrorPositionalLimits) {
            msg = "ruckig.update return ErrorPositionalLimits (< The trajectory exceeds the given positional limits "
                  "(only in Ruckig Pro) >";
        } else if (result == ErrorZeroLimits) {
            msg = "ruckig.update return ErrorZeroLimits (< The trajectory is not valid due to a conflict with zero "
                  "limits >)";
        } else if (result == ErrorExecutionTimeCalculation) {
            msg = "ruckig.update return ErrorExecutionTimeCalculation (< Error during the extreme time calculation "
                  "(Step 1) >)";
        } else if (result == ErrorSynchronizationCalculation) {
            msg = "ruckig.update return ErrorSynchronizationCalculation (< Error during the synchronization "
                  "calculation (Step 2) >)";
        } else {
            msg = "ruckig.update return an unknown error";
        }
        MRA_LOG_ERROR(msg);
    }
}

bool SPGVelocitySetpointController::calculateVelXYRzPhaseSynchronized(VelocityControlData &data,
                                                                      SpgLimits const &spgLimits,
                                                                      Position2D &resultPosition,
                                                                      Velocity2D &resultVelocity) {
    const int numberOfDOFs = 3; // degrees of freedom (X, Y, Rz)
    Ruckig<numberOfDOFs> otg;
    Trajectory<numberOfDOFs> trajectory;

    InputParameter<numberOfDOFs> input;

    // set-up the input parameters
    input.current_position[0] = 0.0; // instead of steering from current to target,
    input.current_position[1] = 0.0; // we steer from zero to delta, so we can better configure
    input.current_position[2] = 0.0; // controlling FCS or RCS

    input.current_velocity[0] = m_currentVelocityRCS.x;
    input.current_velocity[1] = m_currentVelocityRCS.y;
    input.current_velocity[2] = m_currentVelocityRCS.rz;

    input.current_acceleration[0] = 0.0; // not relevant, due to limitation of TypeII library
    input.current_acceleration[1] = 0.0;
    input.current_acceleration[2] = 0.0;

    input.max_velocity = {spgLimits.vx, spgLimits.vy, spgLimits.vRz};
    input.max_acceleration = {spgLimits.ax, spgLimits.ay, spgLimits.aRz};

    if (spgLimits.hasJerkLimit) {
        input.max_jerk = {spgLimits.jx, spgLimits.jy, spgLimits.jRz};
    }

    input.target_position = {};
    input.target_velocity = {data.targetVelocityFcs.x, data.targetVelocityFcs.y, data.targetVelocityFcs.rz};

    auto result = otg.calculate(input, trajectory);
    checkRuckigResult(result);
    if (result == ErrorInvalidInput) {
        MRA_LOG_INFO("Invalid input for ruckig %s", input.to_string().c_str());
        return false;
    }

    // output parameters have been evaluated at first tick (data.config.dt())
    // latency correction: evaluate the trajectory at some offset
    double new_time = data.config.dt() + data.config.spg().latencyoffset(); // TODO why not set new_time as sample_rate?
    std::array<double, numberOfDOFs> new_position, new_velocity, new_acceleration, new_jerk;
    size_t new_section;
    trajectory.at_time(new_time, new_position, new_velocity, new_acceleration, new_jerk, new_section);

    // convert outputs
    resultPosition.x = new_position[0];
    resultPosition.y = new_position[1];
    resultPosition.rz = new_position[2];

    resultVelocity.x = -new_velocity[0];
    resultVelocity.y = -new_velocity[1];
    resultVelocity.rz = -new_velocity[2];

    // Diag data
    /*TODO
    data.spgCurrentPosition.x  = input.current_position[0];
    data.spgCurrentPosition.y  = input.current_position[1];
    data.spgCurrentPosition.Rz = input.current_position[2];

    data.spgCurrentVelocity.x  = input.current_velocity[0];
    data.spgCurrentVelocity.y  = input.current_velocity[1];
    data.spgCurrentVelocity.Rz = input.current_velocity[2];

    data.spgMaxAcceleration.x  = input.max_acceleration[0];
    data.spgMaxAcceleration.y  = input.max_acceleration[1];
    data.spgMaxAcceleration.Rz = input.max_acceleration[2];

    data.spgTargetVelocity.x  = input.target_velocity[0];
    data.spgTargetVelocity.y  = input.target_velocity[1];
    data.spgTargetVelocity.Rz = input.target_velocity[2];

    data.spgNewPosition.x  = new_position[0];
    data.spgNewPosition.y  = new_position[1];
    data.spgNewPosition.Rz = new_position[2];

    data.spgNewVelocity.x  = new_velocity[0];
    data.spgNewVelocity.y  = new_velocity[1];
    data.spgNewVelocity.Rz = new_velocity[2];
    */

    return true;
}
