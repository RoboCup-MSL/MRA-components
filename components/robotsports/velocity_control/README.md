# Summary

VelocityControl is responsible for smooth controlled movement of the robot.


# Scope and context

This component is responsible for applying vel/acc/jerk limits and not losing ball while driving. It uses [Ruckig](../../../libraries/ruckig/ruckig/README.md) trajectory setpoint generator.

This component is NOT responsible for obstacle avoidance. That is done by PathPlanning just before VelocityControl, PathPlanning converts a desired ("unsafe") target setpoint (FCS) to a "safe" intermediate target setpoint (FCS), avoiding obstacles.

The output robot velocity setpoint (in RCS) is given to VelocityTransform, to be transformed to motor setpoints.

# Interface details

See [Input.proto](interface/Input.proto) and [Output.proto](interface/Output.proto).

There are quite some configuration parameters, see [Params.proto](interface/Params.proto) and [DefaultParams.json](interface/DefaultParams.json).

Currently only velocity-only (VEL_ONLY) or position-only (POS_ONLY) requests are supported. Request with position and velocity at same time (POSVEL) is a possible future feature.

If robot has the  ball AND the default motion profile (0) is set, then motion profile with name `withBall` will be used if it exists.

# Design

The Velocity Control is more than a wrapper around the Ruckig library. It is sequence of configurable sub-algorithms. It keep track of the administration of the velocity control.

* **CheckPrepareInputs**: perform check in the provide inputs:
    * Check that input contains valid data (no z,rx,ry) and robot position and velocity are set
    * Determine control mode: POSVEL, POS_ONLY or VEL_ONLY
    * Set internal variables based on inputs, state, params
    * Check if robot active (TODO: reset state if inactive))


* **ConfigureLimits**: determine the limits to use based on configuration and input motion profile
    * check that at least one set of limits is configured
    * on input, user can select which motion profile to use, default 0

    * when robot has the ball, and if input motion profile is default (0), then automatically choose a set of motion profiles called 'withBall' from the configured non-default motion profiles
    * fill limits based on default motion profile then, use requested motionprofile to overrule doing this after filling based on the default set (index 0) helps to keep the configuration clean

* **CheckStop**: immediately stop when only velocity is requested with x,y and Rz 0 (check on < 1e-4). This will resutl in hard motor brake instead of rampdown via setpoint generator. The output becomes a velocity of zero.

* **Deadzone**: This is an optional sub-algorithm (enable/disable via parameters) when is only applied for a position-based setpoint. It the reached position is close (configurable tolerance) to the requested setpoint, it will stop attempting to reach to target sepoint; it is prevent wasting energy by responding to very small setpoints. It also will skip the execution of setpoint generator

* **ShiftBallOffset**: if robot has ball (dribbling), its limits should apply to ball and not robot, this step applies an offset to the request to apply correct offset


* **SelectVelocityController**: Currently only calculation the velocity via a a SPGVelocityController is supported.
* **CalculateVelocity**: velocity control (in FCS or RCS, depends on the inputs) It uses [Ruckig](../../../libraries/ruckig/ruckig/README.md) trajectory setpoint generator which limits velocity and acceleration and jerk.
![Trajectory Profile](https://github.com/pantor/ruckig/raw/main/doc/example_profile.png?raw=true)
    * The internal administration is updated to calculate a new setpoint. For a position setpoint the current position into account. Also take into account if state is filled or it is the first calculation (to avoid jumps when not starting from the FCS origin)

    * In order to achieve higher deceleration, a layer is added around the SPG algorithm, first calculate with (aggressive) deceleration limits, so the robot is normally going to be in time for braking, this gives a good estimate of velocity and acceleration setpoint.  If the calculated sepoint indicated that the robot _seems_ to be accelerating, a new sepoint is calculated with the use of corresponding limits.

    * For a position based setpoint (POS-ONLY or POS-VEL) synchronization of rotation is optional. If rotation is synchronized all exist should finish at same time, if not synchronized then the linear axes (X and Y) synchronized, the  rotation is not.
    * For velocity based setpoint all axes are synchronized.

* **UnShiftBallOffset**: if robot has ball (dribbling), its limits should apply to ball and not robot, this step unshifts (revert) the applied offset to store the XXX in the state.

* **SetOutputsPrepareNext**: set output data, write state variables to be used in next iteration. This step can not be skipped via configuration.


# History

Based on the Falcons Velocity Control component: using Ruckig instead of Reflexxes Library, later extended with fixes for velocity only control and ignoring the state when it is used for the first time.
