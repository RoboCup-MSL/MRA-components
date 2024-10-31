# Summary

VelocityControl is responsible for smooth controlled movement of the robot.

It uses [Ruckig](../../../libraries/ruckig/ruckig/README.md) trajectory setpoint generator which limits velocity and acceleration and jerk.

![Trajectory Profile](https://github.com/pantor/ruckig/raw/main/doc/example_profile.png?raw=true)

# Scope and context

This component is responsible for applying vel/acc/jerk limits and not losing ball while driving. It uses [Ruckig](../../../libraries/ruckig/ruckig/README.md) trajectory setpoint generator.

This component is NOT responsible for obstacle avoidance. That is done by PathPlanning just before VelocityControl, PathPlanning converts a desired ("unsafe") target setpoint (FCS) to a "safe" intermediate target setpoint (FCS), avoiding obstacles.

The output robot velocity setpoint (in RCS) is given to VelocityTransform, to be transformed to motor setpoints.

# Interface details

See [Input.proto](interface/Input.proto) and [Output.proto](interface/Output.proto).

There are quite some configuration parameters, see [Params.proto](interface/Params.proto) and [DefaultParams.json](interface/DefaultParams.json).

At least one (default) motion profile needs to be configured. When another motion profile with name `withBall` is configured, then it will be used to overrule limits while dribbling.

# Design

A sequence of sub-algorithms is applied. They are somewhat configurable.

# History

Based on the Falcons Velocity Control component: using Ruckig instead of Reflexxes Library
