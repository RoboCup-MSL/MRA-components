# Summary

ActionPlanning provides a set of actions to Teamplay/ActionChoice/Strategy component to choose from.

It typically breaks down the action into basic movement / kicker / ballhandler setpoints for further processing by robot peripherals software.
It returns a functional status (PASSED/FAILED/RUNNING) at each tick.


# Scope and context

Actions:
* STOP: just stop robot movement
* MOVE: move to a coordinate in FCS
* DASH: local directional XY movement
* KICK: kick the ball without rotation
* PASS: pass to target (typically teammate), includes aiming/local movement
* SHOOT: shoot at target (typically goal), includes aiming/local movement, maximize ball velocity
* LOB: similar as shoot, but maximize ball trajectory height
* GETBALL: get the ball (may intercept, may sprint/swerve)
* SHIELD: hold the ball, rotate away from opponent
* KEEPER: act as keeper
* INTERCEPT: ??? rename to "CATCH"? why not functionally embed in getball?
* PARK: move to parking position

Input action choice should be somewhat stable.

Resulting movement target should be processed by PathPlanning/ObstacleAvoidance algorithm.

# Interface details

See [Input.proto](interface/Input.proto) and [Output.proto](interface/Output.proto).

The configuration [Params.proto](interface/Params.proto) is specialized for each action.

# Design notes

Use MRA subcomponents.

Run at a frequency of say 30Hz. Optional state per action.

Action .bin files get written at end of each action, with all samples during action and result, for post-mortem analysis.

TODO: python tooling to plot action .bin.

