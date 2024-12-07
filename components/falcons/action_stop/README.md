# Summary

`ActioStop` is one of the actions available to Teamplay/Strategy/ActionChoice. (Naming tbd.)

It is a trivial action mainly to stop the robot movement.
Optionally ballhandlers can remain active.

# Status

Done, stable. Not yet used in Falcons code.

# Scope and context

This is a nested component, controlled by `ActionPlanning`.

# Interface details

See [Input.proto](interface/Input.proto) and [Output.proto](interface/Output.proto).

No params / state / diagnostics.

# Design

As all actions: set motion- and ballhandler commands for `ActionPlanning` to delegate.

The action will always return status `PASSED`.
