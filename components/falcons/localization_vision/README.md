# Summary

LocalizationVision is responsible for determining location candidate(s) in FCS based on observed landmarks (typically white pixels) in RCS.

It uses the Simplex Downhill solver from `opencv`.

# Status

Basics are mostly there, interface is finished, implementation is not yet complete. Remaining work:
* port more tricks from Andre's code
* integrate on robot, compare new against existing, prove it's working
* squeeze milliseconds

# Scope and context

This component is responsible for fitting the observed white line pixels to the expected configured line floor layout.

Its input should be in RCS, typically detected white line pixels, filtered, already transformed from camera pixels coordinates.

Its output is intended to be passed to LocalizationWorldModel for fusion with other sensor data.

# Interface details

See [Input.proto](interface/Input.proto) and [Output.proto](interface/Output.proto).

The configuration is two-fold:
1. the [official MSL field specification](https://msl.robocup.org/wp-content/uploads/2023/01/Rulebook_MSL2023_v24.1.pdf#section.1.1) (`A=22`, `B=14` etc) is used to generate a set of shapes
2. optional extra shapes, for instance to write out the field from the Ambition Challenge

Note that teams tend to borrow each others field measurements using the definitions in [1] when calibrating for a new venue. It hardly makes sense for all of us to re-measure the same field ;).

# Design

Simplex method.

Includes a little python tool to plot field (serialized `CvMatProto`): `plot.py`.

# Demo

## plot.py

After having run the test suite:

`bazel run //components/falcons/localization_vision/test:plot /tmp/testsuite_mra_logging/tickbins/tick_FalconsLocalizationVision_4.bin`

![plottingtool](test/demo2.png)

## tune.py

Run tuning tool (sliders are automatically derived from `Params.proto`):

`bazel run //components/falcons/localization_vision/test:tune /tmp/testsuite_mra_logging/tickbins/tick_FalconsLocalizationVision_0.bin`

![tuningtool](test/demo3.png)

