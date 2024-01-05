// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "RobotsportsLocalObstacleTracking.hpp"

using namespace MRA;

// custom includes, if any
// ...
#include "obstacle_tracking.hpp"

int RobotsportsLocalObstacleTracking::RobotsportsLocalObstacleTracking::tick
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

    // user implementation goes here
    // user implementation goes here
    double timestamp_as_double = google::protobuf::util::TimeUtil::TimestampToMilliseconds(timestamp) / 1000.0;
    obstacle_tracking(timestamp_as_double, input, params, state,output);


    return error_value;
}

