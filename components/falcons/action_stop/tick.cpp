// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "FalconsActionStop.hpp"

using namespace MRA;

// custom includes, if any
// ...


int FalconsActionStop::FalconsActionStop::tick
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

    output.set_actionresult(MRA::Datatypes::PASSED);
    output.set_stopmoving(true);
    output.set_ballhandlersenabled(input.ballhandlersenabled());

    return error_value;
}

