// CODEGEN_NOTE
// with the intent of letting user add the implementation here

// generated component header:
#include "COMPONENT_CPP_NAME_CAMELCASE.hpp"
DEPENDENT_HEADERS
using namespace MRA;

// custom includes, if any
// ...


int COMPONENT_CPP_NAME_CAMELCASE::COMPONENT_CPP_NAME_CAMELCASE::tick
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

    // user implementation goes here



    return error_value;
}

