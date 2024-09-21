// CODEGEN_NOTE
// it should NOT be modified by user

#ifndef _MRA_COMPONENT_CPP_NAME_UNDERSCORE_HPP
#define _MRA_COMPONENT_CPP_NAME_UNDERSCORE_HPP


// component name definition goes on top
// (when logging.hpp is used internally in a component, then component name may resolve to "unknown")
#ifndef MRA_COMPONENT_NAME
#define MRA_COMPONENT_NAME "COMPONENT_CPP_NAME_CAMELCASE"
#endif
#ifndef MRA_COMPONENT_FOLDER
#define MRA_COMPONENT_FOLDER "COMPONENT_REL_PATH"
#endif


#include "abstract_interface.hpp"
#include "params_loader.hpp"
#include <google/protobuf/empty.pb.h>
#include "logging.hpp"

// generated protobuf types from interface of this component
#include "COMPONENT_CPP_NAME_CAMELCASE_datatypes.hpp"




namespace MRA::COMPONENT_CPP_NAME_CAMELCASE
{

PROTOBUF_HPP_TYPE_TYPEDEFS

class COMPONENT_CPP_NAME_CAMELCASE: public MRAInterface<InputType, ParamsType, StateType, OutputType, DiagnosticsType>
{
public:
    COMPONENT_CPP_NAME_CAMELCASE() {};
    ~COMPONENT_CPP_NAME_CAMELCASE() {};

    // user implementation
    int tick(
        google::protobuf::Timestamp timestamp,   // absolute timestamp
        InputType  const           &input,       // input data, type generated from Input.proto
        ParamsType const           &params,      // configuration parameters, type generated from Params.proto
        StateType                  &state,       // state data, type generated from State.proto
        OutputType                 &output,      // output data, type generated from Output.proto
        DiagnosticsType            &diagnostics  // diagnostics data, type generated from Diagnostics.proto
    );

    // make default configuration easily accessible
    ParamsType defaultParams() const
    {
        return MRA::LoadDefaultParams<ParamsType>("components/COMPONENT_REL_PATH/interface/DefaultParams.json");
    };

    // allow omitting arguments, useful for testing and code brevity
    int tick()
    {
        StateType s;
        OutputType o;
        DiagnosticsType d;
        return tick(google::protobuf::util::TimeUtil::GetCurrentTime(), InputType(), defaultParams(), s, o, d);
    };

    int tick(
        InputType  const &input,
        OutputType       &output
    )
    {
        StateType s;
        DiagnosticsType d;
        return tick(google::protobuf::util::TimeUtil::GetCurrentTime(), input, defaultParams(), s, output, d);
    };

    int tick(
        InputType  const &input,
        ParamsType const &params,
        OutputType       &output
    )
    {
        StateType s;
        DiagnosticsType d;
        return tick(google::protobuf::util::TimeUtil::GetCurrentTime(), input, params, s, output, d);
    };

    int tick(
        InputType  const &input,
        ParamsType const &params,
        StateType        &state,
        OutputType       &output,
        DiagnosticsType  &diagnostics
    )
    {
        return tick(google::protobuf::util::TimeUtil::GetCurrentTime(), input, params, state, output, diagnostics);
    };

}; // class COMPONENT_CPP_NAME_CAMELCASE


// configuration handling
inline ParamsType defaultParams()
{
    return COMPONENT_CPP_NAME_CAMELCASE().defaultParams();
}
inline ParamsType loadParams(std::string configFile)
{
    return MRA::LoadDefaultParams<ParamsType>(configFile);
}

} // namespace MRA::COMPONENT_CPP_NAME_CAMELCASE


#endif

