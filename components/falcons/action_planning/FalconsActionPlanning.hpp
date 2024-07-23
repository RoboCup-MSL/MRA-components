// this file was produced by MRA-codegen.py from template_instance.hpp
// it should NOT be modified by user

#ifndef _MRA_FALCONS_ACTION_PLANNING_HPP
#define _MRA_FALCONS_ACTION_PLANNING_HPP


// component name definition goes on top
// (when logging.hpp is used internally in a component, then component name may resolve to "unknown")
#ifndef MRA_COMPONENT_NAME
#define MRA_COMPONENT_NAME "FalconsActionPlanning"
#endif
#ifndef MRA_COMPONENT_FOLDER
#define MRA_COMPONENT_FOLDER "falcons/action_planning"
#endif


#include "abstract_interface.hpp"
#include "params_loader.hpp"
#include <google/protobuf/empty.pb.h>
#include "logging.hpp"

// generated protobuf types from interface of this component
#include "FalconsActionPlanning_datatypes.hpp"




namespace MRA::FalconsActionPlanning
{

typedef MRA::FalconsActionPlanning::Input InputType;
typedef MRA::FalconsActionPlanning::Params ParamsType;
typedef MRA::FalconsActionPlanning::State StateType;
typedef MRA::FalconsActionPlanning::Output OutputType;
typedef MRA::FalconsActionPlanning::Local LocalType;


class FalconsActionPlanning: public MRAInterface<InputType, ParamsType, StateType, OutputType, LocalType>
{
public:
    FalconsActionPlanning() {};
    ~FalconsActionPlanning() {};

    // user implementation
    int tick(
        google::protobuf::Timestamp timestamp,   // absolute timestamp
        InputType  const           &input,       // input data, type generated from Input.proto
        ParamsType const           &params,      // configuration parameters, type generated from Params.proto
        StateType                  &state,       // state data, type generated from State.proto
        OutputType                 &output,      // output data, type generated from Output.proto
        LocalType                  &local        // local/diagnostics data, type generated from Local.proto
    );

    // make default configuration easily accessible
    ParamsType defaultParams() const
    {
        return MRA::LoadDefaultParams<ParamsType>("components/falcons/action_planning/interface/DefaultParams.json");
    };

    // allow omitting arguments, useful for testing and code brevity
    int tick()
    {
        StateType s;
        OutputType o;
        LocalType l;
        return tick(google::protobuf::util::TimeUtil::GetCurrentTime(), InputType(), defaultParams(), s, o, l);
    };

    int tick(
        InputType  const &input,
        OutputType       &output
    )
    {
        StateType s;
        LocalType l;
        return tick(google::protobuf::util::TimeUtil::GetCurrentTime(), input, defaultParams(), s, output, l);
    };

    int tick(
        InputType  const &input,
        ParamsType const &params,
        OutputType       &output
    )
    {
        StateType s;
        LocalType l;
        return tick(google::protobuf::util::TimeUtil::GetCurrentTime(), input, params, s, output, l);
    };

    int tick(
        InputType  const &input,
        ParamsType const &params,
        StateType        &state,
        OutputType       &output,
        LocalType        &local
    )
    {
        return tick(google::protobuf::util::TimeUtil::GetCurrentTime(), input, params, state, output, local);
    };

}; // class FalconsActionPlanning


// configuration handling
inline ParamsType defaultParams()
{
    return FalconsActionPlanning().defaultParams();
}
inline ParamsType loadParams(std::string configFile)
{
    return MRA::LoadDefaultParams<ParamsType>(configFile);
}

} // namespace MRA::FalconsActionPlanning


#endif

