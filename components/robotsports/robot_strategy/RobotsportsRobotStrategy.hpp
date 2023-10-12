// this file was produced by MRA-codegen.py from template_instance.hpp
// it should NOT be modified by user

#ifndef _MRA_ROBOTSPORTS_ROBOT_STRATEGY_HPP
#define _MRA_ROBOTSPORTS_ROBOT_STRATEGY_HPP


// component name definition goes on top
// (when logging.hpp is used internally in a component, then component name may resolve to "unknown")
#ifndef MRA_COMPONENT_NAME
#define MRA_COMPONENT_NAME "RobotsportsRobotStrategy"
#endif


#include "abstract_interface.hpp"
#include "params_loader.hpp"
#include <google/protobuf/empty.pb.h>
#include "logging.hpp"

// generated protobuf types from interface of this component
#include "RobotsportsRobotStrategy_datatypes.hpp"




namespace MRA::RobotsportsRobotStrategy
{

typedef MRA::RobotsportsRobotStrategy::Input InputType;
typedef MRA::RobotsportsRobotStrategy::Params ParamsType;
typedef google::protobuf::Empty StateType; // no .proto -> unused
typedef MRA::RobotsportsRobotStrategy::Output OutputType;
typedef google::protobuf::Empty LocalType; // no .proto -> unused


class RobotsportsRobotStrategy: public MRAInterface<InputType, ParamsType, StateType, OutputType, LocalType>
{
public:
    RobotsportsRobotStrategy() {};
    ~RobotsportsRobotStrategy() {};

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
        return MRA::LoadDefaultParams<ParamsType>("components/robotsports/robot_strategy/interface/DefaultParams.json");
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

}; // class RobotsportsRobotStrategy


// configuration handling
inline ParamsType defaultParams()
{
    return RobotsportsRobotStrategy().defaultParams();
}
inline ParamsType loadParams(std::string configFile)
{
    return MRA::LoadDefaultParams<ParamsType>(configFile);
}

} // namespace MRA::RobotsportsRobotStrategy


#endif

