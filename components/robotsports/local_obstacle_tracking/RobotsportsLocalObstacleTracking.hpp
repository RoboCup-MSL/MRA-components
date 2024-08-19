// this file was produced by MRA-codegen.py from template_instance.hpp
// it should NOT be modified by user

#ifndef _MRA_ROBOTSPORTS_LOCAL_OBSTACLE_TRACKING_HPP
#define _MRA_ROBOTSPORTS_LOCAL_OBSTACLE_TRACKING_HPP


// component name definition goes on top
// (when logging.hpp is used internally in a component, then component name may resolve to "unknown")
#ifndef MRA_COMPONENT_NAME
#define MRA_COMPONENT_NAME "RobotsportsLocalObstacleTracking"
#endif
#ifndef MRA_COMPONENT_FOLDER
#define MRA_COMPONENT_FOLDER "robotsports/local_obstacle_tracking"
#endif


#include "abstract_interface.hpp"
#include "params_loader.hpp"
#include <google/protobuf/empty.pb.h>
#include "logging.hpp"

// generated protobuf types from interface of this component
#include "RobotsportsLocalObstacleTracking_datatypes.hpp"




namespace MRA::RobotsportsLocalObstacleTracking
{

typedef MRA::RobotsportsLocalObstacleTracking::Input InputType;
typedef MRA::RobotsportsLocalObstacleTracking::Params ParamsType;
typedef MRA::RobotsportsLocalObstacleTracking::State StateType;
typedef MRA::RobotsportsLocalObstacleTracking::Output OutputType;
typedef MRA::RobotsportsLocalObstacleTracking::Diagnostics DiagnosticsType;


class RobotsportsLocalObstacleTracking: public MRAInterface<InputType, ParamsType, StateType, OutputType, DiagnosticsType>
{
public:
    RobotsportsLocalObstacleTracking() {};
    ~RobotsportsLocalObstacleTracking() {};

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
        return MRA::LoadDefaultParams<ParamsType>("components/robotsports/local_obstacle_tracking/interface/DefaultParams.json");
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

}; // class RobotsportsLocalObstacleTracking


// configuration handling
inline ParamsType defaultParams()
{
    return RobotsportsLocalObstacleTracking().defaultParams();
}
inline ParamsType loadParams(std::string configFile)
{
    return MRA::LoadDefaultParams<ParamsType>(configFile);
}

} // namespace MRA::RobotsportsLocalObstacleTracking


#endif

