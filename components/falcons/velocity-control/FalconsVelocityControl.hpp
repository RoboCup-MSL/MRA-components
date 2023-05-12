// this file was produced by MRA-codegen.py from template_instance.hpp
// it should NOT be modified by user

#ifndef _MRA_FALCONS_VELOCITY_CONTROL_HPP
#define _MRA_FALCONS_VELOCITY_CONTROL_HPP

#include "abstract_interface.hpp"
#include "params_loader.hpp"

#include "components/falcons/velocity-control/interface/Input.pb.h"
#include "components/falcons/velocity-control/interface/Params.pb.h"
#include "components/falcons/velocity-control/interface/State.pb.h"
#include "components/falcons/velocity-control/interface/Output.pb.h"
#include "components/falcons/velocity-control/interface/Local.pb.h"


namespace MRA::FalconsVelocityControl
{

typedef MRA::FalconsVelocityControl::Input InputType;
typedef MRA::FalconsVelocityControl::Params ParamsType;
typedef MRA::FalconsVelocityControl::State StateType;
typedef MRA::FalconsVelocityControl::Output OutputType;
typedef MRA::FalconsVelocityControl::Local LocalType;


class FalconsVelocityControl: public MRAInterface<InputType, ParamsType, StateType, OutputType, LocalType>
{
public:
    FalconsVelocityControl() {};
    ~FalconsVelocityControl() {};

    // user implementation
    int tick(
        double            timestamp,   // simulation timestamp, seconds since start of simulation
        InputType  const &input,       // input data, type generated from Input.proto
        ParamsType const &params,      // configuration parameters, type generated from Params.proto
        StateType        &state,       // state data, type generated from State.proto
        OutputType       &output,      // output data, type generated from Output.proto
        LocalType        &local        // local/diagnostics data, type generated from Local.proto
    );

    // make default configuration easily accessible
    ParamsType defaultParams() const
    {
        return MRA::LoadDefaultParams<ParamsType>("components/falcons/velocity-control/interface/DefaultParams.json");
    };

    // allow omitting arguments, useful for testing and code brevity
    int tick()
    {
        StateType s;
        OutputType o;
        LocalType l;
        return tick(0.0, InputType(), defaultParams(), s, o, l);
    };

    int tick(
        InputType  const &input,
        OutputType       &output
    )
    {
        StateType s;
        LocalType l;
        return tick(0.0, input, defaultParams(), s, output, l);
    };

    int tick(
        InputType  const &input,
        ParamsType const &params,
        OutputType       &output
    )
    {
        StateType s;
        LocalType l;
        return tick(0.0, input, params, s, output, l);
    };

}; // class FalconsVelocityControl

} // namespace MRA::FalconsVelocityControl

#endif

