/*
 * VelocityControl.cpp
 * Started as copy of file with same name in the Falcons variant of Velocity Control
 */

// own package
#include "VelocityControl.hpp"
#include "VelocitySetpointControllers.hpp"
#include <typeinfo>
#include <string>
#include <typeinfo>

using namespace MRA::internal::RVC;


VelocityControl::VelocityControl()
{
    setup();
}

VelocityControl::~VelocityControl()
{
}

void VelocityControl::add_algorithm(std::shared_ptr<VelocityControlAlgorithm> alg, bool unskippable)
{
    algorithms.push_back(alg);
    alg->unskippable = unskippable;
}

void VelocityControl::setup()
{
    // the sequence of algorithms matters and is managed here, based on configuration
    algorithms.clear();
    bool unskippable = true;

    // check that input contains valid data (no z,rx,ry)
    // determine control mode: POSVEL, POS_ONLY or VEL_ONLY
    // set internal variables based on inputs, state, params
    add_algorithm(std::make_shared<CheckPrepareInputs>());

    // determine the limits to use based on configuration and input motion profile
    add_algorithm(std::make_shared<ConfigureLimits>());

    // immediately stop when the command is given (hard motor brake instead of SPG rampdown)
    add_algorithm(std::make_shared<CheckStop>());


    // prevent wasting energy by responding to very small setpoints
    add_algorithm(std::make_shared<Deadzone>());

    // to enable dribbling, limits should apply to ball, not robot
    add_algorithm(std::make_shared<ShiftBallOffset>());

    // velocity control (in FCS or RCS, depends on configuration)
    // for now: always SPG (SetPointGenerator) using Ruckig library
    add_algorithm(std::make_shared<SelectVelocityController>());
    add_algorithm(std::make_shared<CalculateVelocity>());

    // to enable dribbling, limits should apply to ball, not robot
    add_algorithm(std::make_shared<UnShiftBallOffset>());

    // finally, set output data, write state variables to be used in next iteration
    add_algorithm(std::make_shared<SetOutputsPrepareNext>(), unskippable);
}

void VelocityControl::iterate()
{
    // this function assumes all required inputs (Inputs, Params, State) are set under "data"
    // each algorithm performs some operation on the data

    // execute the sequence of algorithms
    data.done = false;
    data.num_algorithms_executed = 0;
    for (auto it = algorithms.begin(); it != algorithms.end(); ++it)
    {
        // std::cout << "Executioning: " << typeid(**it).name() << std::endl;
        if (!data.done || (*it)->unskippable)
        {
            (*it)->execute(data);
            data.num_algorithms_executed++;
        }
        // std::cout << "\tdata.done: " << std::boolalpha << data.done << std::endl;
    }
}
