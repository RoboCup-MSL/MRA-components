/*
 * AbstractVelocitySetpointController.hpp
 * Started as copy of file with same name in the Falcons variant of Velocity Control
 */

#ifndef ROBOTSPORTS_VELOCITYCONTROL_ABSTRACTVELOCITYSETPOINTCONTROLLER_HPP_
#define ROBOTSPORTS_VELOCITYCONTROL_ABSTRACTVELOCITYSETPOINTCONTROLLER_HPP_


#include "VelocityControlData.hpp"


// abstract base class for specific controllers, like PID, TokyoDrift, Linear, ...

class AbstractVelocitySetpointController
{
public:
    AbstractVelocitySetpointController() {};
    virtual ~AbstractVelocitySetpointController() {};

    // modify resulting data.resultVelocityRcs
    // return success
    virtual bool calculate(VelocityControlData &data) = 0;

};

#endif

