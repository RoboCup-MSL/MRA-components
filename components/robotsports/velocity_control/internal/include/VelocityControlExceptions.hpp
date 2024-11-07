/*
 * VelocityControlExceptions.hpp
 * Started as copy of file with same name in the Falcons variant of Velocity Control
 */

#ifndef VELOCITYCONTROL_EXCEPTIONS_HPP_
#define VELOCITYCONTROL_EXCEPTIONS_HPP_

#include "exception_base.hpp"


namespace VelocityControlExceptions
{

DECLARE_CUSTOM_EXCEPTION(RobotInactive);
DECLARE_CUSTOM_EXCEPTION(InvalidInput);
DECLARE_CUSTOM_EXCEPTION(IncompleteInput);
DECLARE_CUSTOM_EXCEPTION(IncompleteConfiguration);
DECLARE_CUSTOM_EXCEPTION(UnsupportedDimension);

}

#endif // #ifndef VELOCITYCONTROL_EXCEPTIONS_HPP_

