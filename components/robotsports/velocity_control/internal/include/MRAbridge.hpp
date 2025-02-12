#ifndef MRA_COMPONENTS_ROBOTSPORTS_VELOCITYCONTROL_MRA_BRIDGE_HPP_
#define MRA_COMPONENTS_ROBOTSPORTS_VELOCITYCONTROL_MRA_BRIDGE_HPP_

// map RobotSports Position2D, Velocity2D pose to MRA classes and operations (at MRA-libraries)
#include "geometry.hpp"
typedef MRA::Geometry::Position Position2D;
typedef MRA::Geometry::Velocity Velocity2D;
typedef MRA::Geometry::Pose Pose2D;

// VelocityControlData is now basically a collection of protobuf datatypes
#include "components/robotsports/velocity_control/interface/Input.pb.h"
#include "components/robotsports/velocity_control/interface/Params.pb.h"
#include "components/robotsports/velocity_control/interface/Output.pb.h"
#include "components/robotsports/velocity_control/interface/State.pb.h"
#include "components/robotsports/velocity_control/interface/Diagnostics.pb.h"
typedef google::protobuf::Timestamp MRA_timestamp;
typedef MRA::RobotsportsVelocityControl::Input  MRA_InputType;
typedef MRA::RobotsportsVelocityControl::Params MRA_ParamsType;
typedef MRA::RobotsportsVelocityControl::State  MRA_StateType;
typedef MRA::RobotsportsVelocityControl::Output MRA_OutputType;
typedef MRA::RobotsportsVelocityControl::Diagnostics  MRA_DiagnosticsType;

// map configuration types
typedef MRA::RobotsportsVelocityControl::SpgConfig SpgConfig;

#endif

