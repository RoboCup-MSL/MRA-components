/*
 * PathPlanning.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */
#include <boost/bind.hpp>

// own package
#include "PathPlanning.hpp"
#include "PathPlanningAlgorithms.hpp"

#include "logging.hpp"


PathPlanning::PathPlanning()
{
}


PathPlanning::~PathPlanning()
{
}

void PathPlanning::calculate(double ts, 
                    const path_planner_input_t& r_input, 
                    const path_planner_parameters_t& r_params,
                    path_planner_state_t& r_state,
                    path_planner_output_t& r_output,
                    path_planner_diagnostics_t& r_diagnostics)
{
    data.reset();
    data.stop = r_state.stop;
    data.parameters = r_params;

    // timestepping
    if (data.parameters.frequency > 0)
    {
        data.dt = 1.0 / data.parameters.frequency;
    }
    else
    {
        data.dt = 1.0 / 20.0;
    }
    MRA_LOG_DEBUG("nominalFrequency=%.1f dt=%.4fs", data.parameters.frequency, data.dt);

    // get inputs from input interface
    // get and store data
    motionSetpoint_t sp = r_input.motionSetpoint;
    data.target.pos = MRA::Geometry::Pose();
    data.motionType = sp.motionType; 
    if (sp.move_action) // for any other action: do nothing
    {
        data.target.pos.x = sp.position.x;
        data.target.pos.y = sp.position.y;
        data.target.pos.rz = sp.position.rz;
        data.stop = false;
    }
    data.target.vel = MRA::Geometry::Pose(); // nonzero input velocity is not yet supported on external interface
    data.forbiddenAreas = r_input.forbiddenAreas;
    data.addForbiddenAreas(data.forbiddenAreas); // add to calculatedForbiddenAreas
    data.robot = r_input.myRobotState;
    data.teamMembers = r_input.teamRobotState;
    data.obstacles = r_input.obstacles;
    data.ball = r_input.ball;

    // calculate
    data.timestamp = 0.0; // TODO ftime::now();

    data.traceInputs();

    // configure the sequence of algorithms
    std::vector<PathPlanningAlgorithm *> algorithms; // previously known as 'blocks'

    // basics
    algorithms.push_back(new RequireWorldModelActive());
    algorithms.push_back(new CheckStopCommand());
    algorithms.push_back(new CheckTargetValid());
    algorithms.push_back(new CheckTargetReached());

    // subtarget calculation
    algorithms.push_back(new EscapeForbiddenAreas());
    algorithms.push_back(new CalculateObstacles()); // opponents + teammembers + forbiddenAreas + projectedSpeedVectors
    algorithms.push_back(new AvoidObstacles());
    algorithms.push_back(new Shielding());
    algorithms.push_back(new ForwardDriving());

    // initially, the path consists of just the given target
    data.path.push_back(data.target);

    // execute the sequence of algorithms
    for (auto it = algorithms.begin(); it != algorithms.end(); ++it)
    {
        if (!data.done)
        {
            (*it)->execute(data);
        }
    }

    data.traceOutputs();
    // Output of PathPlanning is the first wayPoint / subTarget
    r_output.status = data.resultStatus;
    if (data.stop)
    {
        r_output.velocitySetpointValid = true;
        r_output.positionSetpointValid = false;
        r_output.robotVelocitySetpoint = MRA::Geometry::Pose(0.0, 0.0, 0.0);
        r_output.motionType = motionTypeEnum::NORMAL;
    }
    else
    {
        r_output.velocitySetpointValid = true;
        r_output.robotPositionSetpoint = data.path.front().pos;
        r_output.velocitySetpointValid = true;
        r_output.robotVelocitySetpoint = data.path.front().vel;
        r_output.motionType = data.motionType;

        if (data.robot.hasBall)
        {
            r_output.motionType = motionTypeEnum::WITH_BALL;
        }
    }

    // fill state
    r_state.stop = data.stop;

    // fill diagnostics
    r_diagnostics.path = data.path;
    r_diagnostics.forbiddenAreas = data.calculatedForbiddenAreas;
    r_diagnostics.distanceToSubTargetRCS.x = data.deltaPositionRcs.x;
    r_diagnostics.distanceToSubTargetRCS.y = data.deltaPositionRcs.y;
    r_diagnostics.distanceToSubTargetRCS.rz = data.deltaPositionRcs.rz;
    r_diagnostics.numCalculatedObstacles = data.calculatedObstacles.size();
    r_diagnostics.stop = data.stop;

    MRA_LOG_DEBUG("r_output.status=%d", r_output.status);
}

