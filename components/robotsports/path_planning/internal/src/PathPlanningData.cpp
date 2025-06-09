/*
 * PathPlanningData.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#include "logging.hpp"
#include "position2d.hpp"
#include "PathPlanning.hpp"

void PathPlanningData::reset()
{
    // clear calculation results and internal data; inputs are overridden at start of iteration
    // make sure all diagnostics related items are cleared here, otherwise you might end up diagnosing data from an older iteration
    path.clear();
    calculatedObstacles.clear();
    calculatedForbiddenAreas.clear();
    resultStatus = MRA::Datatypes::ActionResult::INVALID;
    deltaPositionFcs = Position2D(0.0, 0.0, 0.0);
    deltaPositionRcs = Position2D(0.0, 0.0, 0.0);
    done = false;
    motionType = motionTypeEnum::INVALID;
    stop = false;
}

void PathPlanningData::traceInputs()
{
    MRA_LOG_DEBUG("robotPos=[%6.2f, %6.2f, %6.2f] robotVel=[%6.2f, %6.2f, %6.2f]", robot.position.x, robot.position.y, robot.position.Rz, robot.velocity.x, robot.velocity.y, robot.velocity.Rz);
    MRA_LOG_DEBUG("stop=%d targetPos=[%6.2f, %6.2f, %6.2f] motionType=%s", stop, target.pos.x, target.pos.y, target.pos.Rz, enum2str(motionType));
}

void PathPlanningData::traceOutputs()
{
    auto subtarget = path.begin();
    if (subtarget != path.end())
    {
        MRA_LOG_DEBUG("result=%-8s  ROBOT_POSVEL_SETPOINT=( pos=[%6.2f, %6.2f, %6.2f], vel=[%6.2f, %6.2f, %6.2f], motionType=%s )", enum2str(resultStatus), subtarget->pos.x, subtarget->pos.y, subtarget->pos.Rz, subtarget->vel.x, subtarget->vel.y, subtarget->vel.Rz, enum2str(motionType));
    }
    else
    {
        std::string err = "No subtarget defined in PathPlanning. `path` in PathPlanningData must always result in a wayPoint.";
        MRA_LOG_DEBUG(err.c_str());
        throw std::runtime_error(err.c_str());
    }
    
}

void PathPlanningData::insertSubTarget(Position2D const &pos, Velocity2D const &vel)
{
    // only insert if subtarget is sufficiently far away
    Position2D deltaPositionFcsLocal = pos - currentPositionFcs;
    deltaPositionFcsLocal.phi = project_angle_mpi_pi(deltaPositionFcsLocal.phi);
    bool xyFar = deltaPositionFcsLocal.xy().size() >= configPP.deadzone.toleranceXY;
    bool RzFar = fabs(deltaPositionFcsLocal.phi) >= configPP.deadzone.toleranceRz;
    MRA_LOG_DEBUG("xyFar=%d RzFar=%d", xyFar, RzFar);
    if (xyFar || RzFar)
    {
        wayPoint wp;
        wp.pos.x = pos.x;
        wp.pos.y = pos.y;
        wp.pos.Rz = pos.phi;
        wp.vel.x = vel.x;
        wp.vel.y = vel.y;
        wp.vel.Rz = vel.phi;
        path.insert(path.begin(), wp);
        // Position2D p = pos;
        // Velocity2D v = vel;
        MRA_LOG_DEBUG("adding subtarget %s %s, path size is now %d", pos.tostr(), vel.tostr(), (int)path.size());
    }
    else
    {
        MRA_LOG_DEBUG("ignoring new subtarget because it is too close by");
    }
}

void PathPlanningData::addForbiddenAreas(std::vector<forbiddenArea> const &newForbiddenAreas)
{
    MRA_LOG_DEBUG("n=%d", (int)newForbiddenAreas.size());
    // give each one a unique id, for diagnostics, and to prevent visualizer confusion
    for (int it = 0; it < (int)newForbiddenAreas.size(); ++it)
    {
        addForbiddenArea(newForbiddenAreas.at(it));
    }
}

void PathPlanningData::addForbiddenArea(forbiddenArea const &newForbiddenArea)
{
    // give each one a unique id, for diagnostics, and to prevent visualizer confusion
    auto f = newForbiddenArea;
    f.id = (int)calculatedForbiddenAreas.size();
    MRA_LOG_DEBUG("id=%d %6.2f", f.id, f.points[0].x);
    calculatedForbiddenAreas.push_back(f);
}

Position2D PathPlanningData::getSubTarget() const
{
    Position2D result;
    if (path.size())
    {
        auto subtarget = path.at(0);
        result.x = subtarget.pos.x;
        result.y = subtarget.pos.y;
        result.phi = subtarget.pos.Rz;
    }
    else
    {
        result.x = robot.position.x;
        result.y = robot.position.y;
        result.phi = robot.position.Rz;
    }
    return result;
}
