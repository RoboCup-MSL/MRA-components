/*
 * CalculateObstacles.cpp
 *
 *  Created on: November, 2019
 *      Author: Jan Feitsma
 */


#include "PathPlanningAlgorithms.hpp"
#include "logging.hpp"

forbiddenArea makeForbiddenAreaFromLine(MRA::Geometry::Point const &src, MRA::Geometry::Point const &dst)
{
    double WIDTH = 0.3;
    MRA::Geometry::Point p(dst - src);
    MRA::Geometry::Pose perpendicular(p.x, p.y);
    Rotate(perpendicular, M_PI * 0.5);
    Normalize(perpendicular, WIDTH);
    forbiddenArea result;
    result.id = 0;
    MRA::Geometry::Point point = src + perpendicular * 0.5;
    result.points.push_back(vec2d(point.x, point.y));
    point = src - perpendicular * 0.5;
    result.points.push_back(vec2d(point.x, point.y));
    point = dst - perpendicular * 0.5;
    result.points.push_back(vec2d(point.x, point.y));
    point = dst + perpendicular * 0.5;
    result.points.push_back(vec2d(point.x, point.y));
    return result;
}

std::vector<obstacleResult> makeObstaclesFromLine(MRA::Geometry::Point const &src, MRA::Geometry::Point const &dst, double step)
{
    std::vector<obstacleResult> result;
    obstacleResult obst;
    MRA::Geometry::Point t = dst;
    // insert final point
    obst.position.x = t.x;
    obst.position.y = t.y;
    result.push_back(obst);
    // iterate from base
    MRA::Geometry::Point p(dst - src);
    MRA::Geometry::Position d(p.x, p.y);
    int n = floor(d.size() / step);
    Normalize(d, step);
    for (int i = 0; i <= n; ++i)
    {
        t = src + d * i;
        obst.position.x = t.x;
        obst.position.y = t.y;
        result.push_back(obst);
    }
    return result;
}

std::vector<obstacleResult> makeObstaclesFromPolygon(polygon const &poly, double step)
{
    std::vector<obstacleResult> result;
    // place obstacles on each side, not inside the interior
    for (int it = 0; it < (int)poly.points.size(); ++it)
    {
        MRA::Geometry::Point src(poly.points.at(it).x, poly.points.at(it).y), dst;
        if (it + 1 == (int)poly.points.size())
        {
            // last segment, wrap around
            dst = MRA::Geometry::Point(poly.points.at(0).x, poly.points.at(0).y);
        }
        else
        {
            // any but last segment
            dst = MRA::Geometry::Point(poly.points.at(it+1).x, poly.points.at(it+1).y);
        }
        auto segment = makeObstaclesFromLine(src, dst, step);
        // pop the last of each segment to reduce dupes
        segment.pop_back();
        result.insert(result.end(), segment.begin(), segment.end());
    }
    return result;
}

void handleObstacle(obstacleResult const &obstacle, PathPlanningData &data)
{
    // commonly used
    auto config = data.configPP.obstacleAvoidance;
    MRA::Geometry::Point r(data.robot.position.x, data.robot.position.y);
    MRA::Geometry::Point b(99, 99); // far outside field == ignore
    if (data.balls.size())
    {
        b.x = data.balls.at(0).position.x;
        b.y = data.balls.at(0).position.y;
    }

    // add to data.calculatedObstacles
    data.calculatedObstacles.push_back(obstacle);

    // check if speed vector is large enough
    MRA::Geometry::Point p(obstacle.position.x, obstacle.position.y);
    MRA::Geometry::Pose v(obstacle.velocity.x, obstacle.velocity.y);
    if (v.size() < config.speedLowerThreshold)
    {
        // done
        MRA_LOG_DEBUG("obstacle moves too slow");
        return;
    }

    // prevent absurdly large speed vectors from scaring the robots
    if (v.size() > config.speedUpperThreshold)
    {
        MRA_LOG_DEBUG("obstacle moves too fast, v=(%6.2f, %6.2f)", v.x, v.y);
        Normalize(v, config.speedUpperThreshold);
        MRA_LOG_DEBUG("obstacle speed clipped, v=(%6.2f, %6.2f)", v.x, v.y);
    }

    // consider the line from p to p+v*s
    // where s is a scaling factor, which depends on a few things
    // ROADMAP: ignore points around p, since obstacle will be gone from there soon
    // ROADMAP: use a triangle as forbidden area, to account for possible aim change
    double distToObst = (r - p).size();
    double s = config.speedScalingFactor + config.distanceScalingFactor * distToObst;
    MRA::Geometry::Point d = v * s; // direction

    // add forbidden area for diagnostics (visualizer can draw opaque cyan areas)
    if (d.size() > 0.1) // otherwise it is undefined and hardly visible anyway
    {
        data.addForbiddenArea(makeForbiddenAreaFromLine(p, p + d));
    }

    // add obstacles inside that area, unless close to ball
    auto projectedObstacles = makeObstaclesFromLine(p, p + d, config.generatedObstacleSpacing);
    for (auto it = projectedObstacles.begin(); it != projectedObstacles.end(); ++it)
    {
        if ((b - MRA::Geometry::Point(it->position.x, it->position.y)).size() > config.ballClearance)
        {
            data.calculatedObstacles.push_back(*it);
        }
    }
}

void CalculateObstacles::execute(PathPlanningData &data)
{
    // initialize
    data.calculatedObstacles.clear();
    data.calculatedForbiddenAreas.clear();
    MRA_LOG_DEBUG("#forbiddenAreas: %d", (int)data.forbiddenAreas.size());
    data.calculatedForbiddenAreas = data.forbiddenAreas;
    MRA_LOG_DEBUG("#calculatedForbiddenAreas: %d", (int)data.calculatedForbiddenAreas.size());
    MRA_LOG_DEBUG("#calculatedObstacles: %d", (int)data.calculatedObstacles.size());
    auto config = data.configPP.obstacleAvoidance;
    MRA::Geometry::Point r(data.robot.position.x, data.robot.position.y);
    MRA::Geometry::Point b(99, 99); // far outside field == ignore
    if (data.balls.size())
    {
        b.x = data.balls.at(0).position.x;
        b.y = data.balls.at(0).position.y;
    }

    // static forbidden areas
    {
        MRA_LOG_DEBUG("forbiddenAreas", "");
        for (auto it = data.forbiddenAreas.begin(); it != data.forbiddenAreas.end(); ++it)
        {
            // ignore forbidden area if robot is inside, otherwise it cannot escape
            if (!it->isPointInside(vec2d(r.x, r.y)))
            {
                auto areaObstacles = makeObstaclesFromPolygon(*it, config.generatedObstacleSpacing);
                MRA_LOG_DEBUG("adding %d calculated obstacles from area %d", (int)areaObstacles.size(), it->id);
                data.calculatedObstacles.insert(data.calculatedObstacles.end(), areaObstacles.begin(), areaObstacles.end());
            }
            else
            {
                MRA_LOG_DEBUG("robot is inside forbidden area %d", it->id);
            }
        }
    }
    MRA_LOG_DEBUG("#calculatedForbiddenAreas: %d", (int)data.calculatedForbiddenAreas.size());
    MRA_LOG_DEBUG("#calculatedObstacles: %d", (int)data.calculatedObstacles.size());

    // add teammembers
    {
        MRA_LOG_DEBUG("teammembers", "");
        for (auto it = data.teamMembers.begin(); it != data.teamMembers.end(); ++it)
        {
            // convert from robotState
            obstacleResult obst;
            obst.position = vec2d(it->position.x, it->position.y);
            obst.velocity = vec2d(it->velocity.x, it->velocity.y);
            // handle the obstacle
            handleObstacle(obst, data);
        }
    }
    MRA_LOG_DEBUG("#calculatedForbiddenAreas: %d", (int)data.calculatedForbiddenAreas.size());
    MRA_LOG_DEBUG("#calculatedObstacles: %d", (int)data.calculatedObstacles.size());

    // handle opponents
    {
        MRA_LOG_DEBUG("opponents", "");
        for (auto it = data.obstacles.begin(); it != data.obstacles.end(); ++it)
        {
            // handle the obstacle
            obstacleResult obst = *it;
            // ignore its velocity vector in case this obstacle is close to the ball
            // to prevent our robots from being 'too afraid' of the opponent
            double distanceObst2Ball = (MRA::Geometry::Point(obst.position.x, obst.position.y) - b).size();
            if (distanceObst2Ball < config.ballClearance)
            {
                obst.velocity = vec2d(0.0, 0.0);
            }
            // also ignore its velocity (not position!) in case our robot has the ball
            // we are probably not driving too fast, so let them bump into us, then a free kick might be awarded
            if (data.robot.hasBall)
            {
                obst.velocity = vec2d(0.0, 0.0);
            }
            // handle the obstacle
            handleObstacle(obst, data);
        }
    }
    MRA_LOG_DEBUG("#calculatedForbiddenAreas: %d", (int)data.calculatedForbiddenAreas.size());
    MRA_LOG_DEBUG("#calculatedObstacles: %d", (int)data.calculatedObstacles.size());
}

