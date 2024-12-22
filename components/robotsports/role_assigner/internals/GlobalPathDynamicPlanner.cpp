/**
 *  @file
 *  @brief   Class for the dynamic robot planner
 *  @curator Jürge van Eijck
 */
#include "GlobalPathDynamicPlanner.hpp"
#include "Dynamics.hpp"
#include "Vertex.hpp"
#include "GlobalPathPlanner.hpp"
#include "RoleAssignerRobot.hpp"
#include "RoleAssignerData.hpp"

#include "logging.hpp"

#include <iostream>
#include <sstream>
#include <limits>


using namespace MRA;

/**
 * Plans a path assuming constant velocities and indefinite accelerations.
 *
 * @param start
 *            Start situation of the robot. Velocity is currently not taken into account.
 * @param maxSpeed
 *            Maximum speed possible for the robot
 * @param target
 *            Target
 * @param obstacles
 *            Obstacles that must be avoided
 * @param nrIterations
 *            Number of re-planning iterations
 * @return path
 */
std::vector<path_piece_t> GlobalPathDynamicPlanner::planPath(const MRA::Geometry::Position& start_pose, const MRA::Geometry::Position& start_vel,
                                                   const std::vector<RoleAssignerRobot>& filtered_teammates, const RoleAssignerData& r_role_assigner_data,
                                                   const std::vector<MRA::Vertex>& targetPos, planner_target_e targetFunction, bool ballIsObstacle,
                                                   double maxSpeed, int nrIterations,  bool stayInPlayingField)
{
    bool logDynamicPlanner = false;

    if (logDynamicPlanner) {
        MRA_LOG_INFO("Planning path with %d iterations", nrIterations);
    }
    // Currently we do not use the initial robot speed

    // Find initial intercept point
    Dynamics::dynamics_t intercept_data = Dynamics::interceptBall(r_role_assigner_data.ball,
                                                 start_pose, maxSpeed,
                                                 r_role_assigner_data.environment, r_role_assigner_data.parameters.move_to_ball_left_field_position);
    if (intercept_data.intercept_position.x == std::numeric_limits<double>::has_quiet_NaN) {
        // No intercept possible, return empty list
        if (logDynamicPlanner) {
            MRA_LOG_INFO("No intercept possible");
        }
        return std::vector<path_piece_t>();
    }
    if (logDynamicPlanner) {
        MRA_LOG_INFO("calculate interception point %s", intercept_data.intercept_position.toString().c_str());
    }

    GlobalPathPlanner visibilityGraph = GlobalPathPlanner(r_role_assigner_data.environment);
    visibilityGraph.setOptions(r_role_assigner_data.parameters);

    bool avoidBallPath = false; // Not need to avoid the ball. This function is only used for the interceptor
    Geometry::Point BallTargetPos;

    int iteration = 1;

    std::vector<MRA::Vertex> target_vect;
    target_vect.push_back(Vertex(intercept_data.intercept_position, 0));
    visibilityGraph.createGraph(start_pose, start_vel, r_role_assigner_data.ball, filtered_teammates, 
                                r_role_assigner_data.opponents, r_role_assigner_data.no_opponent_obstacles,
                                target_vect, targetFunction, ballIsObstacle, avoidBallPath, stayInPlayingField, BallTargetPos);

    std::vector<path_piece_t> path = visibilityGraph.getShortestPath(r_role_assigner_data);

    if (logDynamicPlanner) {
        MRA_LOG_INFO("Found path with of length %d", path.size());
        MRA_LOG_INFO("nrIterations = %d", nrIterations);
    }
    while ((nrIterations - iteration > 0) && (!intercept_data.move_to_ball_leave_field_pos)) {
        // See how long it takes to get to intercept point
        // add extra 300 milliseconds (default delay, due to vision processing and role assigner execution frequency
        double time = timeOnPath(path, maxSpeed) + 0.300;
        if (logDynamicPlanner) {
            MRA_LOG_INFO("Path takes %f seconds. Recalculating.", time);
        }
        // Calculate new intercept point based on minimum time
        Geometry::Position newInterceptPosition =  r_role_assigner_data.ball.position + (r_role_assigner_data.ball.velocity*time);
        if (newInterceptPosition.x == std::numeric_limits<double>::has_quiet_NaN) {
            // Return previous path
            if (logDynamicPlanner) {
                MRA_LOG_INFO("New intercept impossible. Returning last path.");
            }
            return path;
        }

        auto newInterceptInField = r_role_assigner_data.environment.isInField(newInterceptPosition, 0.0);
        if (!newInterceptInField) {
            auto interceptPoint = Dynamics::calculateBallLeavingFieldPoint(r_role_assigner_data.ball, r_role_assigner_data.environment);
            newInterceptPosition.x = interceptPoint.x;
            newInterceptPosition.y = interceptPoint.y;
        }

        target_vect.clear();
        target_vect.push_back(Vertex(newInterceptPosition, 0));
        GlobalPathPlanner visibilityGraph2 = GlobalPathPlanner(r_role_assigner_data.environment); // create new visibility-graph to avoid dynamic memory issues
        visibilityGraph2.setOptions(r_role_assigner_data.parameters);
        visibilityGraph2.createGraph(start_pose, start_vel, r_role_assigner_data.ball, filtered_teammates, 
                                     r_role_assigner_data.opponents,  r_role_assigner_data.no_opponent_obstacles,
                                     target_vect, targetFunction, ballIsObstacle, avoidBallPath, stayInPlayingField, BallTargetPos);
        path = visibilityGraph2.getShortestPath(r_role_assigner_data);
        if (logDynamicPlanner) {
            MRA_LOG_INFO("New path of length: %d", path.size());
            MRA_LOG_INFO("iteration: %d target_vect.size(): %d", iteration,  target_vect.size());
        }
        iteration++;
    }
    return path;
}

double GlobalPathDynamicPlanner::timeOnPath(const std::vector<path_piece_t>& path, double maxSpeed) {
    // Handle boundary conditions
    if (path.size() < 2) {
        return 0.0;
    } else if (maxSpeed < 0.000001) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    // Get total length of path
    double length = 0.0;
    for (unsigned int index = 1; index < path.size(); ++index) {
        Geometry::Position current = Geometry::Position(path[index].x, path[index].y);
        Geometry::Position previous = Geometry::Position(path[index-1].x, path[index-1].y);
        length += current.distanceTo(previous);
    }
    return length / maxSpeed;
}

