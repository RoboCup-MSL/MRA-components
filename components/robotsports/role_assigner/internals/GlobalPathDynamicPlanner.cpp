/**
 *  @file
 *  @brief   Class for the dynamic robot planner
 *  @curator Jürge van Eijck
 */
#include "GlobalPathDynamicPlanner.hpp"
#include "Dynamics.hpp"
#include "Vertex.hpp"
#include "GlobalPathPlanner.hpp"

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
std::vector<planner_piece_t> GlobalPathDynamicPlanner::planPath(const MRA::Geometry::Position& start_pose, const MRA::Geometry::Position& start_vel,
                                                   const std::vector<RoleAssignerRobot>& filtered_teammates, const TeamPlannerData& r_teamPlannerData,
                                                   const std::vector<MRA::Vertex>& targetPos, planner_target_e targetFunction, bool ballIsObstacle,
                                                   double maxSpeed, int nrIterations,  bool stayInPlayingField)
{
    bool logDynamicPlanner = false;

    if (logDynamicPlanner) {
        MRA_LOG_INFO("Planning path with %d iterations", nrIterations);
    }
    // Currently we do not use the initial robot speed

    // Find initial intercept point
    Dynamics::dynamics_t intercept_data = Dynamics::interceptBall(r_teamPlannerData.ball,
                                                 start_pose, maxSpeed,
                                                 r_teamPlannerData.fieldConfig, r_teamPlannerData.parameters.move_to_ball_left_field_position);
    if (intercept_data.intercept_position.x == std::numeric_limits<double>::has_quiet_NaN) {
        // No intercept possible, return empty list
        if (logDynamicPlanner) {
            MRA_LOG_INFO("No intercept possible");
        }
        return std::vector<planner_piece_t>();
    }
    if (logDynamicPlanner) {
        MRA_LOG_INFO("calculate interception point %s", intercept_data.intercept_position.toString().c_str());
    }

    GlobalPathPlanner visibilityGraph = GlobalPathPlanner(r_teamPlannerData.fieldConfig);
    visibilityGraph.setOptions(r_teamPlannerData.parameters);

    bool avoidBallPath = false; // Not need to avoid the ball. This function is only used for the interceptor
    Geometry::Point BallTargetPos;

    int iteration = 1;

    std::vector<MRA::Vertex> target_vect;
    target_vect.push_back(Vertex(intercept_data.intercept_position, 0));
    visibilityGraph.createGraph(start_pose, start_vel, r_teamPlannerData.ball, filtered_teammates, r_teamPlannerData.opponents,
                                target_vect, targetFunction, ballIsObstacle, avoidBallPath, stayInPlayingField, BallTargetPos);

    std::vector<planner_piece_t> path = visibilityGraph.getShortestPath(r_teamPlannerData);

    if (logDynamicPlanner) {
        MRA_LOG_INFO("Found path with of length %d", path.size());
        MRA_LOG_INFO("nrIterations = %d", nrIterations);
    }
    while ((nrIterations - iteration > 0) && (!intercept_data.move_to_ball_leave_field_pos)) {
        // See how long it takes to get to intercept point
        // add extra 300 milliseconds (default delay, due to vision processing and teamplanner execution frequency
        double time = Dynamics::timeOnPath(path, maxSpeed) + 0.300;
        if (logDynamicPlanner) {
            MRA_LOG_INFO("Path takes %f seconds. Recalculating.", time);
        }
        // Calculate new intercept point based on minimum time
        Geometry::Position newInterceptPosition =  r_teamPlannerData.ball.position + (r_teamPlannerData.ball.velocity*time);
        if (newInterceptPosition.x == std::numeric_limits<double>::has_quiet_NaN) {
            // Return previous path
            if (logDynamicPlanner) {
                MRA_LOG_INFO("New intercept impossible. Returning last path.");
            }
            return path;
        }

        auto newInterceptInField = r_teamPlannerData.fieldConfig.isInField(newInterceptPosition, 0.0);
        if (!newInterceptInField) {
            auto interceptPoint = Dynamics::calculateBallLeavingFieldPoint(r_teamPlannerData.ball, r_teamPlannerData.fieldConfig);
            newInterceptPosition.x = interceptPoint.x;
            newInterceptPosition.y = interceptPoint.y;
        }

        target_vect.clear();
        target_vect.push_back(Vertex(newInterceptPosition, 0));
        GlobalPathPlanner visibilityGraph2 = GlobalPathPlanner(r_teamPlannerData.fieldConfig); // create new visibility-graph to avoid dynamic memory issues
        visibilityGraph2.setOptions(r_teamPlannerData.parameters);
        visibilityGraph2.createGraph(start_pose, start_vel, r_teamPlannerData.ball, filtered_teammates, r_teamPlannerData.opponents,
                                     target_vect, targetFunction, ballIsObstacle, avoidBallPath, stayInPlayingField, BallTargetPos);
        path = visibilityGraph2.getShortestPath(r_teamPlannerData);
        if (logDynamicPlanner) {
            MRA_LOG_INFO("New path of length: %d", path.size());
            MRA_LOG_INFO("iteration: %d target_vect.size(): %d", iteration,  target_vect.size());
        }
        iteration++;
    }
    return path;
}

