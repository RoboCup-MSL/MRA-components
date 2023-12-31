/**
 *  @file
 *  @brief   Class for the dynamic robot planner
 *  @curator Jürge van Eijck
 */
#include "GlobalPathDynamicPlanner.hpp"

#include <iostream>
#include <sstream>
#include <limits>

#include "Dynamics.hpp"
#include "GlobalPathPlanner.hpp"
#include "Vertex.hpp"

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
std::vector<planner_piece_t> GlobalPathDynamicPlanner::planPath(const MRA::Geometry::Pose& start_pose, const MRA::Geometry::Pose& start_vel, const TeamPlannerData& teamplanner_data,
        const std::vector<MRA::Vertex>& targetPos, planner_target_e targetFunction, double maxSpeed, int nrIterations) {
	bool logDynamicPlanner = false;

	if (logDynamicPlanner) {
//		logAlways("Planning path with %d iterations", nrIterations);
	}

	// Find initial intercept point
	Dynamics::dynamics_t intercept_data = Dynamics::interceptBall(teamplanner_data.ball,
	                                            start_pose, maxSpeed,
			                                     teamplanner_data.fieldConfig, teamplanner_data.parameters.move_to_ball_left_field_position);
	if (intercept_data.intercept_position.x == std::numeric_limits<double>::has_quiet_NaN) {
		// No intercept possible, return empty list
		if (logDynamicPlanner) {
//			logAlways("No intercept possible");
		}
		return std::vector<planner_piece_t>();
	}
	if (logDynamicPlanner) {
//		logAlways("calculate interception point %s", intercept_data.intercept_position.toString().c_str());
	}

	GlobalPathPlanner visibilityGraph = GlobalPathPlanner(teamplanner_data.fieldConfig);
	visibilityGraph.setOptions(teamplanner_data.parameters);

	bool avoidBallPath = false; // Not need to avoid the ball. This function is only used for the interceptor
	MRA::Geometry::Point BallTargePos;

	int iteration = 1;

	std::vector<MRA::Vertex> target_vect;
	target_vect.push_back(Vertex(intercept_data.intercept_position, 0));
	visibilityGraph.createGraph(start_pose, start_vel, teamplanner_data, target_vect, targetFunction, teamplanner_data.ballIsObstacle, avoidBallPath, BallTargePos);
	std::vector<planner_piece_t> path = visibilityGraph.getShortestPath(teamplanner_data);

	if (logDynamicPlanner) {
//		logAlways("Found path with of length %d", path.size());
//		logAlways("nrIterations = %d", nrIterations);
	}
	while ((nrIterations - iteration > 0) && (!intercept_data.move_to_ball_leave_field_pos)) {
		// See how long it takes to get to intercept point
		// add extra 300 milliseconds (default delay, due to vision processing and teamplanner execution frequency
		double timespan = Dynamics::timeOnPath(path, maxSpeed) + 0.300;
		if (logDynamicPlanner) {
		    // logAlways("Path takes %f seconds. Recalculating.", time);
		}
		// Calculate new intercept point based on minimum time
		Geometry::Pose newInterceptPosition = teamplanner_data.ball.position; //nAt(time);
		Geometry::Pose move_offset = teamplanner_data.ball.velocity;
		move_offset *= timespan;
		newInterceptPosition += move_offset;
		if (newInterceptPosition.x == std::numeric_limits<double>::has_quiet_NaN) {
			// Return previous path
			if (logDynamicPlanner) {
			    // logAlways("New intercept impossible. Returning last path.");
			}
			return path;
		}

		auto newInterceptInField = teamplanner_data.fieldConfig.isInField(newInterceptPosition, 0.0);
		if (!newInterceptInField) {
			auto interceptPoint = Dynamics::calculateBallLeavingFieldPoint(teamplanner_data.ball, teamplanner_data.fieldConfig);
			newInterceptPosition.x = interceptPoint.x;
			newInterceptPosition.y = interceptPoint.y;
		}

		target_vect.clear();
		target_vect.push_back(Vertex(newInterceptPosition, 0));
		GlobalPathPlanner visibilityGraph2 = GlobalPathPlanner(teamplanner_data.fieldConfig); // create new visibility-graph to avoid dynamic memory issues
		visibilityGraph2.setOptions(teamplanner_data.parameters);
		visibilityGraph2.createGraph(start_pose, start_vel, teamplanner_data, target_vect, targetFunction, teamplanner_data.ballIsObstacle, avoidBallPath, BallTargePos);
		path = visibilityGraph2.getShortestPath(teamplanner_data);
		if (logDynamicPlanner) {
//			logAlways("New path of length: %d", path.size());
//			logAlways("iteration: %d target_vect.size(): %d", iteration,  target_vect.size());
		}
		iteration++;
	}
	return path;
}

