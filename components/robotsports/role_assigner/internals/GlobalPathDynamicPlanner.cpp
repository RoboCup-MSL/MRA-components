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

using namespace trs;

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
std::vector<planner_piece_t> GlobalPathDynamicPlanner::planPath(const MovingObject& start, MovingObject ball, const std::vector<MovingObject>& teammates,
			const std::vector<MovingObject>& opponents, const std::vector<trs::Vertex>& targetPos, planner_target_e targetFunction, bool ballIsObstacle,
			double maxSpeed, const PlannerOptions& plannerOptions, int nrIterations, const FieldConfig& fieldConfig) {
	bool logDynamicPlanner = false;

	if (logDynamicPlanner) {
//		logAlways("Planning path with %d iterations", nrIterations);
	}
	// Currently we do not use the initial robot speed
	Position startPosition = start.getPosition();

	// Find initial intercept point
	Dynamics::dynamics_t intercept_data = Dynamics::interceptBall(ball,
			                                     startPosition.getPoint(), maxSpeed,
												 fieldConfig, plannerOptions.move_to_ball_left_field_position);
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

	GlobalPathPlanner visibilityGraph = GlobalPathPlanner(fieldConfig);
	visibilityGraph.setOptions(plannerOptions);

	bool avoidBallPath = false; // Not need to avoid the ball. This function is only used for the interceptor
	MRA::Geometry::Point BallTargePos;

	int iteration = 1;

	vector<trs::Vertex> target_vect;
	target_vect.push_back(Vertex(intercept_data.intercept_position, 0));
	visibilityGraph.createGraph(start, ball, teammates, opponents, target_vect, targetFunction, ballIsObstacle, avoidBallPath, BallTargePos);
	std::vector<planner_piece_t> path = visibilityGraph.getShortestPath();

	if (logDynamicPlanner) {
//		logAlways("Found path with of length %d", path.size());
//		logAlways("nrIterations = %d", nrIterations);
	}
	while ((nrIterations - iteration > 0) && (!intercept_data.move_to_ball_leave_field_pos)) {
		// See how long it takes to get to intercept point
		// add extra 300 milliseconds (default delay, due to vision processing and teamplanner execution frequency
		double time = Dynamics::timeOnPath(path, maxSpeed) + 0.300;
		if (logDynamicPlanner) {
//			logAlways("Path takes %f seconds. Recalculating.", time);
		}
		// Calculate new intercept point based on minimum time
		Position newInterceptPosition =  ball.getPositionAt(time);
		if (newInterceptPosition.getPoint().x == std::numeric_limits<double>::has_quiet_NaN) {
			// Return previous path
			if (logDynamicPlanner) {
//				logAlways("New intercept impossible. Returning last path.");
			}
			return path;
		}

		auto newInterceptInField = fieldConfig.isInField(newInterceptPosition.getPoint(), 0.0);
		if (!newInterceptInField) {
			auto interceptPoint = Dynamics::calculateBallLeavingFieldPoint(ball, fieldConfig);
			newInterceptPosition.set(interceptPoint.x, interceptPoint.y, 0.0);
		}

		target_vect.clear();
		target_vect.push_back(Vertex(newInterceptPosition.getPoint(), 0));
		GlobalPathPlanner visibilityGraph2 = GlobalPathPlanner(fieldConfig); // create new visibility-graph to avoid dynamic memory issues
		visibilityGraph2.setOptions(plannerOptions);
		visibilityGraph2.createGraph(start, ball, teammates, opponents, target_vect, targetFunction, ballIsObstacle, avoidBallPath, BallTargePos);
		path = visibilityGraph2.getShortestPath();
		if (logDynamicPlanner) {
//			logAlways("New path of length: %d", path.size());
//			logAlways("iteration: %d target_vect.size(): %d", iteration,  target_vect.size());
		}
		iteration++;
	}
	return path;
}

