/**
 *  @file
 *  @brief   Collection of static methods for calculating time related properties.
 *  @curator Jürge van Eijck
 */
#include "Dynamics.h"
#include "MathUtils.h"
#include <cmath>
#include <limits>
#include <iostream>

using namespace trs;

// calculation position where the ball will leave the field
Vector2D Dynamics::calculateBallLeavingFieldPoint(const MovingObject& rBallObject, const FieldConfig &rFieldConfig) {
	Vector2D BallPos = rBallObject.getXYlocation();
	Vector2D BallVelocity;
	double Vr = 0.0;
	rBallObject.getVelocity(BallVelocity, Vr);

	Vector2D leavingPoint = BallPos;

	Vector2D p2 = BallPos.add(BallVelocity); // calculate 2nd point of ball traject.
	int sideline_direction = 1; // positive direction (+x)
	if (BallVelocity.m_x < 0) {
		sideline_direction = -1; // negative direction (-x)
	}
	int backline_direction = 1; // positive direction (+y)
	if (BallVelocity.m_y < 0) {
		backline_direction = -1; // negative direction (-y)
	}
	double margin = 0.25 * rFieldConfig.getRobotSize(); // keep ball in the field, player just outside the field
	double interSectionSideLineX, interSectionSideLineY, interSectionBackLineX,
			interSectionBackLineY = 0.0;
	bool intersectWithBackLine = getIntersectionOfTwoLines(
			interSectionBackLineX, interSectionBackLineY, BallPos.m_x, BallPos.m_y,
			p2.m_x, p2.m_y, rFieldConfig.getMaxFieldX(),
			backline_direction * (rFieldConfig.getMaxFieldY() + margin),
			-rFieldConfig.getMaxFieldX(),
			backline_direction * (rFieldConfig.getMaxFieldY() + margin));
	bool intersectWithSideLine = getIntersectionOfTwoLines(
			interSectionSideLineX, interSectionSideLineY, BallPos.m_x, BallPos.m_y,
			p2.m_x, p2.m_y,
			sideline_direction * (rFieldConfig.getMaxFieldX() + margin),
			rFieldConfig.getMaxFieldY(),
			sideline_direction * (rFieldConfig.getMaxFieldX() + margin),
			-rFieldConfig.getMaxFieldY());
	if (intersectWithBackLine && !intersectWithSideLine) {
		// intersection only with backline
		leavingPoint.m_x = interSectionBackLineX;
		leavingPoint.m_y = interSectionBackLineY;
	} else if (!intersectWithBackLine && intersectWithSideLine) {
		// intersection only with sideline
		leavingPoint.m_x = interSectionSideLineX;
		leavingPoint.m_y = interSectionSideLineY;
	} else if (intersectWithBackLine && intersectWithSideLine) {
		// intersect with both lines.
		// use point closest to ball position
		Vector2D sidelinePoint = Vector2D(interSectionSideLineX,
				interSectionSideLineY);
		Vector2D backlinePoint = Vector2D(interSectionBackLineX,
				interSectionBackLineY);
		if (BallPos.distanceTo(sidelinePoint) < BallPos.distanceTo(backlinePoint)) {
			leavingPoint = sidelinePoint; // closest to sideline
		} else {
			leavingPoint = backlinePoint;
		}
	}
	return leavingPoint;
}

Dynamics::dynamics_t Dynamics::interceptBall(const MovingObject& rBallObject,
		const Vector2D& coordinates, double maxSpeed, const FieldConfig& fieldConfig, bool move_to_ball_left_field_position) {
	dynamics_t result = {};

	// for clarity, lets assume Me wants to intercept a moving Ball
	Vector2D ball = rBallObject.getPosition().getVector2D();
	Vector2D ballVelocity;
	double vrz;
	rBallObject.getVelocity(ballVelocity, vrz);
	Vector2D meCoordinates = coordinates;

	double ballSpeed = ballVelocity.norm();
	if (ballSpeed < 0.0001) {
		// When object is not moving, the intercept point is the object
		// position itself
		result.intercept_position = ball;
		return result;
	}
	double distance = meCoordinates.distanceTo(ball);
	double cosBeta = meCoordinates.subtract(ball).inproduct(ballVelocity)
						/ (distance * ballSpeed);

	double relativeSpeed = fabs(ballSpeed - maxSpeed);
	double time;
	if (relativeSpeed < 0.0001) {
		// Special case, linear equation
		if (cosBeta < 0.0001) {
			time = -1.0; // intercept is impossible
		} else {
			time = distance / (2 * ballSpeed * cosBeta);
		}
	} else {
		// Root of quadratic equation
		double a = (ballSpeed * ballSpeed) - (maxSpeed * maxSpeed);
		double b = -2 * ballSpeed * distance * cosBeta;
		double c = distance * distance;
		double D = b * b - 4 * a * c;
		if (D < 0.0) {
			time = -1.0; // intercept impossible
		} else {
			double time1 = (-b + sqrt(D)) / (2 * a);
			double time2 = (-b - sqrt(D)) / (2 * a);
			time = min(time1, time2);
			if (time < 0) {
				time = max(time1, time2);
			}
		}
	}
	result.intercept_position = ball;
	if (time < 0) {
		// Intercept impossible
		result.move_to_ball_leave_field_pos = true; // no intercept possible
	} else {
		result.intercept_position = ball.add(ballVelocity.multiply(time));
	}

	if (!fieldConfig.isInField(result.intercept_position, 0.25*fieldConfig.getRobotRadius()))
	{
		// interception out of field (used margin: max robot radius outside the field)
		result.move_to_ball_leave_field_pos = true;
	}


	if (result.move_to_ball_leave_field_pos )
	{
		if (!move_to_ball_left_field_position) {
			result.intercept_position = ball;
		}
		else {
			result.intercept_position = calculateBallLeavingFieldPoint(rBallObject, fieldConfig);
		}
	}

	return result;
}

double Dynamics::timeOnPath(const vector<planner_piece_t>& path, double maxSpeed) {
	// Handle boundary conditions
	if (path.size() < 2) {
		return 0.0;
	} else if (maxSpeed < 0.000001) {
		return std::numeric_limits<double>::quiet_NaN();
	}
	// Get total length of path
	double length = 0.0;
	for (unsigned int index = 1; index < path.size(); ++index) {
		Vector2D current = Vector2D(path[index].x, path[index].y);
		Vector2D previous = Vector2D(path[index-1].x, path[index-1].y);
		length += current.distanceTo(previous);
	}
	return length / maxSpeed;
}

