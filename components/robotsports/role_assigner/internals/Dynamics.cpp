/**
 *  @file
 *  @brief   Collection of static methods for calculating time related properties.
 *  @curator Jürge van Eijck
 */
#include "Dynamics.hpp"

#include "MathUtils.hpp"
#include "geometry.hpp"
#include "FieldConfig.hpp"
#include "RoleAssignerData.hpp"

#include <cmath>
#include <limits>
#include <iostream>

using namespace MRA;

// calculation position where the ball will leave the field
Geometry::Position Dynamics::calculateBallLeavingFieldPoint(const RoleAssignerBall& rBallObject, const FieldConfig &rFieldConfig) {
    Geometry::Position BallPos = rBallObject.position;
    Geometry::Position BallVelocity = rBallObject.velocity;

    Geometry::Position leavingPoint = BallPos;

    Geometry::Position p2 = BallPos + BallVelocity; // calculate 2nd point of ball traject.
    int sideline_direction = 1; // positive direction (+x)
    if (BallVelocity.x < 0) {
        sideline_direction = -1; // negative direction (-x)
    }
    int backline_direction = 1; // positive direction (+y)
    if (BallVelocity.y < 0) {
        backline_direction = -1; // negative direction (-y)
    }
    double margin = 0.25 * rFieldConfig.getRobotSize(); // keep ball in the field, player just outside the field
    double interSectionSideLineX, interSectionSideLineY, interSectionBackLineX,
            interSectionBackLineY = 0.0;
    bool intersectWithBackLine = getIntersectionOfTwoLines(
            interSectionBackLineX, interSectionBackLineY, BallPos.x, BallPos.y,
            p2.x, p2.y, rFieldConfig.getMaxFieldX(),
            backline_direction * (rFieldConfig.getMaxFieldY() + margin),
            -rFieldConfig.getMaxFieldX(),
            backline_direction * (rFieldConfig.getMaxFieldY() + margin));
    bool intersectWithSideLine = getIntersectionOfTwoLines(
            interSectionSideLineX, interSectionSideLineY, BallPos.x, BallPos.y,
            p2.x, p2.y,
            sideline_direction * (rFieldConfig.getMaxFieldX() + margin),
            rFieldConfig.getMaxFieldY(),
            sideline_direction * (rFieldConfig.getMaxFieldX() + margin),
            -rFieldConfig.getMaxFieldY());
    if (intersectWithBackLine && !intersectWithSideLine) {
        // intersection only with backline
        leavingPoint.x = interSectionBackLineX;
        leavingPoint.y = interSectionBackLineY;
    } else if (!intersectWithBackLine && intersectWithSideLine) {
        // intersection only with sideline
        leavingPoint.x = interSectionSideLineX;
        leavingPoint.y = interSectionSideLineY;
    } else if (intersectWithBackLine && intersectWithSideLine) {
        // intersect with both lines.
        // use point closest to ball position
        Geometry::Position sidelinePoint = Geometry::Position(interSectionSideLineX,
                interSectionSideLineY);
        Geometry::Position backlinePoint = Geometry::Position(interSectionBackLineX,
                interSectionBackLineY);
        if (BallPos.distanceTo(sidelinePoint) < BallPos.distanceTo(backlinePoint)) {
            leavingPoint = sidelinePoint; // closest to sideline
        } else {
            leavingPoint = backlinePoint;
        }
    }
    return leavingPoint;
}

Dynamics::dynamics_t Dynamics::interceptBall(const RoleAssignerBall& rBallObject,
        const Geometry::Point& meCoordinates, double maxSpeed, const FieldConfig& fieldConfig, bool move_to_ball_left_field_position) {
    dynamics_t result = {};

    // for clarity, lets assume Me wants to intercept a moving Ball
    Geometry::Point ballVelocity = Geometry::Point(rBallObject.velocity.x, rBallObject.velocity.y);

    double ballSpeed = ballVelocity.size();
    if (ballSpeed < 0.0001) {
        // When object is not moving, the intercept point is the object
        // position itself
        result.intercept_position = rBallObject.position;
        return result;
    }
    Geometry::Point point1(meCoordinates.x, meCoordinates.y);
    Geometry::Point ball(rBallObject.position.x, rBallObject.position.y);
    point1 -= ball;
    double distance = meCoordinates.distanceTo(ball);
    double cosBeta = point1.inproduct(ballVelocity)/ (distance * ballSpeed);

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
            time = std::min(time1, time2);
            if (time < 0) {
                time = std::max(time1, time2);
            }
        }
    }
    result.intercept_position = ball;
    if (time < 0) {
        // Intercept impossible
        result.move_to_ball_leave_field_pos = true; // no intercept possible
    } else {
        result.intercept_position = ball;
        Geometry::Point extention = ballVelocity;
        extention *= time;
        result.intercept_position += extention;
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


