/**
 *  @file
 *  @brief   Collection of static methods for calculating time related properties.
 *  @curator Jürge van Eijck
 */

#ifndef DYNAMICS_HPP
#define DYNAMICS_HPP 1

#include "geometry.hpp"

namespace MRA {

class RoleAssignerBall;
class Environment;

class Dynamics {
    public:

    typedef struct dynamics_s {
        MRA::Geometry::Point intercept_position; // position of interception
        bool move_to_ball_leave_field_pos = false;  // interception position is point where ball leaves field: true | false
    } dynamics_t;

    /**
     * Calculates the intercept point. This method works in 2D only.
     * 
     * @param movingObject
     *            The object that we want to intercept. Assuming constant
     *            velocity.
     * @param coordinates
     *            Our coordinates.
     * @param maxSpeed
     *            Our maximum speed. Assuming indefinite acceleration.
     * @return Intercept point, null if intercept is not possible.
     */
    static dynamics_t interceptBall(const RoleAssignerBall& movingObject, const MRA::Geometry::Point& coordinates,
            double maxSpeed, const Environment& rEnvironment, bool move_to_ball_left_field_position);


    /**
     * Calculates the point where ball is leaving the field. This method works in 2D only.
     *
     * @param rBall  position of the ball
     * @param rBallVelocity velocity of the ball
     * @param rEnvironment field configuration
     * @return point where ball leaves the field.
     */
    static MRA::Geometry::Position calculateBallLeavingFieldPoint(const RoleAssignerBall& rBallObject, const Environment &rEnvironment);
};

} // namespace
#endif /* DYNAMICS_HPP */
