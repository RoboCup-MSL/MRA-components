/**
 *  @file
 *  @brief   Collection of static methods for calculating time related properties.
 *  @curator Jürge van Eijck
 */

#ifndef DYNAMICS_H
#define DYNAMICS_H 1

#include "geometry.hpp"
#include <vector>

#include "GlobalPathPlanner.hpp" // for planner_piece_t

namespace MRA {

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
    static dynamics_t interceptBall(const TeamPlannerBall& movingObject, const MRA::Geometry::Point& coordinates,
            double maxSpeed, const FieldConfig& fieldConfig, bool move_to_ball_left_field_position);


    /**
     * Calculates the point where ball is leaving the field. This method works in 2D only.
     *
     * @param rBall  position of the ball
     * @param rBallVelocity velocity of the ball
     * @param rFieldConfig field configuration
     * @return point where ball leaves the field.
     */
    static MRA::Geometry::Position calculateBallLeavingFieldPoint(const TeamPlannerBall& rBallObject, const FieldConfig &rFieldConfig);

    /**
     * Calculates time necessary to traverse a path at maximum speed, assuming
     * indefinite acceleration. If a path has 0 or 1 point, the time is 0.0,
     * otherwise, if the maximum speed is very low, the time returned is NaN.
     * 
     * @param path
     *            Path to travel
     * @param maxSpeed
     *            Maximum possible speed
     * @return Time needed to traverse the path
     */
    static double timeOnPath(const std::vector<planner_piece_t>& path, double maxSpeed);
};

} // namespace
#endif
