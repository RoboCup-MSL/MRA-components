/**
 *  @file
 *  @brief   Class for the dynamic robot planner
 *  @curator Jürge van Eijck
 */
#ifndef GlobalPathDynamicPlanner_HPP
#define GlobalPathDynamicPlanner_HPP 1

#include "geometry.hpp"
#include <vector>
#include "RoleAssigner_types.hpp"

namespace MRA {

class RoleAssignerRobot;
class RoleAssignerData;
class Vertex;

/**
 * A planner that takes into account the movement of the start and target locations (assuming constant velocity), and
 * stationary barrier objects that must be avoided.
 */
class GlobalPathDynamicPlanner {

public:

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
    std::vector<path_piece_t> planPath(const MRA::Geometry::Position& start_pose, const MRA::Geometry::Position& start_vel,
                                          const std::vector<RoleAssignerRobot>& r_teammates, const RoleAssignerData& r_roleAssignerData,
                                          const std::vector<MRA::Vertex>& r_targetPos, planner_target_e targetFunction, bool ballIsObstacle,
                                          double maxSpeed, int nrIterations, bool stayInPlayingField);

private:
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
    double timeOnPath(const std::vector<path_piece_t>& path, double maxSpeed);


};

} // namespace
#endif /* GlobalPathDynamicPlanner_HPP */
