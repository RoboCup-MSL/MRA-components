/**
 *  @file
 *  @brief   Class for the dynamic robot planner
 *  @curator Jürge van Eijck
 */
#ifndef GlobalPathDynamicPlanner_H
#define GlobalPathDynamicPlanner_H 1

#include <vector>

#include "GlobalPathPlanner.hpp"
#include "geometry.hpp"
#include "TeamPlannerParameters.hpp"

namespace MRA {
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
    std::vector<planner_piece_t> planPath(const MRA::Geometry::Position& start_pose, const MRA::Geometry::Position& start_vel,
    		const TeamPlannerData& teamplanner_data,
            const std::vector<MRA::Vertex>& targetPos, planner_target_e targetFunction,
			double maxSpeed, int nrIterations);

private:

};

} // namespace
#endif
