/**
 *  @file
 *  @brief   Class for the dynamic robot planner
 *  @curator Jürge van Eijck
 */
#ifndef GlobalPathDynamicPlanner_H
#define GlobalPathDynamicPlanner_H 1

#include <vector>

#include "GlobalPathPlanner.hpp"
#include "MovingObject.h"
#include "Position.h"
#include "geometry.hpp"
#include "TeamPlannerParameters.hpp"

namespace trs {
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
    std::vector<planner_piece_t> planPath(const MovingObject& start, const TeamPlannerData& teamplanner_data,
            const std::vector<trs::Vertex>& targetPos, planner_target_e targetFunction,
			bool ballIsObstacle, double maxSpeed, const TeamPlannerParameters& plannerOptions, int nrIterations);

private:

};

} // namespace
#endif
