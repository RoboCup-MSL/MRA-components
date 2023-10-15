/**
 *  @file
 *  @brief   Class for the dynamic robot planner
 *  @curator Jürge van Eijck
 */
#ifndef GlobalPathDynamicPlanner_H
#define GlobalPathDynamicPlanner_H 1

#include <vector>

#include "PlannerOptions.h"
#include "GlobalPathPlanner.h"
#include "MovingObject.h"
#include "Position.h"
#include "Vector2D.h"

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
    std::vector<planner_piece_t> planPath(const MovingObject& start, MovingObject ball, const std::vector<MovingObject>& teammates,
			const std::vector<MovingObject>& opponents, const std::vector<trs::Vertex>& targetPos, planner_target_e targetFunction,
			bool ballIsObstacle, double maxSpeed, const PlannerOptions& plannerOptions, int nrIterations, const FieldConfig& fieldConfig);

private:

};

} // namespace
#endif
