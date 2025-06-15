/*
 * AvoidObstacles.cpp
 *
 *  Created on: August, 2019
 *      Author: Josefine Quack
 *
 * Algorithm originally implemented by Erik.
 * Paper: http://www.techunited.nl/media/files/realtime_motion_path_generation_using_subtargets_in_a_changing_environment.pdf
 */


#include "PathPlanningAlgorithms.hpp"
// #include "cDiagnostics.hpp"
#include "logging.hpp"

// internal constants, types and utilities

typedef struct
{
    MRA::Geometry::Point location;
    MRA::Geometry::Point velocity;
    double a_i = 0.0;
    double b_i = 0.0;
    double radius = 0.0;
} pp_obstacle_struct_t;

void convertAndAddObstacles(std::vector<pp_obstacle_struct_t> &dst, std::vector<obstacleResult> const &src)
{
    for (auto it = src.begin(); it != src.end(); ++it)
    {
        pp_obstacle_struct_t obst;
        obst.location = it->position;
        obst.velocity = it->velocity;
        dst.push_back(obst);
    }
}

struct posval_compare // for std::set
{
    bool operator() (const pp_obstacle_struct_t& lhs, const pp_obstacle_struct_t& rhs) const
    {
        if (lhs.location.x < rhs.location.x)
            return true;
        else if (lhs.location.x > rhs.location.x)
            return false;
        else if (lhs.location.y < rhs.location.y)
            return true;
        else
            return false;
    }
};


// obstacle avoidance algorithm

void AvoidObstacles::execute(PathPlanningData &data)
{
    // get target, initialize
    MRA::Geometry::Position target = data.getSubTarget();
    std::string msg = "target: ";
    msg.append(target.toString());
    MRA_LOG_DEBUG(msg.c_str());
    auto config = data.parameters.obstacleAvoidance;

    // check if functionality is enabled
    if (config.enabled == false)
    {
        MRA_LOG_DEBUG("obstacle avoidance is disabled");
        return;
    }

    // get other data
    MRA::Geometry::Position currPos(data.robot.position.x, data.robot.position.y, data.robot.position.rz);

    // get calculated obstacles, convert to internal type
    std::vector<pp_obstacle_struct_t> obstacles;
    convertAndAddObstacles(obstacles, data.calculatedObstacles); // calculated by CalculateObstacles

    MRA_LOG_DEBUG("Starting algorithm with nr_obs: %d", obstacles.size());

    MRA::Geometry::Position subTarget = target;
    bool calculatedSubTarget = false;
    MRA::Geometry::Point targetXY(target.x, target.y);
    MRA::Geometry::Point curPosXY(currPos.x, currPos.y);

    // Guard with moving at least 10cm
    if ((curPosXY - targetXY).size() > 0.1)
    {
        std::vector<pp_obstacle_struct_t> B;
        {
            MRA_LOG_DEBUG("cXYTrajectory#Step1", "");
            // 1. Determine B = Set of all obstacles in the direct path from robot position to target position, taking also the diameter of the obstacles and the robot into account.
            for (int i = 0; i < (int)obstacles.size(); i++)
            {
                // Pre-compute a_i and b_i for all obstacles, and store in the struct for later reuse.
                MRA::Geometry::Point obstVec = MRA::Geometry::Point(obstacles[i].location.x, obstacles[i].location.y);
                // TODO: Jurge
                MRA::Geometry::Point tgtCur = (targetXY - curPosXY);
                MRA::Geometry::Point obstCur = (obstVec - curPosXY);

                obstacles[i].a_i = (tgtCur.x*obstCur.x + tgtCur.y*obstCur.y) / (targetXY - curPosXY).size();
                // where (tgtCur.x*obstCur.x + tgtCur.y*obstCur.y) : vector2d(a) * vector2d(b)

                obstacles[i].b_i = (tgtCur.x*obstCur.y + tgtCur.y*obstCur.x) / (targetXY - curPosXY).size();
                //where  (tgtCur.x*obstCur.y + tgtCur.y*obstCur.x) : vector2d(a).CrossProduct(vector2d(b))

                obstacles[i].radius = config.obstacleRadius;
                // ROADMAP: vision/worldModel might know radius better at some point

                // Equation (4) from the paper.
                if (( 0 < obstacles[i].a_i ) &&
                    ( obstacles[i].a_i < (targetXY - curPosXY).size() ) &&
                    ( fabs(obstacles[i].b_i) < (config.robotRadius + obstacles[i].radius) )) // r_r + r_i
                {
                    B.push_back(obstacles[i]);
                    //TRACE("DIST FROM LINE: %6.2f", obstacles[i].b_i);
                    //TRACE("OBST IN PATH: X: %6.2f, Y: %6.2f", obstVec.x, obstVec.y);
                }

            }
        }

        MRA_LOG_DEBUG("Computed step 1.");

        int B_idx = -1;
        {
            MRA_LOG_DEBUG("cXYTrajectory#Step2", "");
            // 2. Determine f = The obstacle from B that is the nearest to the robot
            double minDist = 100.0;

            for (int i = 0; i < (int)B.size(); i++)
            {
                // Find obstacle with the smallest a_i.
                // Equation (5) from the paper.
                if (B[i].a_i < minDist)
                {
                    minDist = B[i].a_i;
                    B_idx = i;
                }
            }
        }

        MRA_LOG_DEBUG("Computed step 2.");

        if (B_idx != -1)
        {
            //TRACE("NEAREST OBST: X: %6.2f, Y: %6.2f", B[B_idx].location.x, B[B_idx].location.y);

            // 3. Determine G = The set of transitive obstacles that are within the range of 2r_r of each other, starting from f.
            //    "In our approach, the group G will be avoided instead of object f only, otherwise the robot could get stuck in the group G."
            //    G = {f};
            //    while True:
            //       Gsize = len(G)
            //       for g in G:
            //          for o in obstacles:
            //             if dist(g, o) < 2r_r (=diameter):
            //                G += o
            //       if len(G) == Gsize: # size has not increased
            //          break

            // Use G_set to build a set. When done, add elements from set to G.
            std::set<pp_obstacle_struct_t>::iterator itG;
            std::set<pp_obstacle_struct_t, posval_compare> G_set;
            G_set.insert(B[B_idx]);
            {
                MRA_LOG_DEBUG("cXYTrajectory#Step3", "");
                while (true)
                {
                    size_t G_size = G_set.size();

                    for (itG = G_set.begin(); itG != G_set.end(); ++itG)
                    {
                        for (size_t j = 0; j < obstacles.size(); j++)
                        {
                            // Compute distance between obstacle in G and all obstacles.
                            double dist = std::hypot(fabs(itG->location.x - obstacles[j].location.x), fabs(itG->location.y - obstacles[j].location.y));

                            //TRACE("Dist between (%6.2f,%6.2f) and (%6.2f,%6.2f) is %6.2f", itG->location.x, itG->location.y, obstacles[j].location.x, obstacles[j].location.y, dist);

                            // Distance is from center of obstacles.
                            // First subtract the radius from each obstacle, and see if the robot's diameter fits through the gap.
                            if ((dist - (config.robotRadius + obstacles[j].radius)) < config.groupGapDistance)
                            {
                                G_set.insert(obstacles[j]);
                            }
                        }
                    }

                    if (G_set.size() == G_size)
                    {
                        // Size has not increased -- break the while loop.
                        break;
                    }
                }
            }

            if (G_set.size() > obstacles.size())
            {
                MRA_LOG_ERROR("ERROR: G_set may not be larger than obstacles!");
            }
            MRA_LOG_DEBUG("Computed step 3. G_set.size() = %d", G_set.size());

            // Convert the set to a vector.
            std::vector<pp_obstacle_struct_t> G;
            for (itG = G_set.begin(); itG != G_set.end(); ++itG)
            {
                G.push_back(*itG);
                //TRACE("OBST IN G: X: %6.2f, Y: %6.2f", itG->location.x, itG->location.y);
            }

            // 4. Compute distance between line (robot -> target) and every object i in G:
            //    "The side to pass the group G is determined by selecting the side with the smallest absolute value of the perpendicular distance b_i for all i in G."
            //    b+_max = 0
            //    b-_max = 0
            //    for i in G:
            //       b_i = dist( line(robot, target), i )
            //       if b_i > 0:
            //          val = abs(b_i + r_r)
            //          if val > b+_max:
            //             b+_max = val
            //       else:
            //          val = abs(b_i - r_r)
            //          if val > b-_max:
            //             b-_max = val
            //    if b+_max >= b-_max:
            //       # left side of G
            //    else:
            //       # right side of G
            double bp_max = 0.0;
            double bm_max = 0.0;
            std::vector<pp_obstacle_struct_t> G_plus;
            std::vector<pp_obstacle_struct_t> G_min;
            std::vector<pp_obstacle_struct_t> G_side_to_use;
            {
                MRA_LOG_DEBUG("cXYTrajectory#Step4", "");
                for (int i = 0; i < (int)G.size(); i++)
                {
                    // Determine if this obstacle belongs in the positive group or negative group.
                    if (G[i].b_i > 0)
                    {
                        G_plus.push_back(G[i]);

                        // Equation (6a)
                        double val = fabs(G[i].b_i + G[i].radius);
                        if (val > bp_max)
                        {
                            bp_max = val;
                        }
                    }
                    else
                    {
                        G_min.push_back(G[i]);

                        // Equation (6b)
                        double val = fabs(G[i].b_i - G[i].radius);
                        if (val > bm_max)
                        {
                            bm_max = val;
                        }
                    }
                }

                //TRACE("b+_max = %6.2f , b-_max = %6.2f", bp_max, bm_max);

                // Equation (6c)
                if (bp_max >= bm_max)
                {
                    // go for minus side of G
                    G_side_to_use = G_min;
                    //TRACE("GOING FOR MINUS SIDE OF G");
                }
                else
                {
                    // go for plus side of G
                    G_side_to_use = G_plus;
                    //TRACE("GOING FOR PLUS SIDE OF G");
                }

                // Always consider obstacles on the line robot -> target
                for (size_t i = 0; i < B.size(); i++)
                {
                    G_side_to_use.push_back(B[i]);
                }
            }
            MRA_LOG_DEBUG("Computed step 4.");

            // 5. Compute for each i \in G the angle alpha. Save the i with highest angle -> alpha_max.
            // Determine "j" below equation (8) -> G_idx
            double alpha_max = 0.0;
            double alpha_j = 0.0;
            int G_idx = -1;

            {
                MRA_LOG_DEBUG("cXYTrajectory#Step5", "");
                for (size_t i = 0; i < G_side_to_use.size(); i++)
                {
                    // Determine the sign from equation (7)
                    double sign = 0.0;
                    if ((bm_max - bp_max) < 0)
                        sign = -1.0;
                    else
                        sign = 1.0;

                    // Compute the first and second term of equation (7)
                    double first_term = atan(G_side_to_use[i].b_i / G_side_to_use[i].a_i);
                    double distBetweenObstAndRobot = (MRA::Geometry::Point(G_side_to_use[i].location.x, G_side_to_use[i].location.y)-curPosXY).size();
                    double second_term = sign*asin( fmax( fmin( config.subTargetDistance/distBetweenObstAndRobot, 0.99), -0.99) ); // asin's argument must be in [-1 < x < 1]
                    //TRACE("first_term: %6.4f, second_term: %6.4f, sign: %6.4f, distBetweenObstAndRobot: %6.4f", first_term, second_term, sign, distBetweenObstAndRobot);

                    // Equation (7)
                    double alpha = first_term + second_term;
                    double abs_alpha = fabs(alpha);

                    if (abs_alpha > alpha_max)
                    {
                        alpha_max = abs_alpha;
                        alpha_j = alpha;
                        G_idx = i;
                    }

                    //TRACE("(%6.2f,%6.2f) angle is %6.2f", G_side_to_use[i].location.x, G_side_to_use[i].location.y, alpha);
                }
            }

            MRA_LOG_DEBUG("Computed step 5.");

            if (G_idx != -1)
            {
                MRA_LOG_DEBUG("cXYTrajectory#Step6", "");
                // Equation (8) -- G_idx == j
                MRA::Geometry::Point rightPart = ((targetXY - curPosXY) / (targetXY - curPosXY).size()) * (MRA::Geometry::Point(G_side_to_use[G_idx].location.x, G_side_to_use[G_idx].location.y) - curPosXY).size();
                MRA::Geometry::Point multipliedMatrix = MRA::Geometry::Point( ( (cos(alpha_j)*rightPart.x) - (sin(alpha_j)*rightPart.y) ) , ( (sin(alpha_j)*rightPart.x) + (cos(alpha_j)*rightPart.y) ) );
                MRA::Geometry::Point subtarget(curPosXY.x + multipliedMatrix.x, curPosXY.y + multipliedMatrix.y);

                //TRACE("subtarget pre-extend: x %6.2f, y %6.2f", subtarget.x, subtarget.y);
                //TRACE("currPos pre-extend: x %6.2f, y %6.2f", currPos.x, currPos.y);

                // Put the subtarget further away to prevent the robot from slowing down for the subtarget
                MRA::Geometry::Point diffSubTargetCurPos(subtarget - curPosXY);
                diffSubTargetCurPos.normalize();
                subtarget = subtarget + (diffSubTargetCurPos * config.subTargetExtensionFactor);
                // ROADMAP: replace this trick with velocity control making use of target.velocity

                //TRACE("subtarget post-extend: x %6.2f, y %6.2f", subtarget.x, subtarget.y);

                subTarget.x = subtarget.x;
                subTarget.y = subtarget.y;
                calculatedSubTarget = true;
            }

            MRA_LOG_DEBUG("All done.");
        }
    }

    // done
    if (calculatedSubTarget)
    {
        // add waypoint, leave original target orientation and velocity intact
        data.insertSubTarget(MRA::Geometry::Position(subTarget.x, subTarget.y, data.path.at(0).pos.rz));
        MRA_LOG_DEBUG("Algorithm done, inserted subTarget (%6.2f, %6.2f)", subTarget.x, subTarget.y);
    }
    else
    {
        MRA_LOG_DEBUG("Algorithm done, no obstacle avoidance needed");
    }
}

