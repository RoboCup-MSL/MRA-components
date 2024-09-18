/*
 * TeamPlannerGrid.cpp
 *
 *  Created on: Jan 3, 2016
 *      Author: jurge
 */

#include "TeamPlannerGrid.hpp"
#include "FieldConfig.hpp"
#include "PlannerGridInfoData.hpp"
#include "GridHeuristic.hpp"
#include "MathUtils.hpp"
#include "logging.hpp"
#include "TeamPlay.hpp"

#include <limits>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <list>
#include <iomanip>

using namespace std;

namespace MRA {


Geometry::Position TeamPlanner_Grid::findBallPlayerPosition(const TeamPlannerData& r_teamplannerData, int gridFileNumber) {
    const double infield_margin = 0.25;   // distance to stay from side of field
    const unsigned nr_positions_first_circle = 8;
    const double dist_first_circle = 1.0;
    const unsigned nr_positions_second_circle = 16;
    const double dist_second_circle = 2.25;

    double xc = r_teamplannerData.ball_pickup_position.x;
    double yc = r_teamplannerData.ball_pickup_position.y;

    Geometry::Position bestPosition = Geometry::Position(xc, yc);  // default : set best to pickup position

    std::list<Geometry::Position> allowedTargetPositions = list<Geometry::Position>();
    allowedTargetPositions.push_back(Geometry::Position(xc, yc));
    // create possible target positions:

    Geometry::Position ballPlayerPos = bestPosition; // set default to pickup position
    bool ballPlayerFound = false;
    // add current position for ball-player
    for (unsigned r_idx = 0; r_idx < r_teamplannerData.team.size(); r_idx++) {
        if (r_teamplannerData.team[r_idx].controlBall) {
            ballPlayerPos = r_teamplannerData.team[r_idx].position;
            allowedTargetPositions.push_back(ballPlayerPos); // ballplayer current position
            ballPlayerFound = true;
        }
    }
    // pickup position and 2 circles around it.
    for (unsigned i = 0; i < nr_positions_first_circle; i++) {
        double angle = i * 2.0 * M_PI / static_cast<double>(nr_positions_first_circle);
        double xx = xc + dist_first_circle * cos(angle);
        double yy = yc + dist_first_circle * sin(angle);
        if (r_teamplannerData.fieldConfig.isInField(xx,yy,infield_margin)) {
            allowedTargetPositions.push_back(Geometry::Position(xx, yy));
        }

    }
    for (unsigned i = 0; i < nr_positions_second_circle; i++) {
        double angle = i * 2.0 * M_PI / static_cast<double>(nr_positions_second_circle);
        double xx = xc + dist_second_circle * cos(angle);
        double yy = yc + dist_second_circle * sin(angle);
        if (r_teamplannerData.fieldConfig.isInField(xx,yy,infield_margin)) {
            allowedTargetPositions.push_back(Geometry::Position(xx, yy));
        }
    }

    // evaluate all possible target positions
    // FOR all points
    //  - far from opponent is better
    //  - close to own player is better
    //  - 1 circle is better than 2nd circle
    //

    Geometry::Position bestPosForGoal = Geometry::Position(0, (r_teamplannerData.fieldConfig.getMaxFieldY())-2.0); // 2 meters before goal
    PlannerGridInfoData pgid = PlannerGridInfoData();
    vector<GridHeuristic *> heuristics = vector<GridHeuristic *>();
    if (false) {
        heuristics.push_back(new DistanceToHeuristic("Distance to best pos before goal", 1.0, pgid, bestPosForGoal, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance()));
        heuristics.push_back(new InfluenceOpponentsHeuristic("Influence Opponent", 20.0, pgid, r_teamplannerData.opponents, 0.7));
    }
    else {
        //    Shoot on goal
        double shootOnGoalFactor = 100;
        double passFactor = 10;
        // Pass is required OR
        //    -- time in possession goal success factor: 0 to 2 sec. x 1.0, from 2.0 to 5.0 seconds x0.5 , after 5.0 seconds: 1.0x (force shot on goal)
        if (r_teamplannerData.passIsRequired  || (r_teamplannerData.ball_pickup_position.ts > 2.0 && r_teamplannerData.ball_pickup_position.ts < 5.0)) {
            shootOnGoalFactor = 10;
            passFactor = 100;
        }

        if (ballPlayerFound) {
            // prefer driving too much, make current position is slightly preferred
            heuristics.push_back(new DistanceToPointHeuristic("Dist to current position", 0.1, pgid,
                    ballPlayerPos, 1.0 /*scaling*/, 5.0 /* max range*/, false /*inverted*/));
        }
        heuristics.push_back(new ShootOnGoalHeuristic("Shoot on goal", shootOnGoalFactor, pgid,
                r_teamplannerData.team, r_teamplannerData.opponents, r_teamplannerData.fieldConfig, r_teamplannerData.ball_pickup_position));
        heuristics.push_back(new PassHeuristic("passing", passFactor, pgid,
                r_teamplannerData.team, r_teamplannerData.opponents, r_teamplannerData.fieldConfig, r_teamplannerData.ball_pickup_position, r_teamplannerData.parameters));
        // prefer keep 2 meter distance from opponent
        heuristics.push_back(new StayAwayFromOpponentsHeuristic("Stay away from opponents",100,
                pgid, ballPlayerPos, r_teamplannerData.ball.position, r_teamplannerData.opponents, 2.0));

        //
        //    Passing
        //    -- multiply probability success factor : time 0-5 seconds. 1-(0.2 x seconds since ball possession)
        //    -- interception thread per player: normalize.

    }
    bestPosition = calculateGridValues(allowedTargetPositions, heuristics, r_teamplannerData.parameters, pgid);
    TeamPlanner_Grid::writeGridDataToFile(pgid, r_teamplannerData, r_teamplannerData.ball.position, "ShootPosition", gridFileNumber);

    return bestPosition;
}

void TeamPlanner_Grid::handle_penalty_heuristics(const TeamPlannerData& r_teamplannerData, const Geometry::Point& r_ballPositionToUse, vector<GridHeuristic*> &heuristics, PlannerGridInfoData &pgid)
{
    // Handle penalty related heuristics (used for multiple roles)

    if (r_teamplannerData.gamestate == game_state_e::PENALTY || r_teamplannerData.gamestate == game_state_e::PENALTY_AGAINST) {
        // penalty needs minimal distance to ball of ca 3.5 m (minimal 3 meter in robocup rules)
        auto ball_penalty = r_teamplannerData.parameters.grid_close_to_ball_restart_penalty_penalty; // default
        auto ball_radius = r_teamplannerData.parameters.grid_close_to_ball_restart_penalty_radius;
        heuristics.push_back(new InfluenceBallHeuristic("InfluenceBall", ball_penalty, pgid, r_ballPositionToUse.x, r_ballPositionToUse.y, ball_radius));
    }

    if (r_teamplannerData.gamestate == game_state_e::PENALTY_AGAINST) {
        // Not allow to be in own penalty area (RoboCup rules)
        heuristics.push_back(new InOwnPenaltyAreaHeuristic("InOwnPenalyArea", 100000, pgid, r_teamplannerData));

        // prefer position close to the backline: Y halfway between goal area en penalty area
        auto desired_y = -r_teamplannerData.fieldConfig.getMaxFieldY() + r_teamplannerData.fieldConfig.getGoalAreaLength() + 0.5 * (r_teamplannerData.fieldConfig.getPenaltyAreaLength() - r_teamplannerData.fieldConfig.getGoalAreaLength());
        heuristics.push_back(new DistanceToLineHeuristic("DesiredY", 20, pgid,
                -r_teamplannerData.fieldConfig.getMaxFullFieldX(), desired_y, r_teamplannerData.fieldConfig.getMaxFullFieldX(), desired_y, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance()));

        // Do not position close to already assigned team-mate position: avoid to be close to eachother
        heuristics.push_back(new CollideTeamMateHeuristic("CollideTeamMate", 2000.0, pgid, r_teamplannerData, 1.5));

        // Prefer position close too the ball
        heuristics.push_back(new DistanceToPointHeuristic("Close to ball", 200, pgid,
                                                          Geometry::Position(r_ballPositionToUse.x, r_ballPositionToUse.y), 8.0, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance(), false));

    }
    if (r_teamplannerData.gamestate == game_state_e::PENALTY) {
        // Not allow to be in opponent penalty area (RoboCup rules)
        heuristics.push_back(new InOppenentPenaltyAreaHeuristic("InOpponentPenalyArea", 100000, pgid, r_teamplannerData));
    }
}

/**
 * Method to find to most attractive position to defend an given opponent during setplay against.
 * A grid is created, for all points on the grid a heuristic (attractiveness) is calculated.
 * Most attractive position will be the position
 */
Geometry::Position TeamPlanner_Grid::findManToManDefensivePosition(dynamic_role_e dynamic_role, const Geometry::Point& oppentToDefend, const TeamPlannerData& r_teamplannerData, int gridFileNumber, bool setPlayActive)
{
    // define grid of 50 cm, only in field not outside the border
    Geometry::Position ballPos = r_teamplannerData.ball.position;
    double x_pos_ball = ballPos.x;
    double y_pos_ball = ballPos.y;


    PlannerGridInfoData pgid = PlannerGridInfoData();
    vector<GridHeuristic *> heuristics = vector<GridHeuristic *>();

    /*
     * Defender position heuristics in open-play (offensive and defensive)
     */
    double alfa = oppentToDefend.angle(ballPos);
    if (oppentToDefend.distanceTo(ballPos) < r_teamplannerData.parameters.setplay_against_dist_to_opponent + 0.25) {
        // player can not be positioned between ball and opponent.
        // prefer place between opponent and own goal.
        alfa = oppentToDefend.angle(r_teamplannerData.fieldConfig.getOwnGoal());
    }

    double preferred_x = oppentToDefend.x - (cos(alfa) * r_teamplannerData.parameters.setplay_against_dist_to_opponent);
    double preferred_y = oppentToDefend.y - (sin(alfa) * r_teamplannerData.parameters.setplay_against_dist_to_opponent);


    // Defend opponents on own half or close to own half
    // Preferred position approx 1 meter from opponent, with a preference with the defender to the most forward defender (smallest Y)
    //heuristics.push_back(new InCircleHeuristic("ShieldOpponent", 100.0, pgid, oppentToDefend.x, oppentToDefend.y, 1.5, true)); // invert=true, so in circle attracts

    // Positioned on the line between opponent and ball (so opponent cannot be reached)
    heuristics.push_back(new OnLineBetweenPointsHeuristic("OnLineBetweenBallAndOpponent", 500.0, pgid, x_pos_ball, y_pos_ball, preferred_x, preferred_y, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance()));

    // Play close to any opponent but not on exactly on top of it
    heuristics.push_back(new InfluenceOpponentsHeuristic("Influence Opponents", 800, pgid, r_teamplannerData.opponents, 1.0));

    // Play close to the opponent to defend but not on exactly on top of it
    heuristics.push_back(new DistanceToPointHeuristic("Opponent to defend", 200, pgid, Geometry::Position(preferred_x, preferred_y), 8.0, 8.0, false));

    // Do not position close to team-mate path position, avoid double defense if other opponents are available
    heuristics.push_back(new CollideTeamMateHeuristic("CollideTeamMate", 1000, pgid, r_teamplannerData, 0.7));

    // Do not plan in opponent penalty area
    heuristics.push_back(new InOppenentPenaltyAreaHeuristic("InOppenentPenalyArea", 1000.0, pgid, r_teamplannerData));

    // Do not plan in own goal area
    heuristics.push_back(new InOwnGoalAreaHeuristic("InOwnGoalArea", 1000.0, pgid, r_teamplannerData));

    // Do not plan in own penalty area if already somebody assigned
    heuristics.push_back(new AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic("Already player assigned to own penalty area", 1000.0, pgid, r_teamplannerData));

    // This still needed?
    heuristics.push_back(new OutsidePlayFieldHeuristic("OutsidePlayField", 1000.0, pgid, r_teamplannerData.fieldConfig, 0.0));

    // Stability heuristic
    heuristics.push_back(new InfluencePreviousAssignedPositionsHeuristic("previous assigned position", 150.0, pgid, r_teamplannerData, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance(), dynamic_role));

    /*
     * Additional defender position heuristics in set-play (various)
     * Normal defender can be deployed during a set-play with some additional positioning restrictions below
     * Heuristic function itself checks on correct game-state to identify set-play
     */

    // parameters to get a minimal distance to the ball (used function is a parabola).
    double ball_penalty = r_teamplannerData.parameters.grid_close_to_ball_normal_penalty; // default
    double ball_radius = r_teamplannerData.parameters.grid_close_to_ball_normal_radius;

    //stay out of 3 meter zone around ball in case of e.g. free-kick_against; to avoid touching the ball
    if (isOneOf(r_teamplannerData.gamestate, {FREEKICK_AGAINST, GOALKICK_AGAINST, GOALKICK, FREEKICK, CORNER_AGAINST, THROWIN_AGAINST}))
    {
        ball_penalty = r_teamplannerData.parameters.grid_close_to_ball_restart_penalty_penalty; // default
        ball_radius = r_teamplannerData.parameters.grid_close_to_ball_restart_penalty_radius;
        heuristics.push_back(new BallSetplayAgainstHeuristic("InfluenceBallSetplayAgainst", ball_penalty, pgid, x_pos_ball, y_pos_ball, ball_radius, r_teamplannerData.fieldConfig));

        double dist_ball_to_opponent = oppentToDefend.distanceTo(ballPos);
        if (dist_ball_to_opponent <= ball_radius && y_pos_ball < 0) {
            // opponent is in the forbidden area around the ball and ball is at our half.
            // locate player on height at the half of the distance between ball and the opponent
            double desired_y = y_pos_ball;
            if (y_pos_ball < oppentToDefend.y) {
                desired_y -= dist_ball_to_opponent * 0.5; // subtract offset wrt ball-y
            }
            else {
                desired_y += dist_ball_to_opponent * 0.5; // add the offset wrt ball-y
            }
            heuristics.push_back(new DistanceToLineHeuristic("DesiredY", 2000, pgid,
                    -r_teamplannerData.fieldConfig.getMaxFullFieldX(), desired_y, r_teamplannerData.fieldConfig.getMaxFullFieldX(), desired_y, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance()));
        }
    }

    // apply penalty heuristics if needed
    handle_penalty_heuristics(r_teamplannerData, r_teamplannerData.ball.position, heuristics, pgid);

    if (r_teamplannerData.teamControlsBall()) {
        // avoid defender being to close to the ball: add penalty for being close to the ball
        heuristics.push_back(new InfluenceBallHeuristic("InfluenceBall", 5000, pgid, x_pos_ball, y_pos_ball, 2.0 /* [m] */));
    }


    double grid_size = r_teamplannerData.parameters.grid_size * 0.5; //[m]   // times denser than normal with reduce grid
    double x_radius = 8.0;        // limit number of calculations in x (max 2*x_radius)
    double x_steps = floor(x_radius / grid_size);
    double min_x = preferred_x - (x_steps * grid_size);
    double max_x = preferred_x + (x_steps * grid_size);
    double y_radius = 5.0; // limit number of calculations in y (max 2*y_radius)
    double y_steps = floor(y_radius / grid_size);
    double min_y = preferred_y - (y_steps * grid_size);
    double max_y = preferred_y + (y_steps * grid_size);

    std::list<Geometry::Position> allowedTargetPositions = list<Geometry::Position>();

    // create x,y grid for calculation. with prefered x , y in center of grid !!
    for (double x = min_x; x <= (max_x + 0.01); x += grid_size) {
        for (double y = min_y; y <= (max_y + 0.01); y += grid_size) {
            if (r_teamplannerData.fieldConfig.isInField(x, y, 0))
            { // only position where player is inside the field
                allowedTargetPositions.push_back(Geometry::Position(x, y));
            }
        }
    }
    Geometry::Position bestPosition = calculateGridValues(allowedTargetPositions, heuristics, r_teamplannerData.parameters, pgid);
    TeamPlanner_Grid::writeGridDataToFile(pgid, r_teamplannerData, r_teamplannerData.ball.position, "DefensivePosition", gridFileNumber);

    return bestPosition;


}

/**
 * Method to find to most attractive position for zone defense support.
 * A grid is created, for all points on the grid a heuristic (attractiveness) is calculated.
 * Most attractive position will be the offensive position
 */
Geometry::Position TeamPlanner_Grid::findDefensivePosition(const TeamPlannerData& r_teamplannerData, int gridFileNumber)
{

     /* Position sweeper, in order of priority (weighing):
         * Marks/shadows opponents "mandekking" of Y< 2 ( with a preference with the defender to the most forward defender (smallest Y). Preferred position approx 1 meter from opponent.
         * Positioned on the line between opponent and ball (so opponent cannot be reached)
         * If an another robot is already covering a robot ((eg interceptor, defender, etc) pick the other opponent if available; in practice: position not to close to team-mate planned earlier
         * If there are no opponents on our half defend on -4.5 M line (halfway own half)
         * Only one player is allowed in the penalty area (outer box of penalty), if other higher priority role (e.g. interceptor) is already in, do not enter
         * Stability factor: do some minor preference to current location to avoid constant ¨flipping" between 2 almost equal positions
         * Do not plan outside playing field or in goal area (inner box of penalty area)
         *  Stay away out of corners of the field, not the best defensive position
         */

    // define grid of 50 cm, only in field not outside the border
    double grid_size = r_teamplannerData.parameters.grid_size; //[m]
    double x_grid_half = floor(r_teamplannerData.fieldConfig.getMaxFieldX()/grid_size);
    double y_grid_half = floor(r_teamplannerData.fieldConfig.getMaxFieldY()/grid_size);
    int x_grid_points = 1 + 2 * x_grid_half;
    int y_grid_points = 1 + 2 * y_grid_half;

    double x_pos_ball = r_teamplannerData.ball.position.x;
    double y_pos_ball = r_teamplannerData.ball.position.y;


    PlannerGridInfoData pgid = PlannerGridInfoData();
    vector<GridHeuristic *> heuristics = vector<GridHeuristic *>();

    /*
     * Defender position heuristics in open-play (offensive and defensive)
     */


    // Defend opponents on own half or close to own half
    for (unsigned op_idx = 0; op_idx < r_teamplannerData.opponents.size(); op_idx++) {
        TeamPlannerOpponent opponent = r_teamplannerData.opponents[op_idx];
        Geometry::Position opponentPos = opponent.position;
        if (opponentPos.y < 2) {

            // Preferred position approx 1 meter from opponent, with a preference with the defender to the most forward defender (smallest Y)
            heuristics.push_back(new InCircleHeuristic("ShieldOpponent", 100.0 - (opponentPos.y * 2) , pgid, opponentPos.x, opponentPos.y, 1.5, true)); // invert=true, so in circle attracts

            // Positioned on the line between opponent and ball (so opponent cannot be reached)
            heuristics.push_back(new OnLineBetweenPointsHeuristic("OnLineBetweenBallAndOpponent", 30.0, pgid, x_pos_ball, y_pos_ball, opponentPos.x, opponentPos.y, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance()));
        }

    }

    // Play close to opponent but not on exactly on top of it
    heuristics.push_back(new InfluenceOpponentsHeuristic("Influence Opponent", 200.0, pgid, r_teamplannerData.opponents, 0.7));

    // Do not position close to team-mate path position, avoid double defense if other opponents are available
    heuristics.push_back(new CollideTeamMateHeuristic("CollideTeamMate", 50.0, pgid, r_teamplannerData, 2.5));

    // Defend on -4.5 M line (halfway own half), lower priority so it only takes as the position if there are no opponents to defend
    heuristics.push_back(new DesiredY("DesiredYDefender", 20.0, pgid, -r_teamplannerData.fieldConfig.getMaxFieldY()*0.5, r_teamplannerData.fieldConfig));

    // Stay away out of corners of the field, not the best defensive position
    heuristics.push_back(new InfluenceCornerHeuristic("InfluenceCorner", 10.0, pgid, r_teamplannerData.fieldConfig));

    heuristics.push_back(new InfluencePreviousAssignedPositionsHeuristic("previous assigned position", 100.0, pgid, r_teamplannerData, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance(), dynamic_role_e::dr_DEFENDER));


    // Do not plan in opponent penalty area
    heuristics.push_back(new InOppenentPenaltyAreaHeuristic("InOppenentPenalyArea", 1000.0, pgid, r_teamplannerData));

    // Do not plan in own goal area
    heuristics.push_back(new InOwnGoalAreaHeuristic("InOwnGoalArea", 1000.0, pgid, r_teamplannerData));

    // Do not plan in own penalty area if already somebody assigned
    heuristics.push_back(new AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic("Already player assigned to own penalty area", 1000.0, pgid, r_teamplannerData));

    // This still needed?
    heuristics.push_back(new OutsidePlayFieldHeuristic("OutsidePlayField", 1000.0, pgid, r_teamplannerData.fieldConfig, 0.0));

    // Stability heuristic
    heuristics.push_back(new InfluenceCurrentPositionsHeuristic("InfluenceCurrentPositions", 8.0, pgid, r_teamplannerData, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance()));

    /*
     * Additional defender position heuristics in set-play (various)
     * Normal defender can be deployed during a set-play with some additional positioning restrictions below
     * Heuristic function itself checks on correct game-state to identify set-play
     */

    // parameters to get a minimal distance to the ball (used function is a parabola).
    double ball_penalty = r_teamplannerData.parameters.grid_close_to_ball_normal_penalty; // default
    double ball_radius = r_teamplannerData.parameters.grid_close_to_ball_normal_radius;

    // todo below if-then needs to replaced by structured case

    //stay out of 3 meter zone around ball in case of e.g. free-kick_against; to avoid touching the ball
    if (isOneOf(r_teamplannerData.gamestate, {FREEKICK_AGAINST, GOALKICK_AGAINST, GOALKICK, FREEKICK, CORNER_AGAINST, THROWIN_AGAINST})) {
        ball_penalty = r_teamplannerData.parameters.grid_close_to_ball_restart_penalty_penalty; // default
        ball_radius = r_teamplannerData.parameters.grid_close_to_ball_restart_penalty_radius;
        heuristics.push_back(new InfluenceBallHeuristic("InfluenceBall", ball_penalty, pgid, x_pos_ball, y_pos_ball, ball_radius));
    }

    // apply penalty heuristics if needed
    handle_penalty_heuristics(r_teamplannerData, r_teamplannerData.ball.position, heuristics, pgid);

    std::list<Geometry::Position> allowedTargetPositions = list<Geometry::Position>();

    // create x,y grid for calculation. make 0,0 center of grid !!
    for (int x_idx  = 0; x_idx < x_grid_points; x_idx++) {
        for (int y_idx  = 0; y_idx < y_grid_points; y_idx++) {
            double x = (x_idx * grid_size) - (x_grid_half * grid_size);
            double y = (y_idx * grid_size) - (y_grid_half * grid_size);
            if (y > -r_teamplannerData.fieldConfig.getMaxFieldY() + r_teamplannerData.fieldConfig.getRobotRadius()) { // only position where player is not over own backline
                allowedTargetPositions.push_back(Geometry::Position(x, y));
            }
        }
    }
    Geometry::Position bestPosition = calculateGridValues(allowedTargetPositions, heuristics, r_teamplannerData.parameters, pgid);
    TeamPlanner_Grid::writeGridDataToFile(pgid, r_teamplannerData, r_teamplannerData.ball.position, "DefensivePosition", gridFileNumber);

    return bestPosition;
}


/**
 * Method to find to most attractive position for zone defense support.
 * A grid is created, for all points on the grid a heuristic (attractiveness) is calculated.
 * Most attractive position will be the offensive position
 */
Geometry::Position TeamPlanner_Grid::findDefensivePositionDuringPenaltyShootOut(const TeamPlannerData& r_teamplannerData, int gridFileNumber)
{
    double field_direction = -1.0;
    if (r_teamplannerData.gamestate == PENALTY_SHOOTOUT_AGAINST) {
        field_direction = 1.0;
    }


    /* Position sweeper, in order of priority (weighing):
         * Marks/shadows opponents "mandekking" of Y< 2 ( with a preference with the defender to the most forward defender (smallest Y). Preferred position approx 1 meter from opponent.
         * Positioned on the line between opponent and ball (so opponent cannot be reached)
         * If an another robot is already covering a robot ((eg interceptor, defender, etc) pick the other opponent if available; in practice: position not to close to team-mate planned earlier
         * If there are no opponents on our half defend on -4.5 M line (halfway own half)
         * Only one player is allowed in the penalty area (outer box of penalty), if other higher priority role (e.g. interceptor) is already in, do not enter
         * Stability factor: do some minor preference to current location to avoid constant ¨flipping" between 2 almost equal positions
         * Do not plan outside playing field or in goal area (inner box of penalty area)
         *  Stay away out of corners of the field, not the best defensive position
         */

    // define grid of 50 cm, only in field not outside the border
    double grid_size = r_teamplannerData.parameters.grid_size; //[m]
    double x_grid_half = floor(r_teamplannerData.fieldConfig.getMaxFieldX()/grid_size);
    double y_grid_half = floor(r_teamplannerData.fieldConfig.getMaxFieldY()/grid_size);
    int x_grid_points = 1 + 2 * x_grid_half;
    int y_grid_points = 1 + 2 * y_grid_half;



    PlannerGridInfoData pgid = PlannerGridInfoData();
    vector<GridHeuristic *> heuristics = vector<GridHeuristic *>();

    // Play close to opponent but not on exactly on top of it
    heuristics.push_back(new InfluenceOpponentsHeuristic("Influence Opponent", 200.0, pgid, r_teamplannerData.opponents, 1.0));

    // Do not position close to team-mate path position, avoid double defense if other opponents are available
    heuristics.push_back(new CollideTeamMateHeuristic("CollideTeamMate", 200.0, pgid, r_teamplannerData, 1.0));

    // stay on  1 meter from center line (halfway own half), lower priority so it only takes as the position if there are no opponents to defend
    heuristics.push_back(new DesiredY("DesiredYDefender", 200.0, pgid, field_direction * 1.0, r_teamplannerData.fieldConfig));

    heuristics.push_back(new OutsidePlayFieldHeuristic("OutsidePlayField", 100.0, pgid, r_teamplannerData.fieldConfig, 0.0));

    // Stability heuristic
    heuristics.push_back(new InfluenceCurrentPositionsHeuristic("InfluenceCurrentPositions", 8.0, pgid, r_teamplannerData, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance()));

    heuristics.push_back(new InfluencePreviousAssignedPositionsHeuristic("previous assigned position", 100.0, pgid, r_teamplannerData, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance(), dynamic_role_e::dr_PENALTY_DEFENDER));

    //stay out of 3 meter zone around center of the field
    heuristics.push_back(new InfluenceBallHeuristic("Influence Center Circle", 1000.0, pgid, 0, 0, r_teamplannerData.fieldConfig.getCenterCirleDiameter() + 0.5));

    std::list<Geometry::Position> allowedTargetPositions = list<Geometry::Position>();

    // create x,y grid for calculation. make 0,0 center of grid !!
    for (int x_idx  = 0; x_idx < x_grid_points; x_idx++) {
        for (int y_idx  = 0; y_idx < y_grid_points; y_idx++) {
            double x = (x_idx * grid_size) - (x_grid_half * grid_size);
            double y = (y_idx * grid_size) - (y_grid_half * grid_size);
            if (y > -r_teamplannerData.fieldConfig.getMaxFieldY() + r_teamplannerData.fieldConfig.getRobotRadius()) { // only position where player is not over own backline
                allowedTargetPositions.push_back(Geometry::Position(x, y));
            }
        }
    }
    Geometry::Position bestPosition = calculateGridValues(allowedTargetPositions, heuristics, r_teamplannerData.parameters, pgid);
    TeamPlanner_Grid::writeGridDataToFile(pgid, r_teamplannerData, r_teamplannerData.ball.position, "DefensivePosition", gridFileNumber);

    return bestPosition;
}

//---------------------------------------------------------------------
/**
 * Method to find to most attractive position for sweeper role
 * A grid is created, for all points on the grid a heuristic (attractiveness) is calculated.
 * Sweeper role is typical used in defensive formations, it is the "Ausputzer" / "laatste man" / "lock on the door"
 * It normally positioned behind defenders to tackle any attacker bypassing defensive line
 */

Geometry::Position TeamPlanner_Grid::findSweeperPosition(const TeamPlannerData& r_teamplannerData, int gridFileNumber)
{
    /* Position sweeper, in order of priority (weighing):
     *   + on line between ball and own goal
     *   + The preferred Y location of the sweeper on the field
     *          if ball at opponent half (y > 0): put line A (y = -4.5 m (half field)
     *          elseif ball above own penalty area (y > ca - 6.5): put line B (top of own penalty area)
     *          elseif (y< -6.5) and x > width penalty area (ca 3.5): put line C on x = 3.5 (right side of own penalty area)
     *          elseif (y< -6.5) and x < width penalty area (ca 3.5): put line C on x = -3.5 (left side own penalty area)
     *          else (in penalty area): put line D on x = -8.25 (top of own goal area)
     *   + As a consequence sweeper stays always on own half (and only potential position on own half need to be evaluated
     *   + Do not position on top off team-mate (planned before sweeper, higher priority), e.g. goalkeeper, interceptor
     *   + Only one player is allowed in the penalty area (outer box of penalty), if other higher priority role (e.g. interceptor) is already in, do not enter
     *   + If positioned in own penalty area start pushing ball and potential opponents outside penalty area to make room for goalkeeper. (rule if robot is actively making progress to leave penalty area referee may extend 10 second time rule)
     *   + If a teammate is already positioned (e.g. goalkeeper) do not stand on exact same line to ball as teammate nut just a little left or right (no use of 2 robots exactly behind each other)
     *   + Stability factor: do some minor preference to current location to avoid constant ¨flipping" between 2 almost equal positions
     *   + Do not plan outside playing field or in goal area (inner box of penalty area)
     *   + Position of opponents does not determine position of sweeper, fights for position, push opponents away
     */

    // define grid, only in field not outside the border
    double grid_size = r_teamplannerData.parameters.grid_size; //[m]
    double x_grid_half = floor(r_teamplannerData.fieldConfig.getMaxFieldX()/grid_size);
    double y_grid_half = floor(r_teamplannerData.fieldConfig.getMaxFieldY()/grid_size);
    int x_grid_points = 1 + 2 * x_grid_half;
    int y_grid_points = 1 + y_grid_half;  // only half field;

    double x_pos_ball = r_teamplannerData.ball.position.x;
    double y_pos_ball = r_teamplannerData.ball.position.y;

    PlannerGridInfoData pgid = PlannerGridInfoData();
    vector<GridHeuristic *> heuristics = vector<GridHeuristic *>();

    /*
     * Sweeper position heuristics in open-play (offensive and defensive)
     */

    // on line between ball and own goal
    heuristics.push_back(new OnLineBetweenPointsHeuristic("OnLineBetweenBallAndOwnGoal", 100.0, pgid,
                x_pos_ball, y_pos_ball,
                0.0, -r_teamplannerData.fieldConfig.getMaxFieldY(),
                r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance()));

    // The preferred Y location of the sweeper on the field
    double topOwnPenaltyAreaY = -r_teamplannerData.fieldConfig.getMaxFieldY() + r_teamplannerData.fieldConfig.getPenaltyAreaLength() + r_teamplannerData.fieldConfig.getRobotRadius();
    double besidePenaltyAreaX = r_teamplannerData.fieldConfig.getPenaltyAreaWidth()*0.5 + r_teamplannerData.fieldConfig.getRobotRadius();
    double topOwnGoalAreaY = -r_teamplannerData.fieldConfig.getMaxFieldY() + r_teamplannerData.fieldConfig.getGoalAreaLength() + r_teamplannerData.fieldConfig.getRobotRadius();
    if (y_pos_ball >= 0) {
        heuristics.push_back(new DesiredY("DesiredYSweeper", 20.0, pgid, -r_teamplannerData.fieldConfig.getMaxFieldY()*0.5, r_teamplannerData.fieldConfig));
    }
    else if (y_pos_ball >= topOwnPenaltyAreaY) {
        heuristics.push_back(new DesiredY("DesiredYSweeper", 20.0, pgid, topOwnPenaltyAreaY, r_teamplannerData.fieldConfig));
    }
    else if (x_pos_ball >= besidePenaltyAreaX) {
        heuristics.push_back(new DesiredX("DesiredXSweeper", 20.0, pgid, besidePenaltyAreaX, r_teamplannerData.fieldConfig));
    }
    else if (x_pos_ball <= -besidePenaltyAreaX) {
        heuristics.push_back(new DesiredX("DesiredXSweeper", 20.0, pgid, -besidePenaltyAreaX, r_teamplannerData.fieldConfig));
    }
    else {
        // ball in penalty area, put sweeper just above goal area
        heuristics.push_back(new DesiredY("DesiredYSweeper", 20.0, pgid, topOwnGoalAreaY, r_teamplannerData.fieldConfig));

    }
    // Do not position on top off team-mate
    heuristics.push_back(new CollideTeamMateHeuristic("CollideTeamMate", 40.0, pgid, r_teamplannerData, r_teamplannerData.fieldConfig.getRobotSize()*2));

    // Only one player is allowed in the penalty area
    heuristics.push_back(new AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic("Already player assigned to own penalty area", 100000, pgid, r_teamplannerData)); // Do not enter the outer goal area if already an another player is in.

    // Do not plan outside playing field or in goal area
    heuristics.push_back(new InOwnGoalAreaHeuristic("InOwnGoalArea", 1000.0, pgid, r_teamplannerData)); // Do not enter the inner Goal area

    // Stability heuristic: 5 weight
    heuristics.push_back(new InfluenceCurrentPositionsHeuristic("InfluenceCurrentPositions", 5, pgid, r_teamplannerData, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance()));

    heuristics.push_back(new InfluencePreviousAssignedPositionsHeuristic("previous assigned position", 100.0, pgid, r_teamplannerData, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance(), dynamic_role_e::dr_SWEEPER));

    /*
     * Additional sweeper position heuristics in set-play (various)
     * Normal sweeper can be deployed during a set-play with some additional positioning restrictions below
     * Heuristic function itself checks on correct game-state to identify set-play
     */

    // parameters to get a minimal distance to the ball (used function is a parabola).
    double ball_penalty = r_teamplannerData.parameters.grid_close_to_ball_normal_penalty; // default
    double ball_radius = r_teamplannerData.parameters.grid_close_to_ball_normal_radius;

    // todo below if-then needs to replaced by structured case

    //stay out of 3 meter zone around ball in case of e.g. free-kick_against; to avoid touching the ball
    if (isOneOf(r_teamplannerData.gamestate, {FREEKICK_AGAINST, GOALKICK_AGAINST, GOALKICK, FREEKICK, CORNER_AGAINST, THROWIN_AGAINST}) ) {
        ball_penalty = r_teamplannerData.parameters.grid_close_to_ball_restart_penalty_penalty; // default
        ball_radius = r_teamplannerData.parameters.grid_close_to_ball_restart_penalty_radius;
        heuristics.push_back(new BallSetplayAgainstHeuristic("InfluenceBall", ball_penalty, pgid, x_pos_ball, y_pos_ball, ball_radius, r_teamplannerData.fieldConfig));
    }

    // apply penalty heuristics if needed
    handle_penalty_heuristics(r_teamplannerData, r_teamplannerData.ball.position, heuristics, pgid);

    std::list<Geometry::Position> allowedTargetPositions = list<Geometry::Position>();
    // create x,y grid for calculation. make 0,0 center of grid !!
    for (int x_idx  = 0; x_idx < x_grid_points; x_idx++) {
        for (int y_idx  = 0; y_idx < y_grid_points; y_idx++) {
            double x = (x_idx * grid_size) - (x_grid_half * grid_size);
            double y = (y_idx * grid_size) - (y_grid_half * grid_size);
            if (y > -r_teamplannerData.fieldConfig.getMaxFieldY() + r_teamplannerData.fieldConfig.getRobotRadius()) { // only position where player is not over own backline
                allowedTargetPositions.push_back(Geometry::Position(x, y));
            }
        }
    }
    Geometry::Position bestPosition = calculateGridValues(allowedTargetPositions, heuristics, r_teamplannerData.parameters, pgid);
    TeamPlanner_Grid::writeGridDataToFile(pgid, r_teamplannerData, r_teamplannerData.ball.position, "Sweeper", gridFileNumber);
    return bestPosition;
}

Geometry::Position TeamPlanner_Grid::calculateGridValues(const std::list<Geometry::Position>& allowedTargetPositions,
        vector<GridHeuristic*> heuristics, const TeamPlannerParameters& parameters, PlannerGridInfoData& pgid) {

    double lowest_value = std::numeric_limits<double>::infinity();
    double lowest_x = 0;
    double lowest_y = 0;
    unsigned cell_nr = 0;
    vector<griddata_t> gridData = vector<griddata_t>();

    // create x,y grid for calculation. make 0,0 center of grid !!
    for (std::list<Geometry::Position>::const_iterator pos_it=allowedTargetPositions.begin(); pos_it != allowedTargetPositions.end(); ++pos_it) {
        double tot_value = 0;
        double x = pos_it->x;
        double y = pos_it->y;
        vector<double> layerValues = vector<double>(); // TODO get size from layer info length
        for (auto it = heuristics.begin(); it != heuristics.end(); ++it) {
            double value = (*it)->getValue(x, y) * (*it)->getWeight();
            layerValues.push_back(value);
            tot_value += value;
        }
        // FIND LOWEST POINTS (can be multiple).
        // use this as target for path planning.
        if (tot_value < lowest_value) {
            lowest_value = tot_value;
            lowest_x = x;
            lowest_y = y;
        }
        if (parameters.saveGridDataToFile) {
            griddata_t data;
            data.x = x;
            data.y = y;
            data.z = tot_value;
            gridData.push_back(data);
            PlannerGridCell cell = PlannerGridCell(cell_nr, x, y,
                    layerValues);
            pgid.cells.push_back(cell);
        }
        cell_nr++;
    }

    // clear memory: release the pointers
    for (auto it = heuristics.begin(); it != heuristics.end(); it++) {
        delete (*it);
    }
    return Geometry::Position(lowest_x, lowest_y);
}

//---------------------------------------------------------------------
/**
 * Method to find to most attractive position for interceptor role during restart
 * A grid is created, for all points on the grid a heuristic (attractiveness) is calculated.
 */
Geometry::Position TeamPlanner_Grid::findInterceptorPositionDuringRestart(const TeamPlannerData& r_teamplannerData, int gridFileNumber)
{
    /* Position intercepter during restart, in order of priority (weighing):
     *   + During restart player of defending team cannot stay in circle with radius of 3 M around the ball until ball is in play
     *   + Position interceptor on the 3 M circle around the ball (as close as allowed to the ball)
     *   + Position interceptor as close as possible to opponent interceptor (located within the 3 meter circle)
     *   + Position interceptor/sweeper optimized when defending on own half (both leave room for each other)
     *   + Do not position on top off teammate (planned before interceptor, higher priority), e.g. goalkeeper.
     *   + Only one player is allowed in the penalty area (outer box of penalty), if other higher priority role (e.g. interceptor) is already in, do not enter
     *   + Stability factor: do some minor preference to current location to avoid constant ¨flipping" between 2 almost equal positions
     *   + Do not plan outside playing field or in keeper area (inner box of penalty area)
     */

    // define grid of 50 cm, only in field not outside the border
    double grid_size = r_teamplannerData.parameters.grid_size; //[m]
    double x_grid_half = floor(r_teamplannerData.fieldConfig.getMaxFieldX()/grid_size);
    double y_grid_half = floor(r_teamplannerData.fieldConfig.getMaxFieldY()/grid_size);
    int x_grid_points = 1 + 2 * x_grid_half;
    int y_grid_points = 1 + 2 * y_grid_half;

    double x_pos_ball = r_teamplannerData.ball.position.x;
    double y_pos_ball = r_teamplannerData.ball.position.y;

    // parameters to get a minimal distance to the ball (used function is a parabola).
    // TODO should be mixed to below, where is ball_penalty, etc used?

    double ball_penalty = r_teamplannerData.parameters.grid_close_to_ball_normal_penalty; // default
    double ball_radius = r_teamplannerData.parameters.grid_close_to_ball_normal_radius;
    //double ball_ax_sqr = calc_a_penalty_factor(2.0, ball_c); // default 2 meter  // TODO via parameters
    if (isOneOf(r_teamplannerData.gamestate, {FREEKICK_AGAINST, GOALKICK_AGAINST, CORNER_AGAINST, THROWIN_AGAINST}) ) {
        // hotfix! TODO: check this (penalty_penalty and penalty radius)
        ball_penalty = r_teamplannerData.parameters.grid_close_to_ball_restart_penalty_penalty; // default
        ball_radius = r_teamplannerData.parameters.grid_close_to_ball_restart_penalty_radius;
    }
    if (r_teamplannerData.gamestate == game_state_e::DROPPED_BALL) {
        ball_penalty = r_teamplannerData.parameters.grid_close_to_ball_restart_dropball_penalty; // default
        ball_radius = r_teamplannerData.parameters.grid_close_to_ball_restart_dropball_radius;
    }

    // store data in vector when parameters.saveGridDataToFile is enabled.
    PlannerGridInfoData pgid = PlannerGridInfoData();
    vector<GridHeuristic *> heuristics = vector<GridHeuristic *>();

    if (r_teamplannerData.gamestate != game_state_e::PENALTY && r_teamplannerData.gamestate != game_state_e::PENALTY_AGAINST) {
        //During restart player of defending team cannot stay in circle with radius of 3 M around the ball until ball is in play
        heuristics.push_back(new InfluenceBallHeuristic("InfluenceBall", ball_penalty, pgid, x_pos_ball, y_pos_ball, ball_radius));
    }

    // apply penalty heuristics if needed
    handle_penalty_heuristics(r_teamplannerData, r_teamplannerData.ball.position, heuristics, pgid);


    // Position interceptor on the 3 M circle around the ball (as close as allowed to the ball)
    heuristics.push_back(new DistanceToHeuristic("Distance to ball", 100.0, pgid, r_teamplannerData.ball.position, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance()));

    // Position interceptor as close as possible to opponent interceptor (located within the 3 meter circle)
    heuristics.push_back(new InfluencePreviousAssignedPositionsHeuristic("previous assigned position", 100.0, pgid, r_teamplannerData, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance(), dynamic_role_e::dr_INTERCEPTOR));


    // Position interceptor/sweeper optimized when defending on own half (both leave room for each other)
    if (y_pos_ball <= -7) {
        heuristics.push_back(new OnLineBetweenPointsHeuristic("NotOnLineBetweenBallAndOwnGoal", 20.0, pgid,
                        x_pos_ball, y_pos_ball,
                        0.0, 0.0,
                        r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance()));
        }
        else  {
            heuristics.push_back(new OnLineBetweenPointsHeuristic("NotOnLineBetweenBallAndOwnGoal", 20.0, pgid,
                                    x_pos_ball, y_pos_ball,
                                    0.0, -3.0,
                                    r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance()));
        }


    // Do not position on top off team-mate
    heuristics.push_back(new CollideTeamMateHeuristic("CollideTeamMate", 40.0, pgid, r_teamplannerData, r_teamplannerData.fieldConfig.getRobotSize()*2));

    // Do not plan outside playing field or in goal area
    heuristics.push_back(new InOwnGoalAreaHeuristic("InOwnGoalArea", 1000.0, pgid, r_teamplannerData)); // Do not enter the inner Goal area

    // Stability heuristic: 5 weight
    heuristics.push_back(new InfluenceCurrentPositionsHeuristic("InfluenceCurrentPositions", 5, pgid, r_teamplannerData, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance()));

    // DO not plan on top of opponent
    heuristics.push_back(new InfluenceOpponentsHeuristic("Influence Opponent", 5.0, pgid, r_teamplannerData.opponents, 0.7));


    heuristics.push_back(new InOppenentPenaltyAreaHeuristic("InOppenentPenalyArea", 100000, pgid, r_teamplannerData));
    heuristics.push_back(new InOwnGoalAreaHeuristic("Own goal area", 100000, pgid, r_teamplannerData));
    heuristics.push_back(new AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic("Already player assigned to own penalty area", 100000, pgid, r_teamplannerData));
    heuristics.push_back(new AlreadyPlayerAssignedToOpponentPenaltyAreaHeuristic("Already player assigned to opponent penalty area", 100000, pgid, r_teamplannerData));

    // create x,y grid for calculation. make 0,0 center of grid !!
    std::list<Geometry::Position> allowedTargetPositions = list<Geometry::Position>();
    for (int x_idx = 0; x_idx < x_grid_points; x_idx++) {
        for (int y_idx = 0; y_idx < y_grid_points; y_idx++) {
            double x = (x_idx * grid_size) - (x_grid_half * grid_size);
            double y = (y_idx * grid_size) - (y_grid_half * grid_size);
            if (y > -r_teamplannerData.fieldConfig.getMaxFieldY() + r_teamplannerData.fieldConfig.getRobotRadius()) { // only position where player is not over own backline
                allowedTargetPositions.push_back(Geometry::Position(x, y));
            }
        }
    }
    Geometry::Position pos = calculateGridValues(allowedTargetPositions, heuristics, r_teamplannerData.parameters, pgid);
    TeamPlanner_Grid::writeGridDataToFile(pgid, r_teamplannerData, r_teamplannerData.ball.position, "InterceptorRestart", gridFileNumber);
    return pos;
}

//---------------------------------------------------------------------
/**
 * Method to find to most attractive position for attack support (receiving ball) .
 * A grid is created, for all points on the grid a heuristic (attractiveness) is calculated.
 * Most attractive position will be the offensive position
 */
bool TeamPlanner_Grid::findAttackSupportPosition(Geometry::Point& bestPosition, const TeamPlannerData& r_teamplannerData, const Geometry::Point& r_ballPositionToUse, int gridFileNumber, bool position_close_to_ball)
{
    /* Position attack support, in order of priority (weighing):
     *
     *   + 100 Do not block goal shot on opponent goal
     *   + 80  Not too close to the ball
     *   + 50  Not too close to opponents
     *   + 100 previous assigned position
     *   + 50 stay in front of the ball on own half
     *   + 40/80 if we control the ball make yourself available to receive a pass
     *   + 30  Halfway on opponent half, good lobshot distance (also to have eyes on opponent half for ball detection)
     *   + 20  not behind opponent (from ball position)
     *   + 15  if ball on own half, stay on same side (left/right) as ball for better passing
     *   + 10  Do not position close to team-mate (planned before attack support, higher priority), e.g. goalkeeper, interceptor
     *   + 10  Do not position close to other attack_supporter(s)
     *   + 8   Stability factor: do some minor preference to current location to avoid constant ¨flipping" between 2 almost equal positions
     *   + 1000 Do not plan outside playing field or in goal areas (inner box of penalty area)
     *   + 1000 Do not plan in own penalty area if already somebody assigned
     */

    // define grid of 50 cm, only in field not outside the border
    double grid_size = r_teamplannerData.parameters.grid_size; //[m]
    double x_grid_half = floor(r_teamplannerData.fieldConfig.getMaxFieldX()/grid_size);
    double y_grid_half = floor(r_teamplannerData.fieldConfig.getMaxFieldY()/grid_size);
    int x_grid_points = 1 + 2 * x_grid_half;
    int y_grid_points = 1 + 2 * y_grid_half;

    double x_pos_ball = r_ballPositionToUse.x;
    double y_pos_ball = r_ballPositionToUse.y;

    PlannerGridInfoData pgid = PlannerGridInfoData();
    vector<GridHeuristic *> heuristics = vector<GridHeuristic *>();

    // Determine if this is the first attack supporter or not (second/third/etc)
    bool first_attack_supporter = true;
    for (unsigned idx = 0; idx < r_teamplannerData.team.size(); idx++) {
        if (r_teamplannerData.team_admin[idx].assigned  && r_teamplannerData.team_admin[idx].result.dynamic_role == dynamic_role_e::dr_ATTACKSUPPORTER) {
            first_attack_supporter= false;
        }
    };

    /*
     * Attack support position heuristics in open-play (offensive and defensive)
     */

    if (r_teamplannerData.gamestate != game_state_e::PENALTY && r_teamplannerData.gamestate != game_state_e::PENALTY_AGAINST) {
        // Do not block goal shot on opponent goal
        heuristics.push_back(new NotOnLineBetweenBallAndOpponentGoalHeuristic("Not between ball and opponent goal", 100.0, pgid,
                x_pos_ball, y_pos_ball, -r_teamplannerData.fieldConfig.getGoalAreaWidth()*0.5, r_teamplannerData.fieldConfig.getMaxFieldY(), +r_teamplannerData.fieldConfig.getGoalAreaWidth()*0.5, r_teamplannerData.fieldConfig.getMaxFieldY()));

          // Avoid difficult collaboration, not to close to the ball
        heuristics.push_back(new InfluenceBallHeuristic("InfluenceBall", 80.0, pgid, x_pos_ball, y_pos_ball, 4));
    }

    // apply penalty heuristics if needed
    handle_penalty_heuristics(r_teamplannerData, r_ballPositionToUse, heuristics, pgid);


    // Avoid difficult collaboration, do not position close to opponents
    heuristics.push_back(new InfluenceOpponentsHeuristic("Influence Opponent", 50.0, pgid, r_teamplannerData.opponents, 1));

    heuristics.push_back(new InfluencePreviousAssignedPositionsHeuristic("previous assigned position", 100.0, pgid, r_teamplannerData, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance(), dynamic_role_e::dr_ATTACKSUPPORTER));

    // stay in front of the ball on own half, so play forward on own half
    if (y_pos_ball < 0) {
        heuristics.push_back(new InSquareHeuristic("Play forward on own half", 50.0, pgid, -r_teamplannerData.fieldConfig.getMaxFieldX(), y_pos_ball+1, r_teamplannerData.fieldConfig.getMaxFieldX(), -r_teamplannerData.fieldConfig.getMaxFieldY()) );
    }
    else {
        heuristics.push_back(new InSquareHeuristic("Play on opponent half", 50.0, pgid, -r_teamplannerData.fieldConfig.getMaxFieldX(), 1, r_teamplannerData.fieldConfig.getMaxFieldX(), -r_teamplannerData.fieldConfig.getMaxFieldY()) );
    }

    // if we control the ball make yourself available to receive a pass, especially for the first attack supporter
    if (r_teamplannerData.teamControlsBall()) {
        if (first_attack_supporter) {
            heuristics.push_back(new InfluenceCurrentPositionsHeuristic("Receive pass when control ball", 80.0, pgid, r_teamplannerData, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance()));
        }
        else {
            heuristics.push_back(new InfluenceCurrentPositionsHeuristic("Receive pass when control ball", 40.0, pgid, r_teamplannerData, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance()));
        }
    }

    if (r_teamplannerData.gamestate != game_state_e::PENALTY && r_teamplannerData.gamestate != game_state_e::PENALTY_AGAINST) {
        //  position attack_support approx halfway on opponent half
        if (r_teamplannerData.gamestate != game_state_e::KICKOFF && r_teamplannerData.gamestate != game_state_e::KICKOFF_AGAINST) {
            // normally try to position on opponent half on a nice position to lob at the goal
            heuristics.push_back(new DesiredY("DesiredYAttackSupport", 30.0, pgid, (r_teamplannerData.fieldConfig.getMaxFieldY()*0.5)-1, r_teamplannerData.fieldConfig));
        }
        else {
            // in case of kick-off place robot at our own half
            heuristics.push_back(new DesiredY("DesiredYAttacksupport", 30.0, pgid, -1, r_teamplannerData.fieldConfig));
        }

        // Avoid difficult collaboration, not behind opponent (from ball position)
        heuristics.push_back(new InterceptionThreatHeuristic("Interception threat", 20.0, pgid, r_ballPositionToUse, r_teamplannerData));
    }



    // if ball is well on own half, stay on same side (left/right) as ball for better passing
    // else move to opposite side (left/right) to create room
    if (y_pos_ball < -1) {
        if (x_pos_ball < 0) {
            heuristics.push_back(new InSquareHeuristic("Left preference", 15.0, pgid, -2, r_teamplannerData.fieldConfig.getMaxFieldY(), r_teamplannerData.fieldConfig.getMaxFieldX(), -r_teamplannerData.fieldConfig.getMaxFieldY()) );
        }
        else {
            heuristics.push_back(new InSquareHeuristic("Right preference", 15.0, pgid, -r_teamplannerData.fieldConfig.getMaxFieldX(), r_teamplannerData.fieldConfig.getMaxFieldY(), 2, -r_teamplannerData.fieldConfig.getMaxFieldY()) );
        }
    }


    // Do not position close to team-mate path position
    heuristics.push_back(new CollideTeamMateHeuristic("CollideTeamMate", 10.0, pgid, r_teamplannerData, r_teamplannerData.fieldConfig.getRobotSize()*2));

    heuristics.push_back(new CollideTeamMateHeuristic("Distance to AttackSupporters", 50.0, pgid, r_teamplannerData, 4.0, true));


    // Do not plan in opponent penalty area
    heuristics.push_back(new InOppenentPenaltyAreaHeuristic("InOppenentPenalyArea", 1000.0, pgid, r_teamplannerData));
    // heuristics.push_back(new AlreadyPlayerAssignedToOpponentPenaltyAreaHeuristic("Already player assigned to opponent penalty area", 1000.0, pgid, Team, fieldConfig));

    // Do not plan in own goal area
    heuristics.push_back(new InOwnGoalAreaHeuristic("InOwnGoalArea", 1000.0, pgid, r_teamplannerData));

    // Do not plan in own penalty area if already somebody assigned
    heuristics.push_back(new AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic("Already player assigned to own penalty area", 1000.0, pgid, r_teamplannerData));

    // This still needed?
    heuristics.push_back(new OutsidePlayFieldHeuristic("OutsidePlayField", 1000.0, pgid, r_teamplannerData.fieldConfig, r_teamplannerData.parameters.attack_supporter_extra_distance_to_stay_from_sideline));

    // Stability heuristic
    heuristics.push_back(new InfluenceCurrentPositionsHeuristic("InfluenceCurrentPositions", 8.0, pgid, r_teamplannerData, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance()));


    /*
     * Additional attack support position heuristics in set-play (various)
     * Normal sweeper can be deployed during a set-play with some additional positioning restrictions below
     * Heuristic function itself checks on correct game-state to identify set-play
     */

    // create x,y grid for calculation. make 0,0 center of grid !!
    std::list<Geometry::Position> allowedTargetPositions = list<Geometry::Position>();
    for (int x_idx = 0; x_idx < x_grid_points; x_idx++) {
        for (int y_idx = 0; y_idx < y_grid_points; y_idx++) {
            double x = (x_idx * grid_size) - (x_grid_half * grid_size);
            double y = (y_idx * grid_size) - (y_grid_half * grid_size);
            if (y > -r_teamplannerData.fieldConfig.getMaxFieldY() + r_teamplannerData.fieldConfig.getRobotRadius()) { // only position where player is not over own backline
                allowedTargetPositions.push_back(Geometry::Position(x, y));
            }
        }
    }


    bestPosition = calculateGridValues(allowedTargetPositions, heuristics, r_teamplannerData.parameters, pgid);
    TeamPlanner_Grid::writeGridDataToFile(pgid, r_teamplannerData, r_ballPositionToUse, "FindOffensive", gridFileNumber);

    bool prepare_phase = isOneOf(r_teamplannerData.gamestate, {game_state_e::CORNER, game_state_e::GOALKICK, game_state_e::FREEKICK, game_state_e::THROWIN});
    if (r_teamplannerData.parameters.wait_on_non_optimal_position_during_prepare_phase && prepare_phase) {
        // find non optimal position to wait during prepare phase
        const unsigned nr_wait_positions = 8;   // check for 8 positions 1.5 meter around best position
        const double radius_non_opt_wait = 2.0;
        const double infield_margin = 0.25;   // distance to stay from side of field
        InterceptionThreatHeuristic InterceptionThreat = InterceptionThreatHeuristic("Interception threat", -18.0, pgid, r_ballPositionToUse, r_teamplannerData);

        double bestX = bestPosition.x;
        double bestY = bestPosition.y;
        double lowestValue = std::numeric_limits<double>::infinity();
        // for all positions to check
        for (unsigned i = 0; i < nr_wait_positions; i++) {
            double angle = i * 2.0 * M_PI / static_cast<double>(nr_wait_positions);
            double xx = bestPosition.x + radius_non_opt_wait * cos(angle);
            double yy = bestPosition.y + radius_non_opt_wait * sin(angle);
            if (r_teamplannerData.fieldConfig.isInField(xx,yy, infield_margin)) {
                // only check positions in the field
                double val = InterceptionThreat.getValue(xx, yy);
                  if (val < lowestValue) {
                      // value is worse than before, to it a good candidate to wait
                      bestX = xx;
                      bestY = yy;
                  }
            }
        }
        bestPosition.x = bestX;
        bestPosition.y = bestY;
    }

    return true;
}

// ----------------------------------------------------------
// Save provided data to file
void TeamPlanner_Grid::writeGridDataToFile(PlannerGridInfoData& pgid, const TeamPlannerData& r_teamplannerData, const Geometry::Point& r_ballPositionToUse, const std::string& strSituation, int gridFileNumber) {

    if (r_teamplannerData.parameters.saveGridDataToFile) {
        string gridFileName = "";
        for (auto it = r_teamplannerData.team.begin(); it != r_teamplannerData.team.end(); ++it) {
            pgid.gameData.team.push_back(it->position);
        }
        for (auto it = r_teamplannerData.opponents.begin(); it != r_teamplannerData.opponents.end(); ++it) {
            pgid.gameData.opponents.push_back(it->position);
        }
        pgid.gameData.ball = r_ballPositionToUse;
        if (!r_teamplannerData.parameters.svgOutputFileName.empty()) {
            // svg filename provided
            std::stringstream stream("");
            stream << r_teamplannerData.parameters.svgOutputFileName.substr(0, r_teamplannerData.parameters.svgOutputFileName.size()-4) << "_" << strSituation << "_" <<  gridFileNumber << ".gpd";
            gridFileName = stream.str();
        }
        else {
            std::stringstream stream("");
            stream << "planner_grid_" << strSituation << "_" <<  gridFileNumber << ".gpd";
            gridFileName = stream.str();
        }
        MRA_LOG_INFO("save grid data to = %s", gridFileName.c_str(), __FILE__, __LINE__);
        pgid.saveToFile(gridFileName);
    }
}

double TeamPlanner_Grid::calculate_a_penaly_factor_for_teammate(double ball_ax_sqr, double ball_c) {
    double min_distance_to_teammate = 1.5; // meter
    double min_dist_penalty = calc_a_penalty_factor(min_distance_to_teammate, ball_c);
    double min_teammate_sqr;
    // TODO: rewrite this function to not have ax_sqr and ball_c, but radius and penalty.
    if (min_dist_penalty < ball_ax_sqr) {
        min_teammate_sqr = ball_ax_sqr;
    }
    else {
        min_teammate_sqr = min_dist_penalty;
    }
    return min_teammate_sqr;
}

//---------------------------------------------------------------------
double TeamPlanner_Grid::calc_a_penalty_factor(double radius, double c) {
    double a = std::numeric_limits<double>::infinity();
    if (fabs(radius) > 1e-9) {
        a = -(c / (radius*radius));
    }
    return a;
}


Geometry::Position TeamPlanner_Grid::findSetPlayPosition(dynamic_role_e dynamic_role, const TeamPlannerData& r_teamplannerData,
                                               const Geometry::Point& preferred_position, int gridFileNumber,
                                               bool strongDesiredX, bool strongDesiredY, bool beAvailableForPass)
{
/* Position attack support, in order of priority (weighing):
     *
     *   + 80  Not too close to the ball
     *   + 50  Not too close to opponents
     *   + 100  previous assigned position
     *   + 50 stay in front of the ball on own half
     *   + 90/120 desired-Y line
     *   + 90/120 desired-X line
     *   + 40/80 if we control the ball make yourself available to receive a pass
     *   + 30  Halfway on opponent half, good lobshot distance (also to have eyes on opponent half for ball detection)
     *   + 20  not behind opponent (from ball position)
     *   + 50  Do not position close to attack supporters path position
     *   + 10  Do not position close to team-mate (planned before attack support, higher priority), e.g. goalkeeper, interceptor
     *   + 8   Stability factor: do some minor preference to current location to avoid constant ¨flipping" between 2 almost equal positions
     *   + 1000 Do not plan outside playing field or in goal areas (inner box of penalty area)
     *   + 1000 Do not plan in own penalty area if already somebody assigned
     */

    const double WEIGHT_INFLUENCE_BALL = 80.0; // Not too close to the ball
    const double WEIGHT_INFLUENCE_OPPONENTS = 50.0; // Not too close to opponents
    const double WEIGHT_PREVIOUS_ASSIGN_POSITION = 100.0; // previous assigned position
    const double WEIGHT_PLAY_ON_OWN_HALF = 50.0; // stay in front of the ball on own half
    const double WEIGHT_PLAY_ON_OPPONENT_HALF = 50.0; // stay at opponent half
    const double WEIGHT_DESIRED_LINE = 90.0;  // desired to be on the line
    const double WEIGHT_STRONG_DESIRED_LINE = 120.0; // strong desired to be on the line
    const double WEIGHT_INTERCEPTION_THREAT = 20.0; // not behind opponent (from ball position)
    const double WEIGHT_COLLIDE_TEAM_MATE = 10.0; // Do not position close to team-mate
    const double WEIGHT_INFLUENCE_CURRENT_POSITION = 8.0; // Stability factor - influence current position
    const double WEIGHT_NOT_AREA = 1000.0;



    // define grid of 50 cm, only in field not outside the border
    double grid_size = r_teamplannerData.parameters.grid_size; //[m]
    double x_grid_half = floor(r_teamplannerData.fieldConfig.getMaxFieldX()/grid_size);
    double y_grid_half = floor(r_teamplannerData.fieldConfig.getMaxFieldY()/grid_size);
    int x_grid_points = 1 + 2 * x_grid_half;
    int y_grid_points = 1 + 2 * y_grid_half;

    double x_pos_ball = r_teamplannerData.ball.position.x;
    double y_pos_ball = r_teamplannerData.ball.position.y;

    PlannerGridInfoData pgid = PlannerGridInfoData();
    vector<GridHeuristic *> heuristics = {};

    // Avoid difficult collaboration, not to close to the ball
    heuristics.push_back(new InfluenceBallHeuristic("InfluenceBall", WEIGHT_INFLUENCE_BALL, pgid, x_pos_ball, y_pos_ball, 4));

    // Avoid difficult collaboration, do not position close to opponents
    heuristics.push_back(new InfluenceOpponentsHeuristic("Influence Opponent", WEIGHT_INFLUENCE_OPPONENTS, pgid, r_teamplannerData.opponents, 2.0));

    heuristics.push_back(new InfluencePreviousAssignedPositionsHeuristic("previous assigned position", WEIGHT_PREVIOUS_ASSIGN_POSITION, pgid, r_teamplannerData, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance(), dynamic_role_e::dr_ATTACKSUPPORTER));

    // stay in front of the ball on own half, so play forward on own half
    if (y_pos_ball < 0) {
        heuristics.push_back(new InSquareHeuristic("Play forward on own half", WEIGHT_PLAY_ON_OWN_HALF, pgid, -r_teamplannerData.fieldConfig.getMaxFieldX(), y_pos_ball+1, r_teamplannerData.fieldConfig.getMaxFieldX(), -r_teamplannerData.fieldConfig.getMaxFieldY()) );
    }
    else {
        heuristics.push_back(new InSquareHeuristic("Play on opponent half", WEIGHT_PLAY_ON_OPPONENT_HALF, pgid, -r_teamplannerData.fieldConfig.getMaxFieldX(), 1, r_teamplannerData.fieldConfig.getMaxFieldX(), -r_teamplannerData.fieldConfig.getMaxFieldY()) );
    }

    // y position of desired position
    double ydesired_weight = strongDesiredY ? WEIGHT_STRONG_DESIRED_LINE : WEIGHT_DESIRED_LINE;
    heuristics.push_back(new DesiredY("Desired-Y", ydesired_weight, pgid, preferred_position.y, r_teamplannerData.fieldConfig));

    // x position of desired position
    double xdesired_weight = strongDesiredX ? WEIGHT_STRONG_DESIRED_LINE : WEIGHT_DESIRED_LINE;
    heuristics.push_back(new DesiredX("Desired-X", xdesired_weight, pgid, preferred_position.x, r_teamplannerData.fieldConfig));

    if (beAvailableForPass) {
        // Avoid difficult collaboration, not behind opponent (from ball position)
        heuristics.push_back(new InterceptionThreatHeuristic("Interception threat", WEIGHT_INTERCEPTION_THREAT, pgid,
                                                             Geometry::Point(r_teamplannerData.ball.position.x, r_teamplannerData.ball.position.y),
                                                             r_teamplannerData, true));
    }

    heuristics.push_back(new CollideTeamMateHeuristic("Distance to AttackSupporters", 50.0, pgid, r_teamplannerData, 4.0, true));

    // Do not position close to team-mate path position
    heuristics.push_back(new CollideTeamMateHeuristic("CollideTeamMate", WEIGHT_COLLIDE_TEAM_MATE, pgid, r_teamplannerData, r_teamplannerData.fieldConfig.getRobotSize()*2));


    // Do not plan in opponent penalty area
    heuristics.push_back(new InOppenentPenaltyAreaHeuristic("InOppenentPenalyArea", WEIGHT_NOT_AREA, pgid, r_teamplannerData));

    // Do not plan in own goal area
    heuristics.push_back(new InOwnGoalAreaHeuristic("InOwnGoalArea", WEIGHT_NOT_AREA, pgid, r_teamplannerData));

    // Do not plan in own penalty area if already somebody assigned
    heuristics.push_back(new AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic("Already player assigned to own penalty area", WEIGHT_NOT_AREA, pgid, r_teamplannerData));

    // Not outside the playing field
    heuristics.push_back(new OutsidePlayFieldHeuristic("OutsidePlayField", WEIGHT_NOT_AREA, pgid, r_teamplannerData.fieldConfig, r_teamplannerData.parameters.attack_supporter_extra_distance_to_stay_from_sideline));

    // Stability heuristic
    heuristics.push_back(new InfluenceCurrentPositionsHeuristic("InfluenceCurrentPositions", WEIGHT_INFLUENCE_CURRENT_POSITION, pgid,
                                                                r_teamplannerData, r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance()));


    /*
     * Additional attack support position heuristics in set-play (various)
     * Normal sweeper can be deployed during a set-play with some additional positioning restrictions below
     * Heuristic function itself checks on correct game-state to identify set-play
     */

    // create x,y grid for calculation. make 0,0 center of grid !!
    std::list<Geometry::Position> allowedTargetPositions = list<Geometry::Position>();
    for (int x_idx = 0; x_idx < x_grid_points; x_idx++) {
        for (int y_idx = 0; y_idx < y_grid_points; y_idx++) {
            double x = (x_idx * grid_size) - (x_grid_half * grid_size);
            double y = (y_idx * grid_size) - (y_grid_half * grid_size);
            if (y > -r_teamplannerData.fieldConfig.getMaxFieldY() + r_teamplannerData.fieldConfig.getRobotRadius()) { // only position where player is not over own backline
                allowedTargetPositions.push_back(Geometry::Position(x, y));
            }
        }
    }


    auto rolePosition = calculateGridValues(allowedTargetPositions, heuristics, r_teamplannerData.parameters, pgid);

    TeamPlanner_Grid::writeGridDataToFile(pgid, r_teamplannerData, Geometry::Point(r_teamplannerData.ball.position), "SetPlay", gridFileNumber);

    bool prepare_phase = isOneOf(r_teamplannerData.gamestate, {game_state_e::CORNER, game_state_e::GOALKICK, game_state_e::FREEKICK, game_state_e::THROWIN});
    if (r_teamplannerData.parameters.wait_on_non_optimal_position_during_prepare_phase && prepare_phase) {
        // find non optimal position to wait during prepare phase

        const unsigned nr_wait_positions = 8;   // check for 8 positions 1.5 meter around best position
        const double radius_non_opt_wait = 2.0;
        const double infield_margin = 0.25;   // distance to stay from side of field
        InterceptionThreatHeuristic InterceptionThreat = InterceptionThreatHeuristic("Interception threat", -18.0, pgid,
                Geometry::Point(r_teamplannerData.ball.position.x, r_teamplannerData.ball.position.y), r_teamplannerData);

        double bestX = rolePosition.x;
        double bestY = rolePosition.y;
        double lowestValue = std::numeric_limits<double>::infinity();
        // for all positions to check
        for (unsigned i = 0; i < nr_wait_positions; i++) {
            double angle = i * 2.0 * M_PI / static_cast<double>(nr_wait_positions);
            double xx = rolePosition.x + radius_non_opt_wait * cos(angle);
            double yy = rolePosition.y + radius_non_opt_wait * sin(angle);
            if (r_teamplannerData.fieldConfig.isInField(xx,yy, infield_margin)) {
                // only check positions in the field
                double val = InterceptionThreat.getValue(xx, yy);
                 if (val < lowestValue) {
                     // value is worse than before, to it a good candidate to wait
                     bestX = xx;
                     bestY = yy;
                 }
            }
        }
        rolePosition.x = bestX;
        rolePosition.y = bestY;
    }

    return rolePosition;
}

} /* namespace trs */




