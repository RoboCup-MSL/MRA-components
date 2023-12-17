/*
 * TeamPlannerGrid.cpp
 *
 *  Created on: Jan 3, 2016
 *      Author: jurge
 */

#include "TeamPlannerGrid.hpp"

#include <limits>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <list>
#include "FieldConfig.h"
#include "GridHeuristic.hpp"
#include "MathUtils.h"
#include "PlannerGridInfoData.hpp"
#include "TeamPlay.hpp"

using namespace std;

namespace trs {


MRA::Geometry::Point TeamPlanner_Grid::findBallPlayerPosition(const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate, const PlannerOptions& plannerOptions,
		const MovingObject& ball, const std::vector<TeamPlannerOpponent>& Opponents, int gridFileNumber, const FieldConfig& fieldConfig,
		const ball_pickup_position_t& ball_pickup_position, bool passIsRequired) {
	const double infield_margin = 0.25;   // distance to stay from side of field
	const unsigned nr_positions_first_circle = 8;
	const double dist_first_circle = 1.0;
	const unsigned nr_positions_second_circle = 16;
	const double dist_second_circle = 2.25;

	double xc = ball_pickup_position.x;
	double yc = ball_pickup_position.y;

	MRA::Geometry::Point bestPosition = MRA::Geometry::Point(xc, yc);  // default : set best to pickup position

	std::list<MRA::Geometry::Point> allowedTargetPositions = list<MRA::Geometry::Point>();
	allowedTargetPositions.push_back(MRA::Geometry::Point(xc, yc));
	// create possible target positions:

	MRA::Geometry::Point ballPlayerPos = bestPosition; // set default to pickup position
	bool ballPlayerFound = false;
	// add current position for ball-player
	for (unsigned r_idx = 0; r_idx < Team.size(); r_idx++) {
		if (Team[r_idx].controlBall) {
			ballPlayerPos = Team[r_idx].position.getPosition().getPoint();
			allowedTargetPositions.push_back(ballPlayerPos); // ballplayer current position
			ballPlayerFound = true;
		}
	}
	// pickup position and 2 circles around it.
	for (unsigned i = 0; i < nr_positions_first_circle; i++) {
		double angle = i * 2.0 * M_PI / static_cast<double>(nr_positions_first_circle);
		double xx = xc + dist_first_circle * cos(angle);
		double yy = yc + dist_first_circle * sin(angle);
		if (fieldConfig.isInField(xx,yy,infield_margin)) {
			allowedTargetPositions.push_back(MRA::Geometry::Point(xx, yy));
		}

	}
	for (unsigned i = 0; i < nr_positions_second_circle; i++) {
		double angle = i * 2.0 * M_PI / static_cast<double>(nr_positions_second_circle);
		double xx = xc + dist_second_circle * cos(angle);
		double yy = yc + dist_second_circle * sin(angle);
		if (fieldConfig.isInField(xx,yy,infield_margin)) {
			allowedTargetPositions.push_back(MRA::Geometry::Point(xx, yy));
		}
	}

	// evaluate all possible target positions
	// FOR all points
	//  - far from opponent is better
	//  - close to own player is better
	//  - 1 circle is better than 2nd circle
	//

	MRA::Geometry::Point bestPosForGoal = MRA::Geometry::Point(0, (fieldConfig.getMaxFieldY())-2.0); // 2 meters before goal
	PlannerGridInfoData pgid = PlannerGridInfoData();
	vector<GridHeuristic *> heuristics = vector<GridHeuristic *>();
	if (false) {
		heuristics.push_back(new DistanceToHeuristic("Distance to best pos before goal", 1.0, pgid, bestPosForGoal, fieldConfig.getMaxPossibleFieldDistance()));
		heuristics.push_back(new InfluenceOpponentsHeuristic("Influence Opponent", 20.0, pgid, Opponents, 0.7));
	}
	else {
		//	Shoot on goal
		double shootOnGoalFactor = 100;
		double passFactor = 10;
		// Pass is required OR
		//	-- time in possession goal success factor: 0 to 2 sec. x 1.0, from 2.0 to 5.0 seconds x0.5 , after 5.0 seconds: 1.0x (force shot on goal)
		if (passIsRequired  || (ball_pickup_position.ts > 2.0 && ball_pickup_position.ts < 5.0)) {
			shootOnGoalFactor = 10;
			passFactor = 100;
		}

		if (ballPlayerFound) {
			// prefer driving too much, make current position is slightly preferred
			heuristics.push_back(new DistanceToPointHeuristic("Dist to current position", 0.1, pgid,
					ballPlayerPos, 1.0 /*scaling*/, 5.0 /* max range*/, false /*inverted*/));
		}
		heuristics.push_back(new ShootOnGoalHeuristic("Shoot on goal", shootOnGoalFactor, pgid,
						Team, Opponents, fieldConfig, ball_pickup_position));
		heuristics.push_back(new PassHeuristic("passing", passFactor, pgid,
						Team, Opponents, fieldConfig, ball_pickup_position, plannerOptions));
		// prefer keep 2 meter distance from opponent
		heuristics.push_back(new StayAwayFromOpponentsHeuristic("Stay away from opponents",100,
				pgid, ballPlayerPos, ball, Opponents, 2.0));

		//
		//	Passing
		//	-- multiply probability success factor : time 0-5 seconds. 1-(0.2 x seconds since ball possession)
		//	-- interception thread per player: normalize.

	}
	bestPosition = calculateGridValues(allowedTargetPositions, heuristics, plannerOptions, pgid);
	TeamPlanner_Grid::writeGridDataToFile(pgid, Team, Opponents,ball, plannerOptions, "ShootPosition", gridFileNumber);

	return bestPosition;
}

void TeamPlanner_Grid::handle_penalty_heuristics(game_state_e gamestate,
		const PlannerOptions &plannerOptions,
		const std::vector<TeamPlannerRobot>& Team,
		const MRA::Geometry::Point& r_ballPos,
		const FieldConfig &fieldConfig,
		vector<GridHeuristic*> &heuristics,
		PlannerGridInfoData &pgid)
{
	// Handle penalty related heuristics (used for multiple roles)

	if (gamestate == game_state_e::PENALTY || gamestate == game_state_e::PENALTY_AGAINST) {
		// penalty needs minimal distance to ball of ca 3.5 m (minimal 3 meter in robocup rules)
		auto ball_penalty = plannerOptions.grid_close_to_ball_restart_penalty_penalty; // default
		auto ball_radius = plannerOptions.grid_close_to_ball_restart_penalty_radius;
		heuristics.push_back(new InfluenceBallHeuristic("InfluenceBall", ball_penalty, pgid, r_ballPos.x, r_ballPos.y, ball_radius));
	}

	if (gamestate == game_state_e::PENALTY_AGAINST) {
		// Not allow to be in own penalty area (RoboCup rules)
		heuristics.push_back(new InOwnPenaltyAreaHeuristic("InOwnPenalyArea", 100000, pgid, plannerOptions, fieldConfig));

		// prefer position close to the backline: Y halfway between goal area en penalty area
		auto desired_y = -fieldConfig.getMaxFieldY() + fieldConfig.getGoalAreaLength() + 0.5 * (fieldConfig.getPenaltyAreaLength() - fieldConfig.getGoalAreaLength());
		heuristics.push_back(new DistanceToLineHeuristic("DesiredY", 20, pgid,
				-fieldConfig.getMaxFullFieldX(), desired_y, fieldConfig.getMaxFullFieldX(), desired_y, fieldConfig.getMaxPossibleFieldDistance()));

		// Do not position close to already assigned team-mate position: avoid to be close to eachother
		heuristics.push_back(new CollideTeamMateHeuristic("CollideTeamMate", 2000.0, pgid, Team, 1.5));

		// Prefer position close too the ball
		heuristics.push_back(new DistanceToPointHeuristic("Close to ball", 200, pgid, r_ballPos, 8.0, fieldConfig.getMaxPossibleFieldDistance(), false));
	}
	if (gamestate == game_state_e::PENALTY) {
		// Not allow to be in opponent penalty area (RoboCup rules)
		heuristics.push_back(new InOppenentPenaltyAreaHeuristic("InOpponentPenalyArea", 100000, pgid, plannerOptions, fieldConfig));
	}
}

/**
 * Method to find to most attractive position to defend an given opponent during setplay against.
 * A grid is created, for all points on the grid a heuristic (attractiveness) is calculated.
 * Most attractive position will be the position
 */
MRA::Geometry::Point TeamPlanner_Grid::findManToManDefensivePosition(dynamic_role_e dynamic_role, const MRA::Geometry::Point& oppentToDefend, const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate,
		const PlannerOptions& plannerOptions, 	const MovingObject& ball, const std::vector<TeamPlannerOpponent>& Opponents,
		int gridFileNumber, const FieldConfig& fieldConfig, bool setPlayActive, bool teamControlBall)
{
	// define grid of 50 cm, only in field not outside the border
    MRA::Geometry::Point ballPos = ball.getPosition().getPoint();
	double x_pos_ball = ballPos.x;
	double y_pos_ball = ballPos.y;


	PlannerGridInfoData pgid = PlannerGridInfoData();
	vector<GridHeuristic *> heuristics = vector<GridHeuristic *>();

	/*
	 * Defender position heuristics in open-play (offensive and defensive)
	 */
	double alfa = oppentToDefend.angle(ballPos);
	if (oppentToDefend.distanceTo(ballPos) < plannerOptions.setplay_against_dist_to_opponent + 0.25) {
		// player can not be positioned between ball and opponent.
		// prefer place between opponent and own goal.
		alfa = oppentToDefend.angle(fieldConfig.getOwnGoal());
	}

	double preferred_x = oppentToDefend.x - (cos(alfa) * plannerOptions.setplay_against_dist_to_opponent);
	double preferred_y = oppentToDefend.y - (sin(alfa) * plannerOptions.setplay_against_dist_to_opponent);


	// Defend opponents on own half or close to own half
	// Preferred position approx 1 meter from opponent, with a preference with the defender to the most forward defender (smallest Y)
	//heuristics.push_back(new InCircleHeuristic("ShieldOpponent", 100.0, pgid, oppentToDefend.x, oppentToDefend.y, 1.5, true)); // invert=true, so in circle attracts

	// Positioned on the line between opponent and ball (so opponent cannot be reached)
	heuristics.push_back(new OnLineBetweenPointsHeuristic("OnLineBetweenBallAndOpponent", 500.0, pgid, x_pos_ball, y_pos_ball, preferred_x, preferred_y, fieldConfig.getMaxPossibleFieldDistance()));

	// Play close to any opponent but not on exactly on top of it
	heuristics.push_back(new InfluenceOpponentsHeuristic("Influence Opponents", 800, pgid, Opponents, 1.0));

	// Play close to the opponent to defend but not on exactly on top of it
	heuristics.push_back(new DistanceToPointHeuristic("Opponent to defend", 200, pgid, MRA::Geometry::Point(preferred_x, preferred_y), 8.0, 8.0, false));

	// Do not position close to team-mate path position, avoid double defense if other opponents are available
	heuristics.push_back(new CollideTeamMateHeuristic("CollideTeamMate", 1000, pgid, Team, 0.7));

	// Do not plan in opponent penalty area
	heuristics.push_back(new InOppenentPenaltyAreaHeuristic("InOppenentPenalyArea", 1000.0, pgid, plannerOptions, fieldConfig));

	// Do not plan in own goal area
	heuristics.push_back(new InOwnGoalAreaHeuristic("InOwnGoalArea", 1000.0, pgid, plannerOptions, fieldConfig));

	// Do not plan in own penalty area if already somebody assigned
	heuristics.push_back(new AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic("Already player assigned to own penalty area", 1000.0, pgid, Team, fieldConfig));

	// This still needed?
	heuristics.push_back(new OutsidePlayFieldHeuristic("OutsidePlayField", 1000.0, pgid, fieldConfig, 0.0));

	// Stability heuristic
	heuristics.push_back(new InfluencePreviousAssignedPositionsHeuristic("previous assigned position", 150.0, pgid, Team, fieldConfig.getMaxPossibleFieldDistance(), dynamic_role));

	/*
	 * Additional defender position heuristics in set-play (various)
	 * Normal defender can be deployed during a set-play with some additional positioning restrictions below
	 * Heuristic function itself checks on correct game-state to identify set-play
	 */

	// parameters to get a minimal distance to the ball (used function is a parabola).
	double ball_penalty = plannerOptions.grid_close_to_ball_normal_penalty; // default
	double ball_radius = plannerOptions.grid_close_to_ball_normal_radius;

	//stay out of 3 meter zone around ball in case of e.g. free-kick_against; to avoid touching the ball
	if ((gamestate == game_state_e::FREEKICK_AGAINST) || (gamestate == game_state_e::GOALKICK_AGAINST) || (gamestate == game_state_e::GOALKICK) || (gamestate == game_state_e::FREEKICK) ||
		(gamestate == game_state_e::CORNER_AGAINST)   || (gamestate == game_state_e::THROWIN_AGAINST) )
	{
		ball_penalty = plannerOptions.grid_close_to_ball_restart_penalty_penalty; // default
		ball_radius = plannerOptions.grid_close_to_ball_restart_penalty_radius;
		heuristics.push_back(new BallSetplayAgainstHeuristic("InfluenceBallSetplayAgainst", ball_penalty, pgid, x_pos_ball, y_pos_ball, ball_radius, fieldConfig));

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
					-fieldConfig.getMaxFullFieldX(), desired_y, fieldConfig.getMaxFullFieldX(), desired_y, fieldConfig.getMaxPossibleFieldDistance()));
		}
	}

	// apply penalty heuristics if needed
	handle_penalty_heuristics(gamestate, plannerOptions, Team, ballPos, fieldConfig, heuristics, pgid);

	if (teamControlBall) {
		// avoid defender being to close to the ball: add penalty for being close to the ball
		heuristics.push_back(new InfluenceBallHeuristic("InfluenceBall", 5000, pgid, x_pos_ball, y_pos_ball, 2.0 /* [m] */));
	}


	double grid_size = plannerOptions.grid_size * 0.5; //[m]   // times denser than normal with reduce grid
	double x_radius = 8.0;		// limit number of calculations in x (max 2*x_radius)
	double x_steps = floor(x_radius / grid_size);
	double min_x = preferred_x - (x_steps * grid_size);
	double max_x = preferred_x + (x_steps * grid_size);
	double y_radius = 5.0; // limit number of calculations in y (max 2*y_radius)
	double y_steps = floor(y_radius / grid_size);
	double min_y = preferred_y - (y_steps * grid_size);
	double max_y = preferred_y + (y_steps * grid_size);

	std::list<MRA::Geometry::Point> allowedTargetPositions = list<MRA::Geometry::Point>();

	// create x,y grid for calculation. with prefered x , y in center of grid !!
	for (double x = min_x; x <= (max_x + 0.01); x += grid_size) {
		for (double y = min_y; y <= (max_y + 0.01); y += grid_size) {
			if (fieldConfig.isInField(x, y, 0))
			{ // only position where player is inside the field
				allowedTargetPositions.push_back(MRA::Geometry::Point(x, y));
			}
		}
	}
	MRA::Geometry::Point bestPosition = calculateGridValues(allowedTargetPositions, heuristics, plannerOptions, pgid);
	TeamPlanner_Grid::writeGridDataToFile(pgid, Team, Opponents,ball, plannerOptions, "DefensivePosition", gridFileNumber);

	return bestPosition;


}

/**
 * Method to find to most attractive position for zone defense support.
 * A grid is created, for all points on the grid a heuristic (attractiveness) is calculated.
 * Most attractive position will be the offensive position
 */
MRA::Geometry::Point TeamPlanner_Grid::findDefensivePosition(const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate,
		const PlannerOptions& plannerOptions, 	const MovingObject& ball, const std::vector<TeamPlannerOpponent>& Opponents,
		int gridFileNumber, const FieldConfig& fieldConfig)
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
	double grid_size = plannerOptions.grid_size; //[m]
	double x_grid_half = floor(fieldConfig.getMaxFieldX()/grid_size);
	double y_grid_half = floor(fieldConfig.getMaxFieldY()/grid_size);
	int x_grid_points = 1 + 2 * x_grid_half;
	int y_grid_points = 1 + 2 * y_grid_half;

	double x_pos_ball = ball.getPosition().getPoint().x;
	double y_pos_ball = ball.getPosition().getPoint().y;


	PlannerGridInfoData pgid = PlannerGridInfoData();
	vector<GridHeuristic *> heuristics = vector<GridHeuristic *>();

	/*
	 * Defender position heuristics in open-play (offensive and defensive)
	 */


	// Defend opponents on own half or close to own half
	for (unsigned op_idx = 0; op_idx < Opponents.size(); op_idx++) {
		TeamPlannerOpponent opponent = Opponents[op_idx];
		MRA::Geometry::Point opponentPos = opponent.position.getPosition().getPoint();
		if (opponentPos.y < 2) {

			// Preferred position approx 1 meter from opponent, with a preference with the defender to the most forward defender (smallest Y)
			heuristics.push_back(new InCircleHeuristic("ShieldOpponent", 100.0 - (opponentPos.y * 2) , pgid, opponentPos.x, opponentPos.y, 1.5, true)); // invert=true, so in circle attracts

			// Positioned on the line between opponent and ball (so opponent cannot be reached)
			heuristics.push_back(new OnLineBetweenPointsHeuristic("OnLineBetweenBallAndOpponent", 30.0, pgid, x_pos_ball, y_pos_ball, opponentPos.x, opponentPos.y, fieldConfig.getMaxPossibleFieldDistance()));
		}

	}

	// Play close to opponent but not on exactly on top of it
	heuristics.push_back(new InfluenceOpponentsHeuristic("Influence Opponent", 200.0, pgid, Opponents, 0.7));

	// Do not position close to team-mate path position, avoid double defense if other opponents are available
	heuristics.push_back(new CollideTeamMateHeuristic("CollideTeamMate", 50.0, pgid, Team, 2.5));

    // Defend on -4.5 M line (halfway own half), lower priority so it only takes as the position if there are no opponents to defend
	heuristics.push_back(new DesiredYSweeper("DesiredYDefender", 20.0, pgid, -fieldConfig.getMaxFieldY()*0.5, fieldConfig));

	// Stay away out of corners of the field, not the best defensive position
	heuristics.push_back(new InfluenceCornerHeuristic("InfluenceCorner", 10.0, pgid, fieldConfig));

	heuristics.push_back(new InfluencePreviousAssignedPositionsHeuristic("previous assigned position", 100.0, pgid, Team, fieldConfig.getMaxPossibleFieldDistance(), dynamic_role_e::dr_DEFENDER));


	// Do not plan in opponent penalty area
	heuristics.push_back(new InOppenentPenaltyAreaHeuristic("InOppenentPenalyArea", 1000.0, pgid, plannerOptions, fieldConfig));

	// Do not plan in own goal area
	heuristics.push_back(new InOwnGoalAreaHeuristic("InOwnGoalArea", 1000.0, pgid, plannerOptions, fieldConfig));

	// Do not plan in own penalty area if already somebody assigned
	heuristics.push_back(new AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic("Already player assigned to own penalty area", 1000.0, pgid, Team, fieldConfig));

	// This still needed?
	heuristics.push_back(new OutsidePlayFieldHeuristic("OutsidePlayField", 1000.0, pgid, fieldConfig, 0.0));

	// Stability heuristic
	heuristics.push_back(new InfluenceCurrentPositionsHeuristic("InfluenceCurrentPositions", 8.0, pgid, Team, fieldConfig.getMaxPossibleFieldDistance()));

	/*
	 * Additional defender position heuristics in set-play (various)
	 * Normal defender can be deployed during a set-play with some additional positioning restrictions below
	 * Heuristic function itself checks on correct game-state to identify set-play
	 */

	// parameters to get a minimal distance to the ball (used function is a parabola).
	double ball_penalty = plannerOptions.grid_close_to_ball_normal_penalty; // default
	double ball_radius = plannerOptions.grid_close_to_ball_normal_radius;

	// todo below if-then needs to replaced by structured case

	//stay out of 3 meter zone around ball in case of e.g. free-kick_against; to avoid touching the ball
	if ((gamestate == game_state_e::FREEKICK_AGAINST) || (gamestate == game_state_e::GOALKICK_AGAINST) || (gamestate == game_state_e::GOALKICK) || (gamestate == game_state_e::FREEKICK) ||
		(gamestate == game_state_e::CORNER_AGAINST)   || (gamestate == game_state_e::THROWIN_AGAINST) ) {
		ball_penalty = plannerOptions.grid_close_to_ball_restart_penalty_penalty; // default
		ball_radius = plannerOptions.grid_close_to_ball_restart_penalty_radius;
		heuristics.push_back(new InfluenceBallHeuristic("InfluenceBall", ball_penalty, pgid, x_pos_ball, y_pos_ball, ball_radius));
	}

	// apply penalty heuristics if needed
	handle_penalty_heuristics(gamestate, plannerOptions, Team, ball.getXYlocation(), fieldConfig, heuristics, pgid);

	std::list<MRA::Geometry::Point> allowedTargetPositions = list<MRA::Geometry::Point>();

	// create x,y grid for calculation. make 0,0 center of grid !!
	for (int x_idx  = 0; x_idx < x_grid_points; x_idx++) {
		for (int y_idx  = 0; y_idx < y_grid_points; y_idx++) {
			double x = (x_idx * grid_size) - (x_grid_half * grid_size);
			double y = (y_idx * grid_size) - (y_grid_half * grid_size);
			if (y > -fieldConfig.getMaxFieldY() + fieldConfig.getRobotRadius()) { // only position where player is not over own backline
				allowedTargetPositions.push_back(MRA::Geometry::Point(x, y));
			}
		}
	}
	MRA::Geometry::Point bestPosition = calculateGridValues(allowedTargetPositions, heuristics, plannerOptions, pgid);
	TeamPlanner_Grid::writeGridDataToFile(pgid, Team, Opponents,ball, plannerOptions, "DefensivePosition", gridFileNumber);

	return bestPosition;
}


/**
 * Method to find to most attractive position for zone defense support.
 * A grid is created, for all points on the grid a heuristic (attractiveness) is calculated.
 * Most attractive position will be the offensive position
 */
MRA::Geometry::Point TeamPlanner_Grid::findDefensivePositionDuringPenaltyShootOut(const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate,
		const PlannerOptions& plannerOptions, 	const MovingObject& ball, const std::vector<TeamPlannerOpponent>& Opponents,
		int gridFileNumber, const FieldConfig& fieldConfig)
{
	double field_direction = -1.0;
	if (gamestate == PENALTY_SHOOTOUT_AGAINST) {
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
	double grid_size = plannerOptions.grid_size; //[m]
	double x_grid_half = floor(fieldConfig.getMaxFieldX()/grid_size);
	double y_grid_half = floor(fieldConfig.getMaxFieldY()/grid_size);
	int x_grid_points = 1 + 2 * x_grid_half;
	int y_grid_points = 1 + 2 * y_grid_half;



	PlannerGridInfoData pgid = PlannerGridInfoData();
	vector<GridHeuristic *> heuristics = vector<GridHeuristic *>();

	// Play close to opponent but not on exactly on top of it
	heuristics.push_back(new InfluenceOpponentsHeuristic("Influence Opponent", 200.0, pgid, Opponents, 1.0));

	// Do not position close to team-mate path position, avoid double defense if other opponents are available
	heuristics.push_back(new CollideTeamMateHeuristic("CollideTeamMate", 200.0, pgid, Team, 1.0));

    // stay on  1 meter from center line (halfway own half), lower priority so it only takes as the position if there are no opponents to defend
	heuristics.push_back(new DesiredYSweeper("DesiredYDefender", 200.0, pgid, field_direction * 1.0, fieldConfig));

	heuristics.push_back(new OutsidePlayFieldHeuristic("OutsidePlayField", 100.0, pgid, fieldConfig, 0.0));

	// Stability heuristic
	heuristics.push_back(new InfluenceCurrentPositionsHeuristic("InfluenceCurrentPositions", 8.0, pgid, Team, fieldConfig.getMaxPossibleFieldDistance()));

	heuristics.push_back(new InfluencePreviousAssignedPositionsHeuristic("previous assigned position", 100.0, pgid, Team, fieldConfig.getMaxPossibleFieldDistance(), dynamic_role_e::dr_PENALTY_DEFENDER));

	//stay out of 3 meter zone around center of the field
	heuristics.push_back(new InfluenceBallHeuristic("Influence Center Circle", 1000.0, pgid, 0, 0, fieldConfig.getCenterCirleDiameter() + 0.5));

	std::list<MRA::Geometry::Point> allowedTargetPositions = list<MRA::Geometry::Point>();

	// create x,y grid for calculation. make 0,0 center of grid !!
	for (int x_idx  = 0; x_idx < x_grid_points; x_idx++) {
		for (int y_idx  = 0; y_idx < y_grid_points; y_idx++) {
			double x = (x_idx * grid_size) - (x_grid_half * grid_size);
			double y = (y_idx * grid_size) - (y_grid_half * grid_size);
			if (y > -fieldConfig.getMaxFieldY() + fieldConfig.getRobotRadius()) { // only position where player is not over own backline
				allowedTargetPositions.push_back(MRA::Geometry::Point(x, y));
			}
		}
	}
	MRA::Geometry::Point bestPosition = calculateGridValues(allowedTargetPositions, heuristics, plannerOptions, pgid);
	TeamPlanner_Grid::writeGridDataToFile(pgid, Team, Opponents,ball, plannerOptions, "DefensivePosition", gridFileNumber);

	return bestPosition;
}

//---------------------------------------------------------------------
/**
 * Method to find to most attractive position for sweeper role
 * A grid is created, for all points on the grid a heuristic (attractiveness) is calculated.
 * Sweeper role is typical used in defensive formations, it is the "Ausputzer" / "laatste man" / "lock on the door"
 * It normally positioned behind defenders to tackle any attacker bypassing defensive line
 */

MRA::Geometry::Point TeamPlanner_Grid::findSweeperPosition(const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate,
		const PlannerOptions& plannerOptions, 	const MovingObject& ball, const std::vector<TeamPlannerOpponent>& Opponents,
		int gridFileNumber, const FieldConfig& fieldConfig)
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
	double grid_size = plannerOptions.grid_size; //[m]
	double x_grid_half = floor(fieldConfig.getMaxFieldX()/grid_size);
	double y_grid_half = floor(fieldConfig.getMaxFieldY()/grid_size);
	int x_grid_points = 1 + 2 * x_grid_half;
	int y_grid_points = 1 + y_grid_half;  // only half field;

	double x_pos_ball = ball.getPosition().getPoint().x;
	double y_pos_ball = ball.getPosition().getPoint().y;

	PlannerGridInfoData pgid = PlannerGridInfoData();
	vector<GridHeuristic *> heuristics = vector<GridHeuristic *>();

	/*
	 * Sweeper position heuristics in open-play (offensive and defensive)
	 */

	// on line between ball and own goal
	heuristics.push_back(new OnLineBetweenPointsHeuristic("OnLineBetweenBallAndOwnGoal", 100.0, pgid,
				x_pos_ball, y_pos_ball,
				0.0, -fieldConfig.getMaxFieldY(),
				fieldConfig.getMaxPossibleFieldDistance()));

	// The preferred Y location of the sweeper on the field
	double topOwnPenaltyAreaY = -fieldConfig.getMaxFieldY() + fieldConfig.getPenaltyAreaLength() + fieldConfig.getRobotRadius();
	double besidePenaltyAreaX = fieldConfig.getPenaltyAreaWidth()*0.5 + fieldConfig.getRobotRadius();
	double topOwnGoalAreaY = -fieldConfig.getMaxFieldY() + fieldConfig.getGoalAreaLength() + fieldConfig.getRobotRadius();
	if (y_pos_ball >= 0) {
		heuristics.push_back(new DesiredYSweeper("DesiredYSweeper", 20.0, pgid, -fieldConfig.getMaxFieldY()*0.5, fieldConfig));
	}
	else if (y_pos_ball >= topOwnPenaltyAreaY) {
		heuristics.push_back(new DesiredYSweeper("DesiredYSweeper", 20.0, pgid, topOwnPenaltyAreaY, fieldConfig));
	}
	else if (x_pos_ball >= besidePenaltyAreaX) {
		heuristics.push_back(new DesiredXSweeper("DesiredXSweeper", 20.0, pgid, besidePenaltyAreaX, fieldConfig));
	}
	else if (x_pos_ball <= -besidePenaltyAreaX) {
		heuristics.push_back(new DesiredXSweeper("DesiredXSweeper", 20.0, pgid, -besidePenaltyAreaX, fieldConfig));
	}
	else {
		// ball in penalty area, put sweeper just above goal area
		heuristics.push_back(new DesiredYSweeper("DesiredYSweeper", 20.0, pgid, topOwnGoalAreaY, fieldConfig));

	}
    // Do not position on top off team-mate
	heuristics.push_back(new CollideTeamMateHeuristic("CollideTeamMate", 40.0, pgid, Team, fieldConfig.getRobotSize()*2));

	// Only one player is allowed in the penalty area
	heuristics.push_back(new AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic("Already player assigned to own penalty area", 100000, pgid, Team, fieldConfig)); // Do not enter the outer goal area if already an another player is in.

	// Do not plan outside playing field or in goal area
	heuristics.push_back(new InOwnGoalAreaHeuristic("InOwnGoalArea", 1000.0, pgid, plannerOptions, fieldConfig)); // Do not enter the inner Goal area

	// Stability heuristic: 5 weight
	heuristics.push_back(new InfluenceCurrentPositionsHeuristic("InfluenceCurrentPositions", 5, pgid, Team, fieldConfig.getMaxPossibleFieldDistance()));

	heuristics.push_back(new InfluencePreviousAssignedPositionsHeuristic("previous assigned position", 100.0, pgid, Team, fieldConfig.getMaxPossibleFieldDistance(), dynamic_role_e::dr_SWEEPER));

	/*
	 * Additional sweeper position heuristics in set-play (various)
	 * Normal sweeper can be deployed during a set-play with some additional positioning restrictions below
	 * Heuristic function itself checks on correct game-state to identify set-play
	 */

	// parameters to get a minimal distance to the ball (used function is a parabola).
	double ball_penalty = plannerOptions.grid_close_to_ball_normal_penalty; // default
	double ball_radius = plannerOptions.grid_close_to_ball_normal_radius;

	// todo below if-then needs to replaced by structured case

	//stay out of 3 meter zone around ball in case of e.g. free-kick_against; to avoid touching the ball
	if ((gamestate == game_state_e::FREEKICK_AGAINST) || (gamestate == game_state_e::GOALKICK_AGAINST) || (gamestate == game_state_e::GOALKICK) || (gamestate == game_state_e::FREEKICK) ||
			(gamestate == game_state_e::CORNER_AGAINST)   || (gamestate == game_state_e::THROWIN_AGAINST) ) {
		ball_penalty = plannerOptions.grid_close_to_ball_restart_penalty_penalty; // default
		ball_radius = plannerOptions.grid_close_to_ball_restart_penalty_radius;
		heuristics.push_back(new BallSetplayAgainstHeuristic("InfluenceBall", ball_penalty, pgid, x_pos_ball, y_pos_ball, ball_radius, fieldConfig));
	}

	// apply penalty heuristics if needed
	handle_penalty_heuristics(gamestate, plannerOptions, Team, ball.getXYlocation(), fieldConfig, heuristics, pgid);

	std::list<MRA::Geometry::Point> allowedTargetPositions = list<MRA::Geometry::Point>();
	// create x,y grid for calculation. make 0,0 center of grid !!
	for (int x_idx  = 0; x_idx < x_grid_points; x_idx++) {
		for (int y_idx  = 0; y_idx < y_grid_points; y_idx++) {
			double x = (x_idx * grid_size) - (x_grid_half * grid_size);
			double y = (y_idx * grid_size) - (y_grid_half * grid_size);
			if (y > -fieldConfig.getMaxFieldY() + fieldConfig.getRobotRadius()) { // only position where player is not over own backline
				allowedTargetPositions.push_back(MRA::Geometry::Point(x, y));
			}
		}
	}
	MRA::Geometry::Point bestPosition = calculateGridValues(allowedTargetPositions, heuristics, plannerOptions, pgid);
	TeamPlanner_Grid::writeGridDataToFile(pgid, Team, Opponents,ball, plannerOptions, "Sweeper", gridFileNumber);
	return bestPosition;
}

MRA::Geometry::Point TeamPlanner_Grid::calculateGridValues(const std::list<MRA::Geometry::Point>& allowedTargetPositions,
		vector<GridHeuristic*> heuristics, const PlannerOptions& plannerOptions, PlannerGridInfoData& pgid) {

	double lowest_value = std::numeric_limits<double>::infinity();
	double lowest_x = 0;
	double lowest_y = 0;
	unsigned cell_nr = 0;
	vector<griddata_t> gridData = vector<griddata_t>();

	// create x,y grid for calculation. make 0,0 center of grid !!
	for (std::list<MRA::Geometry::Point>::const_iterator pos_it=allowedTargetPositions.begin(); pos_it != allowedTargetPositions.end(); ++pos_it) {
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
		if (plannerOptions.saveGridDataToFile) {
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
	for (auto it = heuristics.begin(); it != heuristics.end(); ++it) {
		delete (*it);
	}
	return MRA::Geometry::Point(lowest_x, lowest_y);
}

//---------------------------------------------------------------------
/**
 * Method to find to most attractive position for interceptor role during restart
 * A grid is created, for all points on the grid a heuristic (attractiveness) is calculated.
 */
MRA::Geometry::Point TeamPlanner_Grid::findInterceptorPositionDuringRestart(const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate,
		const PlannerOptions& plannerOptions, 	const MovingObject& ball, const std::vector<TeamPlannerOpponent>& Opponents,
		int gridFileNumber, const FieldConfig& fieldConfig)
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
	double grid_size = plannerOptions.grid_size; //[m]
	double x_grid_half = floor(fieldConfig.getMaxFieldX()/grid_size);
	double y_grid_half = floor(fieldConfig.getMaxFieldY()/grid_size);
	int x_grid_points = 1 + 2 * x_grid_half;
	int y_grid_points = 1 + 2 * y_grid_half;

	double x_pos_ball = ball.getPosition().getPoint().x;
	double y_pos_ball = ball.getPosition().getPoint().y;

	// parameters to get a minimal distance to the ball (used function is a parabola).
	// TODO should be mixed to below, where is ball_penalty, etc used?

	double ball_penalty = plannerOptions.grid_close_to_ball_normal_penalty; // default
	double ball_radius = plannerOptions.grid_close_to_ball_normal_radius;
	//double ball_ax_sqr = calc_a_penalty_factor(2.0, ball_c); // default 2 meter  // TODO via options
	if ((gamestate == game_state_e::FREEKICK_AGAINST) || (gamestate == game_state_e::GOALKICK_AGAINST) ||
			(gamestate == game_state_e::CORNER_AGAINST)   || (gamestate == game_state_e::THROWIN_AGAINST) ) {
		// hotfix! TODO: check this (penalty_penalty and penalty radius)
		ball_penalty = plannerOptions.grid_close_to_ball_restart_penalty_penalty; // default
		ball_radius = plannerOptions.grid_close_to_ball_restart_penalty_radius;
	}
	if (gamestate == game_state_e::DROPPED_BALL) {
		ball_penalty = plannerOptions.grid_close_to_ball_restart_dropball_penalty; // default
		ball_radius = plannerOptions.grid_close_to_ball_restart_dropball_radius;
	}

	// store data in vector when plannerOptions.saveGridDataToFile is enabled.
	PlannerGridInfoData pgid = PlannerGridInfoData();
	vector<GridHeuristic *> heuristics = vector<GridHeuristic *>();

	if (gamestate != game_state_e::PENALTY && gamestate != game_state_e::PENALTY_AGAINST) {
		//During restart player of defending team cannot stay in circle with radius of 3 M around the ball until ball is in play
		heuristics.push_back(new InfluenceBallHeuristic("InfluenceBall", ball_penalty, pgid, x_pos_ball, y_pos_ball, ball_radius));
	}

	// apply penalty heuristics if needed
	handle_penalty_heuristics(gamestate, plannerOptions, Team, ball.getXYlocation(), fieldConfig, heuristics, pgid);


	// Position interceptor on the 3 M circle around the ball (as close as allowed to the ball)
	heuristics.push_back(new DistanceToHeuristic("Distance to ball", 100.0, pgid, ball.getPosition().getPoint(), fieldConfig.getMaxPossibleFieldDistance()));

	// Position interceptor as close as possible to opponent interceptor (located within the 3 meter circle)
	heuristics.push_back(new InfluencePreviousAssignedPositionsHeuristic("previous assigned position", 100.0, pgid, Team, fieldConfig.getMaxPossibleFieldDistance(), dynamic_role_e::dr_INTERCEPTOR));


	// Position interceptor/sweeper optimized when defending on own half (both leave room for each other)
	if (y_pos_ball <= -7) {
		heuristics.push_back(new OnLineBetweenPointsHeuristic("NotOnLineBetweenBallAndOwnGoal", 20.0, pgid,
						x_pos_ball, y_pos_ball,
						0.0, 0.0,
						fieldConfig.getMaxPossibleFieldDistance()));
		}
		else  {
			heuristics.push_back(new OnLineBetweenPointsHeuristic("NotOnLineBetweenBallAndOwnGoal", 20.0, pgid,
									x_pos_ball, y_pos_ball,
									0.0, -3.0,
									fieldConfig.getMaxPossibleFieldDistance()));
		}


    // Do not position on top off team-mate
	heuristics.push_back(new CollideTeamMateHeuristic("CollideTeamMate", 40.0, pgid, Team, fieldConfig.getRobotSize()*2));

	// Do not plan outside playing field or in goal area
	heuristics.push_back(new InOwnGoalAreaHeuristic("InOwnGoalArea", 1000.0, pgid, plannerOptions, fieldConfig)); // Do not enter the inner Goal area

	// Stability heuristic: 5 weight
	heuristics.push_back(new InfluenceCurrentPositionsHeuristic("InfluenceCurrentPositions", 5, pgid, Team, fieldConfig.getMaxPossibleFieldDistance()));

	// DO not plan on top of opponent
	heuristics.push_back(new InfluenceOpponentsHeuristic("Influence Opponent", 5.0, pgid, Opponents, 0.7));


	heuristics.push_back(new InOppenentPenaltyAreaHeuristic("InOppenentPenalyArea", 100000, pgid, plannerOptions, fieldConfig));
	heuristics.push_back(new InOwnGoalAreaHeuristic("Own goal area", 100000, pgid, plannerOptions, fieldConfig));
	heuristics.push_back(new AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic("Already player assigned to own penalty area", 100000, pgid, Team, fieldConfig));
	heuristics.push_back(new AlreadyPlayerAssignedToOpponentPenaltyAreaHeuristic("Already player assigned to opponent penalty area", 100000, pgid, Team, fieldConfig));

	// create x,y grid for calculation. make 0,0 center of grid !!
	std::list<MRA::Geometry::Point> allowedTargetPositions = list<MRA::Geometry::Point>();
	for (int x_idx = 0; x_idx < x_grid_points; x_idx++) {
		for (int y_idx = 0; y_idx < y_grid_points; y_idx++) {
			double x = (x_idx * grid_size) - (x_grid_half * grid_size);
			double y = (y_idx * grid_size) - (y_grid_half * grid_size);
			if (y > -fieldConfig.getMaxFieldY() + fieldConfig.getRobotRadius()) { // only position where player is not over own backline
				allowedTargetPositions.push_back(MRA::Geometry::Point(x, y));
			}
		}
	}
	MRA::Geometry::Point pos = calculateGridValues(allowedTargetPositions, heuristics, plannerOptions, pgid);
	TeamPlanner_Grid::writeGridDataToFile(pgid, Team, Opponents, ball, plannerOptions, "InterceptorRestart", gridFileNumber);
	return pos;
}

//---------------------------------------------------------------------
/**
 * Method to find to most attractive position for attack support (receiving ball) .
 * A grid is created, for all points on the grid a heuristic (attractiveness) is calculated.
 * Most attractive position will be the offensive position
 */
bool TeamPlanner_Grid::findAttackSupportPosition(MRA::Geometry::Point& bestPosition, const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate,
		const PlannerOptions& plannerOptions, const MovingObject& ball,
		const std::vector<TeamPlannerOpponent>& Opponents, int gridFileNumber, const FieldConfig& fieldConfig,
		bool position_close_to_ball, bool teamControlBall)
{
    /* Position attack support, in order of priority (weighing):
     *
     *   + Do not block goal shot on opponent goal
     *   + Not too close to the ball
     *   + Not too close to opponents
     *   + stay in front of the ball on own half
     *   + if we control the ball make yourself available to receive a pass
     *   + Halfway on opponent half, good lobshot distance (also to have eyes on opponent half for ball detection)
     *   + not behind opponent (from ball position)
     *   + if ball on own half, stay on same side (left/right) as ball for better passing
     *   + Do not position close to team-mate (planned before attack support, higher priority), e.g. goalkeeper, interceptor
     *   + Do not position close to other attack_supporter(s)
     *   + Stability factor: do some minor preference to current location to avoid constant ¨flipping" between 2 almost equal positions
     *   + Do not plan outside playing field or in goal areas (inner box of penalty area)
     *   + Do not plan in own penalty area if already somebody assigned
	 */

	// define grid of 50 cm, only in field not outside the border
	double grid_size = plannerOptions.grid_size; //[m]
	double x_grid_half = floor(fieldConfig.getMaxFieldX()/grid_size);
	double y_grid_half = floor(fieldConfig.getMaxFieldY()/grid_size);
	int x_grid_points = 1 + 2 * x_grid_half;
	int y_grid_points = 1 + 2 * y_grid_half;

	double x_pos_ball = ball.getPosition().getPoint().x;
	double y_pos_ball = ball.getPosition().getPoint().y;

	/*
	 * Attack support position heuristics in open-play (offensive and defensive)
	 */

	PlannerGridInfoData pgid = PlannerGridInfoData();
	vector<GridHeuristic *> heuristics = vector<GridHeuristic *>();

	if (gamestate != game_state_e::PENALTY && gamestate != game_state_e::PENALTY_AGAINST) {
		// Do not block goal shot on opponent goal
		heuristics.push_back(new NotOnLineBetweenBallAndOpponentGoalHeuristic("Not between ball and opponent goal", 100.0, pgid,
				x_pos_ball, y_pos_ball, -fieldConfig.getGoalAreaWidth()*0.5, fieldConfig.getMaxFieldY(), +fieldConfig.getGoalAreaWidth()*0.5, fieldConfig.getMaxFieldY()));

  	    // Avoid difficult collaboration, not to close to the ball
		heuristics.push_back(new InfluenceBallHeuristic("InfluenceBall", 80.0, pgid, x_pos_ball, y_pos_ball, 4));
	}

	// apply penalty heuristics if needed
	handle_penalty_heuristics(gamestate, plannerOptions, Team, ball.getXYlocation(), fieldConfig, heuristics, pgid);


	// Avoid difficult collaboration, do not position close to opponents
	heuristics.push_back(new InfluenceOpponentsHeuristic("Influence Opponent", 50.0, pgid, Opponents, 1));

	heuristics.push_back(new InfluencePreviousAssignedPositionsHeuristic("previous assigned position", 100.0, pgid, Team, fieldConfig.getMaxPossibleFieldDistance(), dynamic_role_e::dr_ATTACKSUPPORTER));

	// stay in front of the ball on own half, so play forward on own half
	if (y_pos_ball < 0) {
		heuristics.push_back(new InSquareHeuristic("Play forward on own half", 50.0, pgid, -fieldConfig.getMaxFieldX(), y_pos_ball+1, fieldConfig.getMaxFieldX(), -fieldConfig.getMaxFieldY()) );
	}
	else {
		heuristics.push_back(new InSquareHeuristic("Play on opponent half", 50.0, pgid, -fieldConfig.getMaxFieldX(), 1, fieldConfig.getMaxFieldX(), -fieldConfig.getMaxFieldY()) );
	}

	// if we control the ball make yourself available to receive a pass
	if (teamControlBall) {
		heuristics.push_back(new InfluenceCurrentPositionsHeuristic("Receive pass when control ball", 40.0, pgid, Team, fieldConfig.getMaxPossibleFieldDistance()));
	}

	if (gamestate != game_state_e::PENALTY && gamestate != game_state_e::PENALTY_AGAINST) {
		//  position attack_support approx halfway on opponent half
		if (gamestate != game_state_e::KICKOFF && gamestate != game_state_e::KICKOFF_AGAINST) {
			// normally try to position on opponent half on a nice position to lob at the goal
			heuristics.push_back(new DesiredYSweeper("DesiredYAttackSupport", 30.0, pgid, (fieldConfig.getMaxFieldY()*0.5)-1, fieldConfig));
		}
		else {
			// in case of kick-off place robot at our own half
			heuristics.push_back(new DesiredYSweeper("DesiredYAttacksupport", 30.0, pgid, -1, fieldConfig));
		}

		// Avoid difficult collaboration, not behind opponent (from ball position)
		heuristics.push_back(new InterceptionThreatHeuristic("Interception threat", 20.0, pgid, ball, Team, Opponents,
				plannerOptions.interceptionChanceStartDistance, plannerOptions.interceptionChanceIncreasePerMeter,
				plannerOptions.interceptionChancePenaltyFactor));
	}



	// if ball is well on own half, stay on same side (left/right) as ball for better passing
	// else move to opposite side (left/right) to create room
	if (y_pos_ball < -1) {
		if (x_pos_ball < 0) {
			heuristics.push_back(new InSquareHeuristic("Left preference", 15.0, pgid, -2, fieldConfig.getMaxFieldY(), fieldConfig.getMaxFieldX(), -fieldConfig.getMaxFieldY()) );
		}
		else {
			heuristics.push_back(new InSquareHeuristic("Right preference", 15.0, pgid, -fieldConfig.getMaxFieldX(), fieldConfig.getMaxFieldY(), 2, -fieldConfig.getMaxFieldY()) );
		}
	}


	// Do not position close to team-mate path position
	heuristics.push_back(new CollideTeamMateHeuristic("CollideTeamMate", 10.0, pgid, Team, fieldConfig.getRobotSize()*2));

	// Do not position close to attack supporters path position
	std::vector<TeamPlannerRobot> AttackSupportTeam = vector<TeamPlannerRobot>();
	for (unsigned idx = 0; idx < Team.size(); idx++) {
		if (Team[idx].assigned  && Team[idx].dynamic_role == dynamic_role_e::dr_ATTACKSUPPORTER) {
			AttackSupportTeam.push_back(Team[idx]);
		}
	}
	heuristics.push_back(new CollideTeamMateHeuristic("Distance to AttackSupporters", 10.0, pgid, AttackSupportTeam, 4.0));



	// Do not plan in opponent penalty area
	heuristics.push_back(new InOppenentPenaltyAreaHeuristic("InOppenentPenalyArea", 1000.0, pgid, plannerOptions, fieldConfig));
	// heuristics.push_back(new AlreadyPlayerAssignedToOpponentPenaltyAreaHeuristic("Already player assigned to opponent penalty area", 1000.0, pgid, Team, fieldConfig));

	// Do not plan in own goal area
	heuristics.push_back(new InOwnGoalAreaHeuristic("InOwnGoalArea", 1000.0, pgid, plannerOptions, fieldConfig));

	// Do not plan in own penalty area if already somebody assigned
	heuristics.push_back(new AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic("Already player assigned to own penalty area", 1000.0, pgid, Team, fieldConfig));

	// This still needed?
	heuristics.push_back(new OutsidePlayFieldHeuristic("OutsidePlayField", 1000.0, pgid, fieldConfig, plannerOptions.attack_supporter_extra_distance_to_stay_from_sideline));

	// Stability heuristic
	heuristics.push_back(new InfluenceCurrentPositionsHeuristic("InfluenceCurrentPositions", 8.0, pgid, Team, fieldConfig.getMaxPossibleFieldDistance()));


	/*
	 * Additional attack support position heuristics in set-play (various)
	 * Normal sweeper can be deployed during a set-play with some additional positioning restrictions below
	 * Heuristic function itself checks on correct game-state to identify set-play
	 */

	// create x,y grid for calculation. make 0,0 center of grid !!
	std::list<MRA::Geometry::Point> allowedTargetPositions = list<MRA::Geometry::Point>();
	for (int x_idx = 0; x_idx < x_grid_points; x_idx++) {
		for (int y_idx = 0; y_idx < y_grid_points; y_idx++) {
			double x = (x_idx * grid_size) - (x_grid_half * grid_size);
			double y = (y_idx * grid_size) - (y_grid_half * grid_size);
			if (y > -fieldConfig.getMaxFieldY() + fieldConfig.getRobotRadius()) { // only position where player is not over own backline
				allowedTargetPositions.push_back(MRA::Geometry::Point(x, y));
			}
		}
	}


	bestPosition = calculateGridValues(allowedTargetPositions, heuristics, plannerOptions, pgid);
	TeamPlanner_Grid::writeGridDataToFile(pgid, Team, Opponents,ball, plannerOptions, "FindOffensive", gridFileNumber);

	bool prepare_phase = (gamestate == game_state_e::CORNER) || (gamestate == game_state_e::GOALKICK) ||
			             (gamestate == game_state_e::FREEKICK || (gamestate == game_state_e::THROWIN));
	if (plannerOptions.wait_on_non_optimal_position_during_prepare_phase && prepare_phase) {
		// find non optimal position to wait during prepare phase

		const unsigned nr_wait_positions = 8;   // check for 8 positions 1.5 meter around best position
		const double radius_non_opt_wait = 2.0;
		const double infield_margin = 0.25;   // distance to stay from side of field
		InterceptionThreatHeuristic InterceptionThreat = InterceptionThreatHeuristic("Interception threat", -18.0, pgid, ball, Team, Opponents,
				plannerOptions.interceptionChanceStartDistance, plannerOptions.interceptionChanceIncreasePerMeter,
				plannerOptions.interceptionChancePenaltyFactor);

		double bestX = bestPosition.x;
		double bestY = bestPosition.y;
		double lowestValue = std::numeric_limits<double>::infinity();
		// for all positions to check
		for (unsigned i = 0; i < nr_wait_positions; i++) {
			double angle = i * 2.0 * M_PI / static_cast<double>(nr_wait_positions);
			double xx = bestPosition.x + radius_non_opt_wait * cos(angle);
			double yy = bestPosition.y + radius_non_opt_wait * sin(angle);
			if (fieldConfig.isInField(xx,yy, infield_margin)) {
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
void TeamPlanner_Grid::writeGridDataToFile(PlannerGridInfoData& pgid, const std::vector<TeamPlannerRobot>& Team, const std::vector<TeamPlannerOpponent>& Opponents,
		const MovingObject& ball, const PlannerOptions& plannerOptions, const string& strSituation, int gridFileNumber) {

	if (plannerOptions.saveGridDataToFile) {
		string gridFileName = "";
		for (auto it = Team.begin(); it != Team.end(); ++it) {
			pgid.gameData.Team.push_back(it->position);
		}
		for (auto it = Opponents.begin(); it != Opponents.end(); ++it) {
			pgid.gameData.Opponents.push_back(it->position);
		}
		pgid.gameData.ball = ball;
		if (!plannerOptions.svgOutputFileName.empty()) {
			// svg filename provided
			std::stringstream stream("");
			stream << plannerOptions.svgOutputFileName.substr(0, plannerOptions.svgOutputFileName.size()-4) << "_" << strSituation << "_" <<  gridFileNumber << ".gpd";
			gridFileName = stream.str();
		}
		else {
			std::stringstream stream("");
			stream << "planner_grid_" << strSituation << "_" <<  gridFileNumber << ".gpd";
			gridFileName = stream.str();
		}
//		logAlways("save grid data to = %s", gridFileName.c_str(), __FILE__, __LINE__);
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


} /* namespace trs */




