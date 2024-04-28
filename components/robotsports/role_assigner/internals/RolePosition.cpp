/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */
#include "TeamPlannerGrid.hpp"  // grid related teamplanner functions
#include "RolePosition.hpp"
#include "planner_types.hpp"
#include "Dynamics.hpp"
#include <cmath>
#include <iostream>
#include "MathUtils.h"
#include <limits>
#include "logging.hpp"
#include "geometry.hpp"

using namespace std;
using namespace MRA;


MRA::Geometry::Point RolePosition::determineDynamicRolePosition(defend_info_t& rDefend_info, planner_target_e& planner_target, int& r_gridFileNumber, dynamic_role_e dynamic_role,
        TeamPlannerData& r_teamplannerData, bool playerPassedBall, bool& r_role_position_is_end_position_of_pass) {
	MRA::Geometry::Point rolePosition = MRA::Geometry::Point(0,0);
	rDefend_info.defending_id = -1;	// no item to defend
	rDefend_info.valid = false;
	r_role_position_is_end_position_of_pass = false;

	if (r_teamplannerData.parameters.man_to_man_defense_during_setplay_against &&
		(r_teamplannerData.gamestate == game_state_e::GOALKICK_AGAINST || r_teamplannerData.gamestate == game_state_e::CORNER_AGAINST ||
		        r_teamplannerData.gamestate == game_state_e::FREEKICK_AGAINST || r_teamplannerData.gamestate == game_state_e::THROWIN_AGAINST 	) )
	{
		// Setplay against: man coverage for all opponents
		rolePosition = calculateManToManDefensePosition(rDefend_info, dynamic_role, r_teamplannerData, r_gridFileNumber, true /* setplay active */);
		return rolePosition;
	}


	switch (dynamic_role) {
	case dr_PENALTY_KICKER: {
		//		In RoboCup, the following modification apply:
		//		• The defending goal keeper stays within his own goal area until the ball
		//		is even slightly moved.
		//		• The goalie can move at any time as long as it does no leave its goal area.
		//		• In case of an end-of-game penalty sequence, the kicker is located inside
		//		the center circle. For an in-game penalty the kicker is located on a line
		//		from the penalty spot to the center circle, approximately one meter from
		//		the penalty spot.
		//		• In case of an end-of-game penalty sequence, the players other than the
		//		kicker and goalie are located inside the field of play, outside the center
		//		circle, and behind the center line on the opposite side of the defending
		//		goal keeper. For an in-game penalty, the players other than the kicker
		//		and goalie are located inside the field of play, at least three meters away
		//		from the penalty spot and not in the penalty area.
		if (r_teamplannerData.gamestate == PENALTY_SHOOTOUT) {
			// position in top of center circle
			rolePosition = MRA::Geometry::Point(0, r_teamplannerData.fieldConfig.getCenterCirleRadius() - r_teamplannerData.fieldConfig.getRobotSize() );
		}
		else
		{
			MRA::Geometry::Point goalPos = MRA::Geometry::Point(0, r_teamplannerData.fieldConfig.getMaxFieldY());
			MRA::Geometry::Point ballPos = r_teamplannerData.ball.position;
			if (r_teamplannerData.fieldConfig.isInField(ballPos.x, ballPos.y, r_teamplannerData.parameters.outsideFieldMargin))
			{
				double alfa = ballPos.angle(goalPos);
				double behind_ball_x = ballPos.x + (cos(alfa) * 1.0);
				double behind_ball_y = ballPos.y + (sin(alfa) * 1.0);
				rolePosition = MRA::Geometry::Point(behind_ball_x,behind_ball_y);
			}
			else
			{
				rolePosition = MRA::Geometry::Point(0, r_teamplannerData.fieldConfig.getMaxFieldY() - r_teamplannerData.fieldConfig.getPenaltySpotToBackline() - 1.0);
			}
		}
	}
	break;
	case dr_SETPLAY_KICKER: {
		// CODE only valid for set-play !!!
		// calculation assume that no opponent or team mate is at calculated position. This is only true for restart.
		// Known issues with algorithm
		// - not take time allowed in penalty area into account.

		MRA::Geometry::Point shooterPosition;
		MRA::Geometry::Point receiverPosition;
		// Get ball position, will be the base of the positioning
		calculateSetPlayPosition(shooterPosition, receiverPosition, r_teamplannerData);
		rolePosition = shooterPosition;  // right of field, just middle of opponent half in Y
	}
	break;
	case dr_SETPLAY_RECEIVER: {
		// CODE only valid for set-play
		// put setplay-receiver at a location for the specific setplay, e.g. freekick at own half put it at opponent half
		// calculation assume that no opponent or team mate is at calculated position. This is only true for restart.
		// Known issues with algorithm
		// - not take time allowed in penalty area into account.
		MRA::Geometry::Point shooterPosition;
		MRA::Geometry::Point receiverPosition;
		// Get ball position, will be the base of the positioning
		calculateSetPlayPosition(shooterPosition, receiverPosition, r_teamplannerData);
		rolePosition = receiverPosition;  // right of field, just middle of opponent half in Y
	}
	break;
	case dr_PENALTY_DEFENDER:
	{
			rolePosition = TeamPlanner_Grid::findDefensivePositionDuringPenaltyShootOut(r_teamplannerData, r_gridFileNumber++);
	}
	break;
	case dr_DEFENDER:
	{
		if (r_teamplannerData.parameters.man_to_man_defense_during_normal_play) {
			// man coverage
			rolePosition = calculateManToManDefensePosition(rDefend_info, dynamic_role, r_teamplannerData, r_gridFileNumber, false /* normal-play */);
		}
		else {
			rolePosition = TeamPlanner_Grid::findDefensivePosition(r_teamplannerData, r_gridFileNumber++);
		}
	}
	break;
	case dr_ATTACKSUPPORTER: {
		bool position_close_to_ball = false;
		if (r_teamplannerData.gamestate == game_state_e::GOALKICK) {
			position_close_to_ball = true;
		}
		Geometry::Point ballPostionToUse = r_teamplannerData.ball.position;
		if (r_teamplannerData.parameters.use_pass_to_position_for_attack_support && r_teamplannerData.pass_data.valid) {
			ballPostionToUse.x = r_teamplannerData.pass_data.target_pos.x;
			ballPostionToUse.y = r_teamplannerData.pass_data.target_pos.y;
		}

		// check if already player moves to ball target position
		bool target_player_for_pass_is_present = false;
		for (unsigned ap_idx = 0; ap_idx < r_teamplannerData.team.size(); ap_idx++) {
			if (r_teamplannerData.team[ap_idx].assigned and r_teamplannerData.team[ap_idx].robotId == r_teamplannerData.pass_data.target_id)  {
				// player is target for the pass
			    target_player_for_pass_is_present = true;
			}
		}
		if ((r_teamplannerData.pass_data.valid) &&
			(r_teamplannerData.pass_data.target_id != 0 /* not shot on goal */) &&
			(! target_player_for_pass_is_present)) {
			// ball passed and no other player moves to the ball, go to ball target position
			rolePosition = MRA::Geometry::Point(r_teamplannerData.pass_data.target_pos.x, r_teamplannerData.pass_data.target_pos.y);
			r_role_position_is_end_position_of_pass = true;
		} else {
			TeamPlanner_Grid::findAttackSupportPosition(rolePosition, r_teamplannerData, ballPostionToUse, r_gridFileNumber++, position_close_to_ball);
		}
	}
	break;
	case dr_SWEEPER: {
		rolePosition = TeamPlanner_Grid::findSweeperPosition(r_teamplannerData, r_gridFileNumber++);
	}
	break;
	case dr_INTERCEPTOR: {
		if (r_teamplannerData.gamestate == game_state_e::GOALKICK_AGAINST || r_teamplannerData.gamestate == game_state_e::CORNER_AGAINST ||
		    r_teamplannerData.gamestate == game_state_e::DROPPED_BALL || r_teamplannerData.gamestate == game_state_e::FREEKICK_AGAINST ||
		    r_teamplannerData.gamestate == game_state_e::PENALTY_AGAINST || r_teamplannerData.gamestate == game_state_e::PENALTY_SHOOTOUT_AGAINST ||
		    r_teamplannerData.gamestate == game_state_e::THROWIN_AGAINST 	)
		{
			rolePosition = TeamPlanner_Grid::findInterceptorPositionDuringRestart(r_teamplannerData, r_gridFileNumber++);
		}
		else {
			MRA::Geometry::Point intercept_position = r_teamplannerData.ball.position;
			if (r_teamplannerData.parameters.interceptor_assign_use_ball_velocity &&
			        r_teamplannerData.ball.velocity.size() > r_teamplannerData.parameters.interceptor_assign_min_velocity_for_calculate_interception_position) {
				// ball is moving: calculate distance to interception position per robot.
				double maxSpeed = r_teamplannerData.parameters.maxPossibleLinearSpeed;
				double bestIntercept_distance = std::numeric_limits<double>::infinity();
				for (unsigned int idx = 0; idx < r_teamplannerData.team.size(); idx++) {
					if (!r_teamplannerData.team[idx].assigned) {
						// Find initial intercept point
						MRA::Geometry::Point player_pos = r_teamplannerData.team[idx].position;
						Dynamics::dynamics_t intercept_data = Dynamics::interceptBall(r_teamplannerData.ball, player_pos, maxSpeed,
						        r_teamplannerData.fieldConfig, r_teamplannerData.parameters.move_to_ball_left_field_position);
						if (player_pos.distanceTo(intercept_data.intercept_position) < bestIntercept_distance) {
							bestIntercept_distance = player_pos.distanceTo(intercept_data.intercept_position);
							intercept_position = intercept_data.intercept_position;
						}
					}
				}
				rolePosition = intercept_position;
			}
			else {
				rolePosition = InterceptorNormalPlayPosition(planner_target, r_teamplannerData);
			}

			int closest_to_ball_idx = FindOpponentClostestToPositionAndNotAssigned(intercept_position, r_teamplannerData);
			if (closest_to_ball_idx >= 0) {
				// Found opponent closest to the ball. update administration that the interceptor defense this opponent.
			    r_teamplannerData.opponents[closest_to_ball_idx].assigned = true;
			}
		}
	}
	break;
	case dr_BALLPLAYER: {
		MRA::Geometry::Point shootPos = TeamPlanner_Grid::findBallPlayerPosition(r_teamplannerData, r_gridFileNumber++);
		rolePosition = shootPos;
	}
	break;
	case dr_GOALKEEPER: // empty
	case dr_SEARCH_FOR_BALL: // empty: only used for diagnostics
	case dr_BEGIN_POSITION: // empty: only used for diagnostics
	case dr_PARKING: // empty: only used for diagnostics
	case dr_LOB_CALIBRATION:
	case dr_NONE: {

	}
	break;
	}
	return rolePosition;
}

MRA::Geometry::Point RolePosition::setplay_receiver_position_90deg_to_ball_goal(const TeamPlannerData& r_teamplannerData)
{
	// select position 90 degrees to the line between the ball and the opponent goal
	MRA::Geometry::Position receiverPosition;
	double distBallReciever = r_teamplannerData.parameters.restart_receiver_ball_dist; // distance between receiver and ball
	MRA::Geometry::Position goalPos = MRA::Geometry::Position(0, r_teamplannerData.fieldConfig.getMaxFieldY());
	double alfa = Geometry::Point(r_teamplannerData.ball.position.x, r_teamplannerData.ball.position.y).angle(goalPos);

	double outsideFieldMargin = r_teamplannerData.parameters.outsideFieldMargin; // distance to stay from side of field
	double recieveX_plus_90deg = r_teamplannerData.ball.position.x + (cos(alfa + 0.5 * M_PI) * distBallReciever);
	double recieveY_plus_90deg = r_teamplannerData.ball.position.y + (sin(alfa + 0.5 * M_PI) * distBallReciever);
	double recieveX_min_90deg = r_teamplannerData.ball.position.x  + (cos(alfa - 0.5 * M_PI) * distBallReciever);
	double recieveY_min_90deg = r_teamplannerData.ball.position.y  + (sin(alfa - 0.5 * M_PI) * distBallReciever);
	bool min90_inField = r_teamplannerData.fieldConfig.isInField(recieveX_min_90deg, recieveY_min_90deg, outsideFieldMargin);
	bool plus90_inField = r_teamplannerData.fieldConfig.isInField(recieveX_plus_90deg, recieveY_plus_90deg, outsideFieldMargin);
	if (min90_inField && plus90_inField) {
		// both positions are in the field, select the one closest to opponent goal.
		if (recieveY_min_90deg > recieveY_plus_90deg) {
			receiverPosition = MRA::Geometry::Position(recieveX_min_90deg, recieveY_min_90deg);
		}
		else {
			receiverPosition = MRA::Geometry::Position(recieveX_plus_90deg, recieveY_plus_90deg);
		}
	} else if (min90_inField) {
		receiverPosition = MRA::Geometry::Position(recieveX_min_90deg, recieveY_min_90deg);
	} else {
		receiverPosition = MRA::Geometry::Position(recieveX_plus_90deg, recieveY_plus_90deg);
	}

	// in corner area it might go wrong and a Y position outside the field is selected.
	if (!r_teamplannerData.fieldConfig.isInField(receiverPosition.x, receiverPosition.y, outsideFieldMargin)) {
		if (!r_teamplannerData.fieldConfig.isInField(0, receiverPosition.y,	outsideFieldMargin)) {
			// y-location is outside the field.
			if (recieveY_min_90deg > recieveY_plus_90deg) {
				receiverPosition = MRA::Geometry::Position(recieveX_plus_90deg,
						recieveY_plus_90deg);
			} else {
				receiverPosition = MRA::Geometry::Position(recieveX_min_90deg,
						recieveY_min_90deg);
			}
		}
		// x-position can be field
		if (receiverPosition.x < -(r_teamplannerData.fieldConfig.getMaxFieldX() - outsideFieldMargin)) {
			receiverPosition.x = -(r_teamplannerData.fieldConfig.getMaxFieldX()	- outsideFieldMargin);
		}
		if (receiverPosition.x > (r_teamplannerData.fieldConfig.getMaxFieldX() - outsideFieldMargin)) {
			receiverPosition.x = (r_teamplannerData.fieldConfig.getMaxFieldX() - outsideFieldMargin);
		}
	}

	return receiverPosition;
}

// ------------------------------------------------------------------
bool RolePosition::calculateSetPlayReceiverMinTurnPosition(const TeamPlannerData& r_teamplannerData, MRA::Geometry::Point& receiverPosition) {
	bool found = false;

	// no straight lob-shot possible
	double outsideFieldMargin = r_teamplannerData.parameters.outsideFieldMargin; // distance to stay from side of field
	double distBallReciever = r_teamplannerData.parameters.restart_receiver_ball_dist; // distance between receiver and ball
	double sol1_x, sol1_y, sol2_x, sol2_y = 0;
	double maxFieldX = r_teamplannerData.fieldConfig.getMaxFieldX();
	double maxFieldY = r_teamplannerData.fieldConfig.getMaxFieldY();
	int nr_crossings = 0;
	double backline_y = -r_teamplannerData.fieldConfig.getMaxFieldY() + outsideFieldMargin;
	// check crossings with last y-line where receiver putting back is allowed.
	nr_crossings = findLineCircleIntersections(r_teamplannerData.ball.position.x,
	        r_teamplannerData.ball.position.y, distBallReciever, -maxFieldX, backline_y,
			maxFieldX, backline_y, sol1_x, sol1_y, sol2_x, sol2_y);
	if (nr_crossings > 0) {
		// at least 1 intersection found the last line
		if (nr_crossings == 1) {
			if (r_teamplannerData.fieldConfig.isInField(sol1_x, sol1_y, outsideFieldMargin)) {
				receiverPosition = MRA::Geometry::Point(sol1_x, sol1_y);
				found = true;
			}
		} else {
			// check if both intersections are in the field
			bool sol1_inField = r_teamplannerData.fieldConfig.isInField(sol1_x, sol1_y,
					outsideFieldMargin)
					&& !r_teamplannerData.fieldConfig.isInOwnGoalArea(sol1_x, sol1_y);
			bool sol2_inField = r_teamplannerData.fieldConfig.isInField(sol2_x, sol2_y,
					outsideFieldMargin)
					&& !r_teamplannerData.fieldConfig.isInOwnGoalArea(sol2_x, sol2_y);
			if (sol1_inField && sol2_inField) {
				// both intersections are in field, select position closes to the side-line
				if (fabs(sol1_x) < fabs(sol2_x)) {
					receiverPosition = MRA::Geometry::Point(sol2_x, sol2_y);
					found = true;
				} else {
					receiverPosition = MRA::Geometry::Point(sol1_x, sol1_y);
					found = true;
				}
			} else if (sol1_inField) {
				receiverPosition = MRA::Geometry::Point(sol1_x, sol1_y);
				found = true;
			} else if (sol2_inField) {
				receiverPosition = MRA::Geometry::Point(sol2_x, sol2_y);
				found = true;
			}
		}
	} else {
		// no intersection with last line
		// Check intersection with side-lines
		// x-position is closest side-line with some margin
		double side_sol1_x, side_sol1_y, side_sol2_x, side_sol2_y = 0;
		double penalty_sol1_x, penalty_sol1_y, penalty_sol2_x, penalty_sol2_y =
				0;
		double penalty2_sol1_x, penalty2_sol1_y, penalty2_sol2_x,
				penalty2_sol2_y = 0;
		int closest_direction = 1;
		if (r_teamplannerData.ball.position.x < 0) {
			closest_direction = -closest_direction; // ball is left on the field
		}
		double closest_sidelineX = closest_direction
				* (maxFieldX - outsideFieldMargin);
		// find intersection with closest sideline
		int nr_crossings_side = findLineCircleIntersections(
		        r_teamplannerData.ball.position.x, r_teamplannerData.ball.position.y,
				distBallReciever, closest_sidelineX, -maxFieldY,
				closest_sidelineX, maxFieldY, side_sol1_x, side_sol1_y,
				side_sol2_x, side_sol2_y);
		double paw = r_teamplannerData.fieldConfig.getPenaltyAreaWidth();
		double pal = r_teamplannerData.fieldConfig.getPenaltyAreaLength();
		double dist_to_side_penalty_area = 0.5 * paw
				+ r_teamplannerData.parameters.setplay_margin_to_penalty_area_side;
		// find intersection with penalty area. side and top
		int nr_crossings_penalty = findLineCircleIntersections(
		        r_teamplannerData.ball.position.x, r_teamplannerData.ball.position.y,
				distBallReciever, closest_direction * dist_to_side_penalty_area,
				-maxFieldY, closest_direction * dist_to_side_penalty_area,
				-(maxFieldY - pal), penalty_sol1_x, penalty_sol1_y,
				penalty_sol2_x, penalty_sol2_y);
		int nr_crossings_penalty2 = findLineCircleIntersections(
		        r_teamplannerData.ball.position.x, r_teamplannerData.ball.position.y,
				distBallReciever, -paw, -(maxFieldY - pal), +paw,
				-(maxFieldY - pal), penalty2_sol1_x, penalty2_sol1_y,
				penalty2_sol2_x, penalty2_sol2_y);
		double best_X = maxFieldX;
		double best_Y = maxFieldY;
		if (nr_crossings_side > 0) {
			if (r_teamplannerData.fieldConfig.isInField(side_sol1_x, side_sol1_y,
					outsideFieldMargin) && side_sol1_y < best_Y
					&& !r_teamplannerData.fieldConfig.isInOwnGoalArea(sol1_x, sol1_y)) {
				best_X = side_sol1_x;
				best_Y = side_sol1_y;
				found = true;
			}
		}
		if (nr_crossings_side > 1) {
			if (r_teamplannerData.fieldConfig.isInField(side_sol2_x, side_sol2_y,
					outsideFieldMargin) && side_sol2_y < best_Y
					&& !r_teamplannerData.fieldConfig.isInOwnGoalArea(sol2_x, sol2_y)) {
				best_X = side_sol2_x;
				best_Y = side_sol2_y;
				found = true;
			}
		}
		if (nr_crossings_penalty > 0) {
			if (r_teamplannerData.fieldConfig.isInField(penalty_sol1_x, penalty_sol1_y, 0)
					&& penalty_sol1_y < best_Y
					&& !r_teamplannerData.fieldConfig.isInOwnGoalArea(sol1_x, sol1_y)) {
				best_X = penalty_sol1_x;
				best_Y = penalty_sol1_y;
				found = true;
			}
		}
		if (nr_crossings_penalty > 1) {
			if (r_teamplannerData.fieldConfig.isInField(penalty_sol2_x, penalty_sol2_y, 0)
					&& penalty_sol2_y < best_Y
					&& !r_teamplannerData.fieldConfig.isInOwnGoalArea(sol2_x, sol2_y)) {
				best_X = penalty_sol2_x;
				best_Y = penalty_sol2_y;
				found = true;
			}
		}
		if (nr_crossings_penalty2 > 0) {
			if (r_teamplannerData.fieldConfig.isInField(penalty2_sol1_x, penalty2_sol1_y, 0)
					&& penalty2_sol1_y < best_Y
					&& !r_teamplannerData.fieldConfig.isInOwnGoalArea(sol2_x, sol2_y)) {
				best_X = penalty2_sol1_x;
				best_Y = penalty2_sol1_y;
				found = true;
			}
		}
		if (nr_crossings_penalty2 > 1) {
			if (r_teamplannerData.fieldConfig.isInField(penalty2_sol2_x, penalty2_sol2_y, 0)
					&& penalty2_sol2_y < best_Y
					&& !r_teamplannerData.fieldConfig.isInOwnGoalArea(sol2_x, sol2_y)) {
				best_X = penalty2_sol2_x;
				best_Y = penalty2_sol2_y;
				found = true;
			}
		}
		if (found) {
			receiverPosition = MRA::Geometry::Point(best_X, best_Y);
		}
	}
	return found;
} // end of calculateSetPlayReceiverMinTurnPosition

// ------------------------------------------------------------------
bool RolePosition::calculateSetPlayReceiverOnLobShotLinePosition(const TeamPlannerData& r_teamplannerData, MRA::Geometry::Point& receiverPosition) {
	bool found = false;

	// no straight lob-shot possible
	double outsideFieldMargin = r_teamplannerData.parameters.outsideFieldMargin; // distance to stay from side of field
	double distBallReciever = r_teamplannerData.parameters.restart_receiver_ball_dist; // distance between receiver and ball
	double sol1_x, sol1_y, sol2_x, sol2_y = 0;
	double maxFieldX = r_teamplannerData.fieldConfig.getMaxFieldX();
	int nr_crossings = 0;
	double lobshot_line = r_teamplannerData.parameters.min_y_for_lob_shot;
	// check crossings with lobshot line where receiver putting back is allowed.
	nr_crossings = findLineCircleIntersections(r_teamplannerData.ball.position.x,
	        r_teamplannerData.ball.position.y, distBallReciever, -maxFieldX, lobshot_line,
			maxFieldX, lobshot_line, sol1_x, sol1_y, sol2_x, sol2_y);
	if (nr_crossings > 0) {
		// at least 1 intersection found the lobshot line
		if (nr_crossings == 1) {
			if (r_teamplannerData.fieldConfig.isInField(sol1_x, sol1_y, outsideFieldMargin)) {
				receiverPosition = MRA::Geometry::Position(sol1_x, sol1_y);
				found = true;
			}
		} else {
			// check if both intersections are in the field
			bool sol1_inField = r_teamplannerData.fieldConfig.isInField(sol1_x, sol1_y,
					outsideFieldMargin)
					&& !r_teamplannerData.fieldConfig.isInOwnGoalArea(sol1_x, sol1_y);
			bool sol2_inField = r_teamplannerData.fieldConfig.isInField(sol2_x, sol2_y,
					outsideFieldMargin)
					&& !r_teamplannerData.fieldConfig.isInOwnGoalArea(sol2_x, sol2_y);
			if (sol1_inField && sol2_inField) {
				// both intersections are in field, select position closes to the side-line
				if (fabs(sol1_x) < fabs(sol2_x)) {
					receiverPosition = MRA::Geometry::Position(sol2_x, sol2_y);
					found = true;
				} else {
					receiverPosition = MRA::Geometry::Position(sol1_x, sol1_y);
					found = true;
				}
			} else if (sol1_inField) {
				receiverPosition = MRA::Geometry::Position(sol1_x, sol1_y);
				found = true;
			} else if (sol2_inField) {
				receiverPosition = MRA::Geometry::Position(sol2_x, sol2_y);
				found = true;
			}
		}
	}
	return found;
} // End of calculateSetPlayReceiverOnLobShotLinePosition


// ---------------------------------------------------------------------
bool RolePosition::calculateSetPlayReceiverConservativePosition(const TeamPlannerData& r_teamplannerData, MRA::Geometry::Point& receiverPosition)
{
	bool found = false;
	double outsideFieldMargin = r_teamplannerData.parameters.outsideFieldMargin; // distance to stay from side of field
	double distBallReciever = r_teamplannerData.parameters.restart_receiver_ball_dist; // distance between receiver and ball

	// ball between our backline and lobshot line.
	// place robot on same y as ball, preferable to nearest sideline.
	int closest_direction = 1;
	if (r_teamplannerData.ball.position.x < 0) {
		closest_direction = -closest_direction; // ball is left on the field
	}
	double closest_sidelineX = r_teamplannerData.ball.position.x + (closest_direction * distBallReciever);
	if (r_teamplannerData.fieldConfig.isInField(closest_sidelineX, r_teamplannerData.ball.position.y, outsideFieldMargin)) {
		// place receiver towards sideline
		receiverPosition = MRA::Geometry::Position(r_teamplannerData.ball.position.x + (closest_direction * distBallReciever), r_teamplannerData.ball.position.y);
	}
	else {
		// place receiver towards center of field
		receiverPosition = MRA::Geometry::Position(r_teamplannerData.ball.position.x - (closest_direction * distBallReciever), r_teamplannerData.ball.position.y);
	}
	found = true;
	return found;
}

// ---------------------------------------------------------------------
void RolePosition::calculateSetPlayPosition(MRA::Geometry::Point& shooterPosition, MRA::Geometry::Point& receiverPosition, const TeamPlannerData& r_teamplannerData)  {


	receiverPosition = calculateSetPlayReceiverPosition(r_teamplannerData);

	// calculate kicker position on line between ball and receiver
	// and check if position of shoot is reachable, otherwise try other found alternative position
	double distBallShooter = r_teamplannerData.parameters.restart_shooter_ball_dist; // distance between shooter and ball
	double distBallReciever = r_teamplannerData.parameters.restart_receiver_ball_dist; // distance between receiver and ball

	double alfaKicker = Geometry::Point(r_teamplannerData.ball.position.x, r_teamplannerData.ball.position.y).angle(receiverPosition);
	bool shootPositionValid = false; // indicate if kicker position is valid
	double shootX = 0.0;
	double shootY = 0.0;
	double actualDistBallShooter = distBallShooter;

	while (shootPositionValid == false) {
		double distShooterReceiver = actualDistBallShooter + distBallReciever;
		shootX = receiverPosition.x + (cos(alfaKicker) * distShooterReceiver);
		shootY = receiverPosition.y + (sin(alfaKicker) * distShooterReceiver);
		if (r_teamplannerData.fieldConfig.isInReachableField(shootX, shootY)) {
			// shooter position is reachable for player
			shootPositionValid = true;
		}
		else {
			// shooter position is not reachable for player
			// adjust position for the shooter

			actualDistBallShooter = actualDistBallShooter - 0.10; // try again with distance to ball 10 cm shorter.
			if (actualDistBallShooter < (r_teamplannerData.fieldConfig.getRobotRadius() + 0.10) ) {
				// can not move close to ball then 10 cm from the front of the robot.
				// use same x position as ball and position the front of the player cm from the ball
				shootX = r_teamplannerData.ball.position.x;
				shootY = r_teamplannerData.ball.position.y - r_teamplannerData.fieldConfig.getRobotSize();
				shootPositionValid = true;
			}
		}
	}
	shooterPosition = MRA::Geometry::Position(shootX, shootY);
}

MRA::Geometry::Point RolePosition::calculateSetPlayReceiverPosition(const TeamPlannerData& r_teamplannerData)
{
	// CODE only valid for replay !!!
	// calculation assume that no opponent or team mate is at calculated position. This is only true for restart.
	// Ball position will be the base of the positioning.
	// Default players will be on the line between ball and opponent goal
	// 	Except if ball is close to the back-line. The player will be on the line between the ball and half of the opponent half (par example during corner)
	//  Step 1: calculate position for receiver (as described above)
	//  If the best position of the receiver is in the penalty area (or behind the goal)
	//          then the receiver will be located on top of the penalty area.
	//  Step 2: locate shooter on the line between ball  and receiver.

	MRA::Geometry::Point receiverPosition;

	const double margin_to_side_line = 0.5;
	double distBallReciever = r_teamplannerData.parameters.restart_receiver_ball_dist; // distance between receiver and ball
	MRA::Geometry::Position goalPos = MRA::Geometry::Position(0, r_teamplannerData.fieldConfig.getMaxFieldY());
	double topPenaltyAreaY = r_teamplannerData.fieldConfig.getMaxFieldY()- (r_teamplannerData.fieldConfig.getPenaltyAreaLength() + r_teamplannerData.fieldConfig.getRobotRadius());
	// create situation for lob shot: receiver on line between ball and goal x meter behind ball facing to goal.
	// This only when the ball is more then x meter on opponent half (otherwise lob shot on goal is not allowed).
	// if location is outside the field, then the player will be located inside the field.
	double outsideFieldMargin = r_teamplannerData.parameters.outsideFieldMargin; // distance to stay from side of field

	MRA::Geometry::Position previousEndPos;
	for (unsigned int idx = 0; idx < r_teamplannerData.team.size(); idx++) {
		if (r_teamplannerData.team[idx].previous_result.previous_result_present == 1 && static_cast<dynamic_role_e>(r_teamplannerData.team[idx].previous_result.dynamic_role) == dr_SETPLAY_RECEIVER) {
			MRA::Geometry::Position previousEndPos = MRA::Geometry::Position(r_teamplannerData.team[idx].previous_result.end_position.x, r_teamplannerData.team[idx].previous_result.end_position.y);
			MRA::Geometry::Position previousBall = MRA::Geometry::Position(r_teamplannerData.previous_ball.x, r_teamplannerData.previous_ball.y);

			if (r_teamplannerData.ball.position.distanceTo(previousBall) < 0.26) {
				receiverPosition = previousEndPos;
				receiverPosition.x = receiverPosition.x - (r_teamplannerData.ball.position.x - r_teamplannerData.previous_ball.x);
				receiverPosition.y = receiverPosition.y - (r_teamplannerData.ball.position.y - r_teamplannerData.previous_ball.y);
				if (r_teamplannerData.fieldConfig.isInField(receiverPosition, margin_to_side_line)) {
					return receiverPosition;
				}
			}
		}
	}

	// Prefer lobshot when possible
	// Default position is in line with ball and opponent goal.
	double alfa = Geometry::Point(r_teamplannerData.ball.position.x, r_teamplannerData.ball.position.y).angle(goalPos);
	double behind_ball_x = r_teamplannerData.ball.position.x + (cos(alfa) * distBallReciever);
	double behind_ball_y = r_teamplannerData.ball.position.y + (sin(alfa) * distBallReciever);
	double min_y_for_default_position = r_teamplannerData.parameters.min_y_for_lob_shot;
	bool found = false;
	bool default_in_field = r_teamplannerData.fieldConfig.isInField(behind_ball_x, behind_ball_y, outsideFieldMargin);
	bool default_in_own_penalty_area = r_teamplannerData.fieldConfig.isInOwnPenaltyArea(behind_ball_x, behind_ball_y, outsideFieldMargin);

	if (default_in_field && !default_in_own_penalty_area && behind_ball_y > min_y_for_default_position) {
		// lob-shot: receiver will be located at opponent half and within the field
		receiverPosition = MRA::Geometry::Position(behind_ball_x, behind_ball_y);
		found = true;
	}
	else {
		// no straight lob-shot possible
		found = calculateSetPlayReceiverMinTurnPosition(r_teamplannerData, receiverPosition);
		if (found && receiverPosition.y < r_teamplannerData.parameters.min_y_for_lob_shot)
		{
			if (r_teamplannerData.ball.position.y > r_teamplannerData.parameters.min_y_for_lob_shot) {
				// ball is close to <plannerOptions.min_y_for_lob_shot>, but not behind it.
				// place receiver on the line <plannerOptions.min_y_for_lob_shot>
				found = calculateSetPlayReceiverOnLobShotLinePosition(r_teamplannerData, receiverPosition);
				if (!found) {
					// could not find valid position on on the line <plannerOptions.min_y_for_lob_shot>
					found = calculateSetPlayReceiverConservativePosition(r_teamplannerData, receiverPosition);
				}
			}
			else {
				// ball is between <plannerOptions.min_y_for_lob_shot> and our backline
				found = calculateSetPlayReceiverConservativePosition(r_teamplannerData, receiverPosition);
			}
		}
		else {
			if ((r_teamplannerData.ball.position.y > r_teamplannerData.parameters.min_y_for_lob_shot) &&
			    fabs(r_teamplannerData.ball.position.y - r_teamplannerData.parameters.min_y_for_lob_shot) < distBallReciever) {
				// check if place on lobshot line is possible
				found = calculateSetPlayReceiverOnLobShotLinePosition(r_teamplannerData, receiverPosition);

			}
		}
	}

	if (!found) {
		// no good position found, use original 90 degree position
		// select position 90 degrees to the line between the ball and the opponent goal
		receiverPosition = setplay_receiver_position_90deg_to_ball_goal(r_teamplannerData);
	}

	// A position for the setplay receiver has be calculated.
	// Avoid position where the setplay receiver is positioned in front of own goal.
	// If the setplay receiver position is in front of own goal then calculate a safer position for the setplay receiver.
	// The setplay receiver should not be position in area between side of penalty area and middle circle at own half.
	// Then position the setplay receiver to nearest side-line.
	if (((fabs(receiverPosition.x) < (r_teamplannerData.fieldConfig.getPenaltyAreaWidth() * 0.5)) &&
		 (receiverPosition.y < -(r_teamplannerData.fieldConfig.getCenterCirleDiameter()*0.5))))
	{
		// Update receiver location because it is in front of own goal

		// find nearest side-line
		int closest_direction = 1;
		if (r_teamplannerData.ball.position.x < 0) {
			closest_direction = -closest_direction; // ball is left on the field
		}
		double closest_sidelineX = closest_direction * (r_teamplannerData.fieldConfig.getMaxFieldX() - margin_to_side_line);

		// calculate intersection points of the nearest side-line with circle with radius distBallReciever with ball as center
		// use one of the intersection points as new position (position must be in the field)
		double intersection1x, intersection1y, intersection2x, intersection2y = 0.0;
		int nr_intersections_with_sideline = findLineCircleIntersections(
		        r_teamplannerData.ball.position.x, r_teamplannerData.ball.position.y,distBallReciever,
				closest_sidelineX, r_teamplannerData.fieldConfig.getMaxFieldY(),
				closest_sidelineX, -r_teamplannerData.fieldConfig.getMaxFieldY(),
				intersection1x, intersection1y, intersection2x, intersection2y);
		// if no intersections are found: then keep original result, but this should not occur

		if (nr_intersections_with_sideline == 1) {
			if (r_teamplannerData.fieldConfig.isInField(intersection1x, intersection1y, margin_to_side_line)) {
				receiverPosition = MRA::Geometry::Position(intersection1x, intersection1y); // select the only intersection
			}
		}
		else if (nr_intersections_with_sideline == 2) {
			// two intersections: select the intersection closest to the middle line (check if position is in the field).
			if ((intersection2y < intersection1y) && r_teamplannerData.fieldConfig.isInField(intersection1x, intersection1y, margin_to_side_line)){
				receiverPosition = MRA::Geometry::Position(intersection1x, intersection1y);
			}
			else{
				if (r_teamplannerData.fieldConfig.isInField(intersection2x, intersection2y, margin_to_side_line)) {
					receiverPosition = MRA::Geometry::Position(intersection2x, intersection2y);
				}
			}
		}
	}

	if (r_teamplannerData.fieldConfig.isInOpponentPenaltyArea(receiverPosition.x, receiverPosition.y) ) {
		// receiver position is in Opponent Goal area AND game-state is not penalty
		// Then place the receiver on the line on top of the opponent penalty area
		double preffered_Y_receiver = topPenaltyAreaY;
		double a = preffered_Y_receiver - r_teamplannerData.ball.position.y;
		double relX = sqrt(distBallReciever*distBallReciever - a*a);
		// two positions are possible, select position closest to middle of the field
		if (fabs(r_teamplannerData.ball.position.x + relX) < fabs(r_teamplannerData.ball.position.x - relX)) {
			receiverPosition = MRA::Geometry::Position(r_teamplannerData.ball.position.x + relX, preffered_Y_receiver);
		}
		else {
			receiverPosition = MRA::Geometry::Point(r_teamplannerData.ball.position.x - relX, preffered_Y_receiver);
		}
	}
	return receiverPosition;
}

// return the closest vector of the given vector
MRA::Geometry::Point RolePosition::closestTo(const MRA::Geometry::Point& reference_point, const std::vector<MRA::Geometry::Point>& positions)  {
   MRA::Geometry::Point closest = positions[0];
    double distClosest = reference_point.distanceTo(closest);
    for (auto it = positions.begin(); it != positions.end(); ++it ) {
        if (reference_point.distanceTo(*it) < distClosest) {
            closest = *it;
            distClosest = reference_point.distanceTo(*it);
        }
    }
    return closest;
}

//---------------------------------------------------------------------------------------------------------------------
/**
 */
void RolePosition::GetFixedPositions(std::vector<MRA::Geometry::Point>& playerPositions, const TeamPlannerData& r_teamplannerData) {

	bool parking_is_left = true;
	if (!r_teamplannerData.parking_positions.empty()) {
		parking_is_left = (r_teamplannerData.parking_positions[0].x < 0);
	}

	if (r_teamplannerData.gamestate == game_state_e::BEGIN_POSITION) {
		/**
		 * Assign algorithm used for begin of a half (first half/second half). Assign all players to a begin position
		 * Internally a list of begin positions is created (most imported position first). Then the players will be assigned to
		 * the positions, with use of the path-calculation.
		 */
		MRA::Geometry::Point fordwardRight = MRA::Geometry::Point( 1.0, -(r_teamplannerData.fieldConfig.getCenterCirleRadius() + r_teamplannerData.fieldConfig.getRobotRadius())-1);
		MRA::Geometry::Point fordwardLeft  = MRA::Geometry::Point(-1.0, -(r_teamplannerData.fieldConfig.getCenterCirleRadius() + r_teamplannerData.fieldConfig.getRobotRadius())-1);
		if (parking_is_left) {
			playerPositions.push_back(fordwardRight);
			playerPositions.push_back(fordwardLeft);
		}
		else {
			playerPositions.push_back(fordwardLeft);
			playerPositions.push_back(fordwardRight);
		}
		MRA::Geometry::Point backRight = MRA::Geometry::Point(  (r_teamplannerData.fieldConfig.getMaxFieldX()*0.5) + 0.5, -(r_teamplannerData.fieldConfig.getMaxFieldY()*0.5)-0.3);
		MRA::Geometry::Point backLeft  = MRA::Geometry::Point( -(r_teamplannerData.fieldConfig.getMaxFieldX()*0.5) - 0.5, -(r_teamplannerData.fieldConfig.getMaxFieldY()*0.5)-0.3);
		if (parking_is_left) {
			playerPositions.push_back(backRight);
			playerPositions.push_back(backLeft);
		}
		else {
			playerPositions.push_back(backLeft);
			playerPositions.push_back(backRight);
		}
	}
	else if (r_teamplannerData.gamestate == game_state_e::PARKING) {
		// select position closest to default goalie position as parking position for the goalie
		MRA::Geometry::Point goalieDefaultPosition = MRA::Geometry::Point(0, -r_teamplannerData.fieldConfig.getMaxFieldY());
		MRA::Geometry::Point goalieParkingPosition = closestTo(goalieDefaultPosition, r_teamplannerData.parking_positions);

		std::vector<MRA::Geometry::Point> fieldplayer_parking_positions = std::vector<MRA::Geometry::Point>();
		for (auto it = r_teamplannerData.parking_positions.begin(); it != r_teamplannerData.parking_positions.end(); ++it) {
			MRA::Geometry::Point parking_pos = *it;
			if (parking_pos.equals(goalieParkingPosition) == false) {
				fieldplayer_parking_positions.push_back(parking_pos);
			}
		}

		/**
		 * Assign algorithm used for parking game situation. Assign all players to a place in the parking lot.
		 * Internally a list of parking positions is created (most imported position first). Then the players will be assigned to
		 * the positions, with use of the path-calculation.
		 */
		if (r_teamplannerData.parking_positions.size() > 0) {
			playerPositions.push_back(fieldplayer_parking_positions[0]);
		}
		if (r_teamplannerData.parking_positions.size() > 1) {
			playerPositions.push_back(fieldplayer_parking_positions[1]);
		}
		if (r_teamplannerData.parking_positions.size() > 2) {
			playerPositions.push_back(fieldplayer_parking_positions[2]);
		}
		if (r_teamplannerData.parking_positions.size() > 3) {
			playerPositions.push_back(fieldplayer_parking_positions[3]);
		}
	}
	else if (r_teamplannerData.gamestate == game_state_e::KICKOFF) {

		/**
		 * Assign players to positions for an offensive kickoff
		 * Kick off setup is determined by the KickOffSide
		 * TODO: find a better solution to integrate the 2m rule...
		 */
		playerPositions.push_back( MRA::Geometry::Point(r_teamplannerData.parameters.kickoff_fp1_x, r_teamplannerData.parameters.kickoff_fp1_y));
		playerPositions.push_back( MRA::Geometry::Point(r_teamplannerData.parameters.kickoff_fp2_x, r_teamplannerData.parameters.kickoff_fp2_y));
		playerPositions.push_back( MRA::Geometry::Point(r_teamplannerData.parameters.kickoff_fp3_x, r_teamplannerData.parameters.kickoff_fp3_y));
		playerPositions.push_back( MRA::Geometry::Point(r_teamplannerData.parameters.kickoff_fp4_x, r_teamplannerData.parameters.kickoff_fp4_y));
	}
	else if (r_teamplannerData.gamestate == game_state_e::KICKOFF_AGAINST) {
		/**
		 * Static assignment of defensive positions during an kickoff against
		 *
		 * MSL rule RC-8.3:
		 *  - team taking kickoff: must be minimal 2 meter from ball except the kicker
		 *  - team with kickoff-against: Minimal 3 meters from the ball
		 */

		playerPositions.push_back( MRA::Geometry::Point(r_teamplannerData.parameters.kickoff_against_fp1_x, r_teamplannerData.parameters.kickoff_against_fp1_y));
		playerPositions.push_back( MRA::Geometry::Point(r_teamplannerData.parameters.kickoff_against_fp2_x, r_teamplannerData.parameters.kickoff_against_fp2_y));
		playerPositions.push_back( MRA::Geometry::Point(r_teamplannerData.parameters.kickoff_against_fp3_x, r_teamplannerData.parameters.kickoff_against_fp3_y));
		playerPositions.push_back( MRA::Geometry::Point(r_teamplannerData.parameters.kickoff_against_fp4_x, r_teamplannerData.parameters.kickoff_against_fp4_y));
		}
	else if (r_teamplannerData.searchForBall) {
		getSearchForBallPositions(playerPositions, r_teamplannerData);
	}
	//print_provided_position(gamestate, playerPositions);
}

void RolePosition::getSearchForBallPositions(std::vector<MRA::Geometry::Point>& playerPositions, const TeamPlannerData& r_teamplannerData)
{
	double topPenaltyAreaSearchPos = r_teamplannerData.fieldConfig.getMaxFieldY()-(r_teamplannerData.fieldConfig.getPenaltyAreaLength()+0.5);

	if (r_teamplannerData.gamestate == game_state_e::CORNER) {
		playerPositions.push_back(MRA::Geometry::Point(-r_teamplannerData.fieldConfig.getMaxFieldX()+1.0, +r_teamplannerData.fieldConfig.getMaxFieldY()-1.0));  // left near opponent corner
		playerPositions.push_back(MRA::Geometry::Point( r_teamplannerData.fieldConfig.getMaxFieldX()-1.0, +r_teamplannerData.fieldConfig.getMaxFieldY()-1.0));  // right near opponent corner
		playerPositions.push_back(MRA::Geometry::Point( r_teamplannerData.fieldConfig.getMaxFieldX()*0.5, -r_teamplannerData.fieldConfig.getMaxFieldY()*0.5));  // right of field, just middle of own half in Y
		playerPositions.push_back(MRA::Geometry::Point(-r_teamplannerData.fieldConfig.getMaxFieldX()*0.5, -r_teamplannerData.fieldConfig.getMaxFieldY()*0.5));  // left of field just middle of own half in Y
	}
	else if (r_teamplannerData.gamestate == game_state_e::CORNER_AGAINST) {
		playerPositions.push_back(MRA::Geometry::Point(-r_teamplannerData.fieldConfig.getMaxFieldX()+1.5, -r_teamplannerData.fieldConfig.getMaxFieldY()+1.5));  // left near own corner
		playerPositions.push_back(MRA::Geometry::Point( r_teamplannerData.fieldConfig.getMaxFieldX()-1.5, -r_teamplannerData.fieldConfig.getMaxFieldY()+1.5));  // right near own corner
		playerPositions.push_back(MRA::Geometry::Point( r_teamplannerData.fieldConfig.getMaxFieldX()*0.5, +r_teamplannerData.fieldConfig.getMaxFieldY()*0.5));  // right of field, just middle of opponent half in Y
		playerPositions.push_back(MRA::Geometry::Point(-r_teamplannerData.fieldConfig.getMaxFieldX()*0.5, +r_teamplannerData.fieldConfig.getMaxFieldY()*0.5));  // left of field just middle of opponent half in Y
	}
	else if ((r_teamplannerData.gamestate == game_state_e::GOALKICK) || (r_teamplannerData.gamestate == game_state_e::PENALTY_AGAINST) || (r_teamplannerData.gamestate == game_state_e::PENALTY_SHOOTOUT_AGAINST)) {
		playerPositions.push_back(MRA::Geometry::Point(-r_teamplannerData.fieldConfig.getMaxFieldX()*0.5, -topPenaltyAreaSearchPos));  // left near top own penalty area
		playerPositions.push_back(MRA::Geometry::Point( r_teamplannerData.fieldConfig.getMaxFieldX()*0.5, -topPenaltyAreaSearchPos));  // right near top own penalty area
		playerPositions.push_back(MRA::Geometry::Point( r_teamplannerData.fieldConfig.getMaxFieldX()*0.5, +r_teamplannerData.fieldConfig.getMaxFieldY()*0.5));  // right of field, just middle of opponent half in Y
		playerPositions.push_back(MRA::Geometry::Point(-r_teamplannerData.fieldConfig.getMaxFieldX()*0.5, +r_teamplannerData.fieldConfig.getMaxFieldY()*0.5));  // left of field just middle of opponent half in Y
	}
	else if ((r_teamplannerData.gamestate == game_state_e::GOALKICK_AGAINST) || (r_teamplannerData.gamestate == game_state_e::PENALTY) || (r_teamplannerData.gamestate == game_state_e::PENALTY_SHOOTOUT)) {
		playerPositions.push_back(MRA::Geometry::Point(-r_teamplannerData.fieldConfig.getMaxFieldX()*0.5, +topPenaltyAreaSearchPos));  // left near top opponent corner
		playerPositions.push_back(MRA::Geometry::Point( r_teamplannerData.fieldConfig.getMaxFieldX()*0.5, +topPenaltyAreaSearchPos));  // right near top opponent corner
		playerPositions.push_back(MRA::Geometry::Point( r_teamplannerData.fieldConfig.getMaxFieldX()*0.5, -r_teamplannerData.fieldConfig.getMaxFieldY()*0.5));  // right of field, just middle of own half in Y
		playerPositions.push_back(MRA::Geometry::Point(-r_teamplannerData.fieldConfig.getMaxFieldX()*0.5, -r_teamplannerData.fieldConfig.getMaxFieldY()*0.5));  // left of field just middle of own half in Y

	}
	else if ((r_teamplannerData.gamestate == game_state_e::THROWIN) || (r_teamplannerData.gamestate == game_state_e::THROWIN_AGAINST)) {
		double x_dist = r_teamplannerData.fieldConfig.getMaxFieldX()-1.5;
		playerPositions.push_back(MRA::Geometry::Point(-x_dist, -(r_teamplannerData.fieldConfig.getMaxFieldY()*0.5)));  // left of field, just middle of own half in Y
		playerPositions.push_back(MRA::Geometry::Point( x_dist, +(r_teamplannerData.fieldConfig.getMaxFieldY()*0.5)));  // right of field, just middle of opponent half in Y
		playerPositions.push_back(MRA::Geometry::Point( x_dist, -(r_teamplannerData.fieldConfig.getMaxFieldY()*0.5)));  // right of field, just middle of own half in Y
		playerPositions.push_back(MRA::Geometry::Point(-x_dist, +(r_teamplannerData.fieldConfig.getMaxFieldY()*0.5)));  // left of field, just middle of opponent half in Y
	}
	else {
		// any other situation
		playerPositions.push_back(MRA::Geometry::Point(-r_teamplannerData.fieldConfig.getMaxFieldX()*0.5, -(r_teamplannerData.fieldConfig.getMaxFieldY()*0.5)));  // left of field, just middle of own half in Y
		playerPositions.push_back(MRA::Geometry::Point( r_teamplannerData.fieldConfig.getMaxFieldX()*0.5, +(r_teamplannerData.fieldConfig.getMaxFieldY()*0.5)));  // right of field, just middle of opponent half in Y
		playerPositions.push_back(MRA::Geometry::Point( r_teamplannerData.fieldConfig.getMaxFieldX()*0.5, -(r_teamplannerData.fieldConfig.getMaxFieldY()*0.5)));  // right of field, just middle of own half in Y
		playerPositions.push_back(MRA::Geometry::Point(-r_teamplannerData.fieldConfig.getMaxFieldX()*0.5, +(r_teamplannerData.fieldConfig.getMaxFieldY()*0.5)));  // left of field, just middle of opponent half in Y
	}
}

//---------------------------------------------------------------------------------------------------------------------
void RolePosition::print_provided_position(game_state_e gamestate, const std::vector<std::vector<MRA::Geometry::Point>>& positions) {
	MRA_LOG_INFO("Provided position info: ");
	MRA_LOG_INFO("game-state: %d (%s)",gamestate, GameStateAsString(gamestate).c_str());
	for (unsigned int idx = 0; idx < positions.size(); idx++) {
		MRA_LOG_INFO("position- vector [%d]", idx);
		for (unsigned int v_idx = 0; v_idx < positions[idx].size(); v_idx++) {
			MRA_LOG_INFO("\tposition[%d] = %s", v_idx, positions[idx][v_idx].toString().c_str());
		}
	}
}

MRA::Geometry::Point RolePosition::InterceptorNormalPlayPosition(planner_target_e& planner_target, const TeamPlannerData& r_teamplannerData) {
//
// Set role-position to ball-position (default dist < 3.0 m) and ball in priority-block area (default y < 0)
// IF Opponent is close to ball and priority block is enabled THEN
// 	IF any unassigned field-player is on the the defense line (= the line between the ball and the own goal)
//		THEN role-position will be 1.0 m from the ball on the defense line.
// 	ELSE
//		position unassigned field-player as fast as possible on the defense line.
//		- 	determine cross-section of the perpendicular (dutch: loodlijn) of the defense line with the defense line
//          (end point 1.5 meter from ball (= A) and middle of the goal-line (B).
//	         1.5 meter is chosen to prevent fast path from other side of the ball)
//      -   if cross-section is between A and B then use distance from player to defense line as cost.
//	    -   if cross-section is NOT between A and B then use 4 times distance from player to defense line as cost (and select A as cross-section).
//	    -  cross-section point of the field player with lowest cost becomes the the intercepter player
//

	MRA::Geometry::Position ballPos = r_teamplannerData.ball.position;
	MRA::Geometry::Position rolePosition = ballPos; // set default position to the ball position


	// determine the smallest distance from any opponent to the ball
	double smallestDistOpponentToBall = r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance();
	for (auto it = r_teamplannerData.opponents.begin(); it != r_teamplannerData.opponents.end(); ++it) {
		smallestDistOpponentToBall = min(it->position.distanceTo(ballPos), smallestDistOpponentToBall);
	}

	bool ballInArea = r_teamplannerData.parameters.priority_block_check_ball_in_area && (ballPos.y <= r_teamplannerData.parameters.priority_block_max_ball_y);
	bool opponentCloseToBall = r_teamplannerData.parameters.priority_block_check_opponent_close_to_ball && smallestDistOpponentToBall < r_teamplannerData.parameters.priority_block_max_opponent_to_ball_dist;

	// CHECK if priority block must be performed
	if (r_teamplannerData.parameters.priority_block_apply && ballInArea && opponentCloseToBall) {
		// priority block only when:
		// - ball.Y is below threshold (original default own half)
		// - opponent is close to the ball
		// - priority block option is enabled

		MRA::Geometry::Position goalPos = MRA::Geometry::Position(0.0, -r_teamplannerData.fieldConfig.getMaxFieldY());
		double distBall2Goal = goalPos.distanceTo(ballPos);
		double smallestDistToBallOnTheDefenseLine = r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance();
		unsigned nr_field_players_between_ball_and_own_goal = 0;
		double maxDistanceToBall = r_teamplannerData.parameters.priority_block_max_distance;  // safe distance to ball (used to prevent players move around ball).
		double minDistanceToBall = r_teamplannerData.parameters.priority_block_min_distance;  // safe distance to ball (used to prevent players move around ball).


		// count number of field-players on the line between ball and own goal.
		for (auto it = r_teamplannerData.team.begin(); it != r_teamplannerData.team.end(); ++it) {
			if( it->player_type == player_type_e::FIELD_PLAYER && !(it->assigned)) {
				// only unassigned field-players

				MRA::Geometry::Position playerPos = it->position;
				double distToDefenseLine = getDistanceFromPointToLine(ballPos.x, ballPos.y, goalPos.x, goalPos.y, playerPos.x, playerPos.y);
				double distToGoal = playerPos.distanceTo(goalPos);
				double distToBall = playerPos.distanceTo(ballPos);

				if ((distToGoal < distBall2Goal) &&
				    (distToDefenseLine < r_teamplannerData.parameters.priority_block_max_distance_to_defense_line) &&
					(distToBall < maxDistanceToBall))
				{
					// field player is on line between goal and ball AND close enough to the ball (<safeDistanceToBall):
					// -> closer than the ball to the goal and small distance to defense line
					nr_field_players_between_ball_and_own_goal++; // increase counter
					double distToBall = playerPos.distanceTo(ballPos);
					smallestDistToBallOnTheDefenseLine = min(distToBall, smallestDistToBallOnTheDefenseLine);
				}
			}
		}
		double goalToBallAngle = Geometry::Point(ballPos.x, ballPos.y).angle(goalPos);
		if (nr_field_players_between_ball_and_own_goal >= 1) {
			// Select position on the defense line, close to the ball (intended: drive towards the ball if on defense line)
			double targetDistToBall = min(smallestDistToBallOnTheDefenseLine, r_teamplannerData.parameters.priority_block_max_distance); // take smallest: to prevent moving backwards
			targetDistToBall = max(targetDistToBall, r_teamplannerData.parameters.priority_block_min_distance); // take biggest: to prevent moving too much forward
			double priorityBlockX = ballPos.x - (cos(goalToBallAngle) * targetDistToBall);
			double priorityBlockY = ballPos.y - (sin(goalToBallAngle) * targetDistToBall);
			rolePosition = MRA::Geometry::Position(priorityBlockX, priorityBlockY); // set role position on the priority block location
		}
		else {
			// No field-player on the defense line
			// calculate the safe priority block position (last position on line between ball and own goal)
			// calculate distance from maxPriorityBlockPoint to goal
			double maxPriorityBlockX = ballPos.x - (cos(goalToBallAngle) * maxDistanceToBall);
			double maxPriorityBlockY = ballPos.y - (sin(goalToBallAngle) * maxDistanceToBall);
			MRA::Geometry::Position maxPriorityBlockPoint = MRA::Geometry::Position(maxPriorityBlockX, maxPriorityBlockY);
			double minPriorityBlockX = ballPos.x - (cos(goalToBallAngle) * minDistanceToBall);
			double minPriorityBlockY = ballPos.y - (sin(goalToBallAngle) * minDistanceToBall);
			MRA::Geometry::Position minPriorityBlockPoint = MRA::Geometry::Position(minPriorityBlockX, minPriorityBlockY);


			// find team-mate which is closest to defense line then the priority block position
			// (from ball to safe priority block position - (between ball and own goal))
			MRA::Geometry::Position bestPos = maxPriorityBlockPoint;
			for (auto it = r_teamplannerData.team.begin(); it != r_teamplannerData.team.end(); ++it) {
				if( it->player_type == player_type_e::FIELD_PLAYER && !(it->assigned)) {
					// only unassigned field-players
					MRA::Geometry::Position teamMatePos = it->position;
					// calculate second location of on the perpendicular to the defense line
					double tx1 = teamMatePos.x + (cos(goalToBallAngle+M_PI*0.5) * 1.0);
					double ty1 = teamMatePos.y + (sin(goalToBallAngle+M_PI*0.5) * 1.0);

					// calculate intersection of the perpendicular of the defense line with the defense line
					double intersectX = 0;
					double intersectY = 0;
					bool linesIntersect = getIntersectionOfTwoLines(intersectX, intersectY, tx1, ty1, teamMatePos.x, teamMatePos.y,
							maxPriorityBlockX, maxPriorityBlockY, goalPos.x, goalPos.y);
					MRA::Geometry::Position intersectionPoint = MRA::Geometry::Position(intersectX, intersectY);
					if (linesIntersect && intersectionPoint.distanceTo(ballPos) < bestPos.distanceTo(ballPos) && intersectionPoint.distanceTo(goalPos) < distBall2Goal ) {
						// intersection point is closer to ball then best position found so far. And intersectionPoint is between ball and own goal
						if (intersectionPoint.distanceTo(ballPos) >= minDistanceToBall && intersectionPoint.distanceTo(ballPos) <= maxDistanceToBall) {
							// intersection point is between min and max allowed dist
							bestPos = intersectionPoint;
						}
						else
						{
							if (intersectionPoint.distanceTo(ballPos) < minDistanceToBall){
								bestPos = minPriorityBlockPoint;
							}
							else {
								bestPos = maxPriorityBlockPoint;
							}

						}
					}

				}
			}
			// set role position for the best found position
			rolePosition = bestPos;
			planner_target = planner_target_e::PRIORITY_BLOCK;  // indicate to followpath that path purpose is priority block

		}
	}
	return rolePosition;
}

int RolePosition::FindMostDangerousOpponentAndNotAssigned(const TeamPlannerData& r_teamplannerData)
/* find the opponent who is most dangerous and not yet assigned (= defended)
 * return index of opponent in Opponents vector, return -1 if no opponent found to defend.
 */
{
	int closest_to_idx = -1;

	// find closest to ball
	std::vector<double> oppenent_to_ball_dist = std::vector<double>();
	std::vector<double> oppenent_to_goal_dist = std::vector<double>();
	MRA::Geometry::Position ballPos = r_teamplannerData.ball.position;
	MRA::Geometry::Position goalPos = MRA::Geometry::Position(0, -r_teamplannerData.fieldConfig.getMaxFieldY());
	// get distances for each opponent to ball and our goal.
	for (unsigned idx = 0; idx < r_teamplannerData.opponents.size(); idx++) {
		MRA::Geometry::Position opponent_position = r_teamplannerData.opponents[idx].position;
		oppenent_to_ball_dist.push_back(ballPos.distanceTo(opponent_position));
		oppenent_to_goal_dist.push_back(goalPos.distanceTo(opponent_position));
	}

	// assume opponent closest to our goal is most dangerous
	MRA::Geometry::Position opponentGoal = MRA::Geometry::Position(0, r_teamplannerData.fieldConfig.getMaxFieldY());
	double closest_dist_to = std::numeric_limits<double>::infinity();
	for (unsigned idx = 0; idx < r_teamplannerData.opponents.size(); idx++) {
		if (r_teamplannerData.opponents[idx].position.distanceTo(opponentGoal) > r_teamplannerData.parameters.dist_to_goal_to_mark_opponent_as_goalie) {
			// Not opponent goalie  (too far from opponent goal)
			if (oppenent_to_goal_dist[idx] < closest_dist_to
					&& r_teamplannerData.opponents[idx].assigned == false) {
				// opponent is not defended yet and closest to the ball
				closest_dist_to = oppenent_to_goal_dist[idx];
				closest_to_idx = idx;
			}
		}

	}
	return closest_to_idx;
}

int RolePosition::FindOpponentClostestToPositionAndNotAssigned(const MRA::Geometry::Point& targetPos, const TeamPlannerData& r_teamplannerData)
/* find the opponent closest to the ball and not yet assigned (= defended)
 * return index of opponent in Opponents vector, return -1 if no opponent found to defend.
 */
{
	int closest_to_idx = -1;

	// find closest to ball
	std::vector<double> oppenent_to_ball_dist = std::vector<double>();
	std::vector<double> oppenent_to_goal_dist = std::vector<double>();
	MRA::Geometry::Position goalPos = MRA::Geometry::Position(0, -r_teamplannerData.fieldConfig.getMaxFieldY());
	// get distances for each opponent to ball and our goal.
	for (unsigned idx = 0; idx < r_teamplannerData.opponents.size(); idx++) {
		MRA::Geometry::Position opponent_position = r_teamplannerData.opponents[idx].position;
		oppenent_to_ball_dist.push_back(targetPos.distanceTo(opponent_position));
		oppenent_to_goal_dist.push_back(goalPos.distanceTo(opponent_position));
	}

	MRA::Geometry::Position opponentGoal = MRA::Geometry::Position(0, r_teamplannerData.fieldConfig.getMaxFieldY());
	double closest_dist_to = std::numeric_limits<double>::infinity();
	for (unsigned idx = 0; idx < r_teamplannerData.opponents.size(); idx++) {
		if (r_teamplannerData.opponents[idx].position.distanceTo(opponentGoal) > r_teamplannerData.parameters.dist_to_goal_to_mark_opponent_as_goalie) {
			// Not opponent goalie  (too far from opponent goal)
			if (oppenent_to_ball_dist[idx] < closest_dist_to
					&& r_teamplannerData.opponents[idx].assigned == false) {
				// opponent is not defended yet and closest to the ball
				closest_dist_to = oppenent_to_ball_dist[idx];
				closest_to_idx = idx;
			}
		}
	}
	return closest_to_idx;
}

//--------------------------------------------------------------------------
MRA::Geometry::Position RolePosition::calculateManToManDefensePosition(
		defend_info_t& rDefend_info, dynamic_role_e dynamic_role,
		TeamPlannerData& r_teamplannerData, int& r_gridFileNumber, bool setPlayActive) {
	// man coverage for all opponents
	MRA::Geometry::Position rolePosition = MRA::Geometry::Position();

	rDefend_info.valid = false;
	int closest_to_ball_idx = FindMostDangerousOpponentAndNotAssigned(r_teamplannerData);
	if (closest_to_ball_idx >= 0) {
		// Found opponent closest to the ball. Apply man defense for this opponent
	    r_teamplannerData.opponents[closest_to_ball_idx].assigned = true;
		rolePosition = TeamPlanner_Grid::findManToManDefensivePosition(dynamic_role,
		        r_teamplannerData.opponents[closest_to_ball_idx].position, r_teamplannerData, r_gridFileNumber++, setPlayActive);
		rDefend_info.valid = true;
		rDefend_info.defending_id = r_teamplannerData.opponents[closest_to_ball_idx].label;
		rDefend_info.dist_from_defending_id = r_teamplannerData.opponents[closest_to_ball_idx].position.distanceTo(rolePosition);
		rDefend_info.between_ball_and_defending_pos = r_teamplannerData.parameters.manDefenseBetweenBallAndPlayer;

	} else {
		// no opponent to defend left, find defensive position
		rolePosition = TeamPlanner_Grid::findDefensivePosition(r_teamplannerData, r_gridFileNumber++);
	}
	return rolePosition;
}


