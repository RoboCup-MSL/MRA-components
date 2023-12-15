/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */
#include "RolePosition.hpp"

#include <cmath>
#include <iostream>
#include "MathUtils.h"
#include <limits>
#include "Dynamics.hpp"
#include "planner_types.hpp"
#include "TeamPlannerGrid.hpp"  // grid related teamplanner functions

using namespace std;
using namespace trs;


Vector2D RolePosition::determineDynamicRolePosition(defend_info_t& rDefend_info, planner_target_e& planner_target, int& r_gridFileNumber, dynamic_role_e dynamic_role, game_state_e gamestate,
		const MovingObject& globalBall, const MovingObject& locallBall, const previous_used_ball_by_planner_t& previous_global_ball,
		std::vector<TeamPlannerRobot>& Team, std::vector<TeamPlannerOpponent>& Opponents,
		const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig,
		const ball_pickup_position_t& ball_pickup_position, bool passIsRequired,
		bool teamControlBall, bool playerPassedBall, const pass_data_t& pass_data, bool& r_role_position_is_end_position_of_pass) {
	Vector2D rolePosition = Vector2D(0,0);
	rDefend_info.defending_id = -1;	// no item to defend
	r_role_position_is_end_position_of_pass = false;

	if (plannerOptions.man_to_man_defense_during_setplay_against &&
		(gamestate == game_state_e::GOALKICK_AGAINST || gamestate == game_state_e::CORNER_AGAINST ||
        gamestate == game_state_e::FREEKICK_AGAINST || gamestate == game_state_e::THROWIN_AGAINST 	) )
	{
		// Setplay against: man coverage for all opponents
		rolePosition = calculateManToManDefensePosition(rDefend_info, dynamic_role, globalBall, fieldConfig, gamestate, plannerOptions, Opponents, Team, r_gridFileNumber, true /* setplay active */, teamControlBall);
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
		if (gamestate == PENALTY_SHOOTOUT) {
			// position in top of center circle
			rolePosition = Vector2D(0, fieldConfig.getCenterCirleRadius() - fieldConfig.getRobotSize() );
		}
		else
		{
			Vector2D goalPos = Vector2D(0, fieldConfig.getMaxFieldY());
			Vector2D ballPos = globalBall.getPosition().getVector2D();
			if (fieldConfig.isInField(ballPos.m_x, ballPos.m_y, plannerOptions.outsideFieldMargin))
			{
				double alfa = ballPos.angle(goalPos);
				double behind_ball_x = ballPos.m_x + (cos(alfa) * 1.0);
				double behind_ball_y = ballPos.m_y + (sin(alfa) * 1.0);
				rolePosition = Vector2D(behind_ball_x,behind_ball_y);
			}
			else
			{
				rolePosition = Vector2D(0, fieldConfig.getMaxFieldY() - fieldConfig.getPenaltySpotToBackline() - 1.0);
			}
		}
	}
	break;
	case dr_SETPLAY_KICKER: {
		// CODE only valid for set-play !!!
		// calculation assume that no opponent or team mate is at calculated position. This is only true for restart.
		// Known issues with algorithm
		// - not take time allowed in penalty area into account.

		Vector2D shooterPosition;
		Vector2D receiverPosition;
		// Get ball position, will be the base of the positioning
		Vector2D globalBallPosition = globalBall.getPosition().getVector2D();
		calculateSetPlayPosition(shooterPosition, receiverPosition, Team, globalBallPosition, previous_global_ball, gamestate, plannerOptions, fieldConfig);
		rolePosition = shooterPosition;  // right of field, just middle of opponent half in Y
	}
	break;
	case dr_SETPLAY_RECEIVER: {
		// CODE only valid for set-play
		// put setplay-receiver at a location for the specific setplay, e.g. freekick at own half put it at opponent half
		// calculation assume that no opponent or team mate is at calculated position. This is only true for restart.
		// Known issues with algorithm
		// - not take time allowed in penalty area into account.
		Vector2D shooterPosition;
		Vector2D receiverPosition;
		// Get ball position, will be the base of the positioning
		Vector2D globalBallPosition = globalBall.getPosition().getVector2D();
		calculateSetPlayPosition(shooterPosition, receiverPosition, Team, globalBallPosition, previous_global_ball, gamestate, plannerOptions, fieldConfig);
		rolePosition = receiverPosition;  // right of field, just middle of opponent half in Y
	}
	break;
	case dr_PENALTY_DEFENDER:
	{
			rolePosition = TeamPlanner_Grid::findDefensivePositionDuringPenaltyShootOut(Team,  gamestate, 	plannerOptions, globalBall, Opponents,r_gridFileNumber++, fieldConfig);
	}
	break;
	case dr_DEFENDER:
	{
		if (plannerOptions.man_to_man_defense_during_normal_play) {
			// man coverage
			rolePosition = calculateManToManDefensePosition(rDefend_info, dynamic_role, globalBall, fieldConfig, gamestate, plannerOptions, Opponents, Team, r_gridFileNumber, false /* normal-play */, teamControlBall);
		}
		else {
			rolePosition = TeamPlanner_Grid::findDefensivePosition(Team,  gamestate, 	plannerOptions, globalBall, Opponents,r_gridFileNumber++, fieldConfig);
		}
	}
	break;
	case dr_ATTACKSUPPORTER: {
		bool position_close_to_ball = false;
		if (gamestate == game_state_e::GOALKICK) {
			position_close_to_ball = true;
		}
		MovingObject ballPostionToUse = globalBall;
		if (plannerOptions.use_pass_to_position_for_attack_support && pass_data.valid) {
			ballPostionToUse.set(pass_data.target_pos.m_x, pass_data.target_pos.m_y, 0.0, 0.0, 0.0, 0.0, -1, true);
		}

		// check if already player moves to ball target position
		bool assigned_another_attack_supporter_to_ball_pos = false;
		for (unsigned ap_idx = 0; ap_idx < Team.size(); ap_idx++) {
			if (Team[ap_idx].assigned
				&& Team[ap_idx].result.target.distanceTo(Vector2D(pass_data.target_pos.m_x, pass_data.target_pos.m_y)) < 0.1)  {
				// player already assigned to ball position
				assigned_another_attack_supporter_to_ball_pos = true;
			}
		}
		if ((pass_data.valid) &&
			(pass_data.target_id != 0 /* not shot on goal */) &&   // TODO: used id for attack supporter assignment
			(! assigned_another_attack_supporter_to_ball_pos)) {
			// ball passed and no other player moves to the ball, go to ball target position
			rolePosition = Vector2D(pass_data.target_pos.m_x, pass_data.target_pos.m_y);
			r_role_position_is_end_position_of_pass = true;
		} else {
			TeamPlanner_Grid::findAttackSupportPosition(rolePosition, Team, gamestate, plannerOptions, ballPostionToUse, Opponents, r_gridFileNumber++, fieldConfig, position_close_to_ball, teamControlBall);
		}
	}
	break;
	case dr_SWEEPER: {
		rolePosition = TeamPlanner_Grid::findSweeperPosition(Team, gamestate, plannerOptions, globalBall, Opponents, r_gridFileNumber++, fieldConfig);
	}
	break;
	case dr_INTERCEPTOR: {
		if (gamestate == game_state_e::GOALKICK_AGAINST || gamestate == game_state_e::CORNER_AGAINST ||
			gamestate == game_state_e::DROPPED_BALL || gamestate == game_state_e::FREEKICK_AGAINST ||
			gamestate == game_state_e::PENALTY_AGAINST || gamestate == game_state_e::PENALTY_SHOOTOUT_AGAINST ||
			gamestate == game_state_e::THROWIN_AGAINST 	)
		{
			rolePosition = TeamPlanner_Grid::findInterceptorPositionDuringRestart(Team, gamestate, plannerOptions,
					globalBall, Opponents, r_gridFileNumber++, fieldConfig);
		}
		else {
			Vector2D intercept_position = globalBall.getXYlocation();
			if (plannerOptions.interceptor_assign_use_ball_velocity &&
				globalBall.getLinearVelocity() > plannerOptions.interceptor_assign_min_velocity_for_calculate_interception_position) {
				// ball is moving: calculate distance to interception position per robot.
				double maxSpeed = plannerOptions.maxPossibleLinearSpeed;
				double bestIntercept_distance = std::numeric_limits<double>::infinity();
				for (unsigned int idx = 0; idx < Team.size(); idx++) {
					if (!Team[idx].assigned) {
						// Find initial intercept point
						Vector2D player_pos = Team[idx].position.getXYlocation();
						Dynamics::dynamics_t intercept_data = Dynamics::interceptBall(globalBall, player_pos, maxSpeed,
								fieldConfig, plannerOptions.move_to_ball_left_field_position);
						if (player_pos.distanceTo(intercept_data.intercept_position) < bestIntercept_distance) {
							bestIntercept_distance = player_pos.distanceTo(intercept_data.intercept_position);
							intercept_position = intercept_data.intercept_position;
						}
					}
				}
				rolePosition = intercept_position;
			}
			else {
				rolePosition = InterceptorNormalPlayPosition(planner_target, globalBall, Team, Opponents,plannerOptions, fieldConfig);
			}

			int closest_to_ball_idx = FindOpponentClostestToPositionAndNotAssigned(intercept_position, fieldConfig, plannerOptions, Opponents);
			if (closest_to_ball_idx >= 0) {
				// Found opponent closest to the ball. update administration that the interceptor defense this opponent.
				Opponents[closest_to_ball_idx].assigned = true;
			}
		}
	}
	break;
	case dr_BALLPLAYER: {
		Vector2D shootPos = TeamPlanner_Grid::findBallPlayerPosition(Team, gamestate, plannerOptions, globalBall, Opponents, r_gridFileNumber++, fieldConfig, ball_pickup_position, passIsRequired);
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

Vector2D RolePosition::setplay_receiver_position_90deg_to_ball_goal(const Vector2D& globalBallPosition, const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig)
{
	// select position 90 degrees to the line between the ball and the opponent goal
	Vector2D receiverPosition;
	double distBallReciever = plannerOptions.restart_receiver_ball_dist; // distance between receiver and ball
	Vector2D goalPos = Vector2D(0, fieldConfig.getMaxFieldY());
	double alfa = globalBallPosition.angle(goalPos);

	double outsideFieldMargin = plannerOptions.outsideFieldMargin; // distance to stay from side of field
	double recieveX_plus_90deg = globalBallPosition.m_x + (cos(alfa + 0.5 * M_PI) * distBallReciever);
	double recieveY_plus_90deg = globalBallPosition.m_y + (sin(alfa + 0.5 * M_PI) * distBallReciever);
	double recieveX_min_90deg = globalBallPosition.m_x  + (cos(alfa - 0.5 * M_PI) * distBallReciever);
	double recieveY_min_90deg = globalBallPosition.m_y  + (sin(alfa - 0.5 * M_PI) * distBallReciever);
	bool min90_inField = fieldConfig.isInField(recieveX_min_90deg, recieveY_min_90deg, outsideFieldMargin);
	bool plus90_inField = fieldConfig.isInField(recieveX_plus_90deg, recieveY_plus_90deg, outsideFieldMargin);
	if (min90_inField && plus90_inField) {
		// both positions are in the field, select the one closest to opponent goal.
		if (recieveY_min_90deg > recieveY_plus_90deg) {
			receiverPosition = Vector2D(recieveX_min_90deg, recieveY_min_90deg);
		}
		else {
			receiverPosition = Vector2D(recieveX_plus_90deg, recieveY_plus_90deg);
		}
	} else if (min90_inField) {
		receiverPosition = Vector2D(recieveX_min_90deg, recieveY_min_90deg);
	} else {
		receiverPosition = Vector2D(recieveX_plus_90deg, recieveY_plus_90deg);
	}

	// in corner area it might go wrong and a Y position outside the field is selected.
	if (!fieldConfig.isInField(receiverPosition.m_x, receiverPosition.m_y, outsideFieldMargin)) {
		if (!fieldConfig.isInField(0, receiverPosition.m_y,	outsideFieldMargin)) {
			// y-location is outside the field.
			if (recieveY_min_90deg > recieveY_plus_90deg) {
				receiverPosition = Vector2D(recieveX_plus_90deg,
						recieveY_plus_90deg);
			} else {
				receiverPosition = Vector2D(recieveX_min_90deg,
						recieveY_min_90deg);
			}
		}
		// x-position can be field
		if (receiverPosition.m_x < -(fieldConfig.getMaxFieldX() - outsideFieldMargin)) {
			receiverPosition.m_x = -(fieldConfig.getMaxFieldX()	- outsideFieldMargin);
		}
		if (receiverPosition.m_x > (fieldConfig.getMaxFieldX() - outsideFieldMargin)) {
			receiverPosition.m_x = (fieldConfig.getMaxFieldX() - outsideFieldMargin);
		}
	}

	return receiverPosition;
}

// ------------------------------------------------------------------
bool RolePosition::calculateSetPlayReceiverMinTurnPosition(const FieldConfig& fieldConfig,
		const Vector2D& globalBallPosition, const PlannerOptions& plannerOptions, Vector2D& receiverPosition) {
	bool found = false;

	// no straight lob-shot possible
	double outsideFieldMargin = plannerOptions.outsideFieldMargin; // distance to stay from side of field
	double distBallReciever = plannerOptions.restart_receiver_ball_dist; // distance between receiver and ball
	double sol1_x, sol1_y, sol2_x, sol2_y = 0;
	double maxFieldX = fieldConfig.getMaxFieldX();
	double maxFieldY = fieldConfig.getMaxFieldY();
	int nr_crossings = 0;
	double backline_y = -fieldConfig.getMaxFieldY() + outsideFieldMargin;
	// check crossings with last y-line where receiver putting back is allowed.
	nr_crossings = findLineCircleIntersections(globalBallPosition.m_x,
			globalBallPosition.m_y, distBallReciever, -maxFieldX, backline_y,
			maxFieldX, backline_y, sol1_x, sol1_y, sol2_x, sol2_y);
	if (nr_crossings > 0) {
		// at least 1 intersection found the last line
		if (nr_crossings == 1) {
			if (fieldConfig.isInField(sol1_x, sol1_y, outsideFieldMargin)) {
				receiverPosition = Vector2D(sol1_x, sol1_y);
				found = true;
			}
		} else {
			// check if both intersections are in the field
			bool sol1_inField = fieldConfig.isInField(sol1_x, sol1_y,
					outsideFieldMargin)
					&& !fieldConfig.isInOwnGoalArea(sol1_x, sol1_y);
			bool sol2_inField = fieldConfig.isInField(sol2_x, sol2_y,
					outsideFieldMargin)
					&& !fieldConfig.isInOwnGoalArea(sol2_x, sol2_y);
			if (sol1_inField && sol2_inField) {
				// both intersections are in field, select position closes to the side-line
				if (fabs(sol1_x) < fabs(sol2_x)) {
					receiverPosition = Vector2D(sol2_x, sol2_y);
					found = true;
				} else {
					receiverPosition = Vector2D(sol1_x, sol1_y);
					found = true;
				}
			} else if (sol1_inField) {
				receiverPosition = Vector2D(sol1_x, sol1_y);
				found = true;
			} else if (sol2_inField) {
				receiverPosition = Vector2D(sol2_x, sol2_y);
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
		if (globalBallPosition.m_x < 0) {
			closest_direction = -closest_direction; // ball is left on the field
		}
		double closest_sidelineX = closest_direction
				* (maxFieldX - outsideFieldMargin);
		// find intersection with closest sideline
		int nr_crossings_side = findLineCircleIntersections(
				globalBallPosition.m_x, globalBallPosition.m_y,
				distBallReciever, closest_sidelineX, -maxFieldY,
				closest_sidelineX, maxFieldY, side_sol1_x, side_sol1_y,
				side_sol2_x, side_sol2_y);
		double paw = fieldConfig.getPenaltyAreaWidth();
		double pal = fieldConfig.getPenaltyAreaLength();
		double dist_to_side_penalty_area = 0.5 * paw
				+ plannerOptions.setplay_margin_to_penalty_area_side;
		// find intersection with penalty area. side and top
		int nr_crossings_penalty = findLineCircleIntersections(
				globalBallPosition.m_x, globalBallPosition.m_y,
				distBallReciever, closest_direction * dist_to_side_penalty_area,
				-maxFieldY, closest_direction * dist_to_side_penalty_area,
				-(maxFieldY - pal), penalty_sol1_x, penalty_sol1_y,
				penalty_sol2_x, penalty_sol2_y);
		int nr_crossings_penalty2 = findLineCircleIntersections(
				globalBallPosition.m_x, globalBallPosition.m_y,
				distBallReciever, -paw, -(maxFieldY - pal), +paw,
				-(maxFieldY - pal), penalty2_sol1_x, penalty2_sol1_y,
				penalty2_sol2_x, penalty2_sol2_y);
		double best_X = maxFieldX;
		double best_Y = maxFieldY;
		if (nr_crossings_side > 0) {
			if (fieldConfig.isInField(side_sol1_x, side_sol1_y,
					outsideFieldMargin) && side_sol1_y < best_Y
					&& !fieldConfig.isInOwnGoalArea(sol1_x, sol1_y)) {
				best_X = side_sol1_x;
				best_Y = side_sol1_y;
				found = true;
			}
		}
		if (nr_crossings_side > 1) {
			if (fieldConfig.isInField(side_sol2_x, side_sol2_y,
					outsideFieldMargin) && side_sol2_y < best_Y
					&& !fieldConfig.isInOwnGoalArea(sol2_x, sol2_y)) {
				best_X = side_sol2_x;
				best_Y = side_sol2_y;
				found = true;
			}
		}
		if (nr_crossings_penalty > 0) {
			if (fieldConfig.isInField(penalty_sol1_x, penalty_sol1_y, 0)
					&& penalty_sol1_y < best_Y
					&& !fieldConfig.isInOwnGoalArea(sol1_x, sol1_y)) {
				best_X = penalty_sol1_x;
				best_Y = penalty_sol1_y;
				found = true;
			}
		}
		if (nr_crossings_penalty > 1) {
			if (fieldConfig.isInField(penalty_sol2_x, penalty_sol2_y, 0)
					&& penalty_sol2_y < best_Y
					&& !fieldConfig.isInOwnGoalArea(sol2_x, sol2_y)) {
				best_X = penalty_sol2_x;
				best_Y = penalty_sol2_y;
				found = true;
			}
		}
		if (nr_crossings_penalty2 > 0) {
			if (fieldConfig.isInField(penalty2_sol1_x, penalty2_sol1_y, 0)
					&& penalty2_sol1_y < best_Y
					&& !fieldConfig.isInOwnGoalArea(sol2_x, sol2_y)) {
				best_X = penalty2_sol1_x;
				best_Y = penalty2_sol1_y;
				found = true;
			}
		}
		if (nr_crossings_penalty2 > 1) {
			if (fieldConfig.isInField(penalty2_sol2_x, penalty2_sol2_y, 0)
					&& penalty2_sol2_y < best_Y
					&& !fieldConfig.isInOwnGoalArea(sol2_x, sol2_y)) {
				best_X = penalty2_sol2_x;
				best_Y = penalty2_sol2_y;
				found = true;
			}
		}
		if (found) {
			receiverPosition = Vector2D(best_X, best_Y);
		}
	}
	return found;
} // end of calculateSetPlayReceiverMinTurnPosition

// ------------------------------------------------------------------
bool RolePosition::calculateSetPlayReceiverOnLobShotLinePosition(const FieldConfig& fieldConfig,
		const Vector2D& globalBallPosition, const PlannerOptions& plannerOptions, Vector2D& receiverPosition) {
	bool found = false;

	// no straight lob-shot possible
	double outsideFieldMargin = plannerOptions.outsideFieldMargin; // distance to stay from side of field
	double distBallReciever = plannerOptions.restart_receiver_ball_dist; // distance between receiver and ball
	double sol1_x, sol1_y, sol2_x, sol2_y = 0;
	double maxFieldX = fieldConfig.getMaxFieldX();
	int nr_crossings = 0;
	double lobshot_line = plannerOptions.min_y_for_lob_shot;
	// check crossings with lobshot line where receiver putting back is allowed.
	nr_crossings = findLineCircleIntersections(globalBallPosition.m_x,
			globalBallPosition.m_y, distBallReciever, -maxFieldX, lobshot_line,
			maxFieldX, lobshot_line, sol1_x, sol1_y, sol2_x, sol2_y);
	if (nr_crossings > 0) {
		// at least 1 intersection found the lobshot line
		if (nr_crossings == 1) {
			if (fieldConfig.isInField(sol1_x, sol1_y, outsideFieldMargin)) {
				receiverPosition = Vector2D(sol1_x, sol1_y);
				found = true;
			}
		} else {
			// check if both intersections are in the field
			bool sol1_inField = fieldConfig.isInField(sol1_x, sol1_y,
					outsideFieldMargin)
					&& !fieldConfig.isInOwnGoalArea(sol1_x, sol1_y);
			bool sol2_inField = fieldConfig.isInField(sol2_x, sol2_y,
					outsideFieldMargin)
					&& !fieldConfig.isInOwnGoalArea(sol2_x, sol2_y);
			if (sol1_inField && sol2_inField) {
				// both intersections are in field, select position closes to the side-line
				if (fabs(sol1_x) < fabs(sol2_x)) {
					receiverPosition = Vector2D(sol2_x, sol2_y);
					found = true;
				} else {
					receiverPosition = Vector2D(sol1_x, sol1_y);
					found = true;
				}
			} else if (sol1_inField) {
				receiverPosition = Vector2D(sol1_x, sol1_y);
				found = true;
			} else if (sol2_inField) {
				receiverPosition = Vector2D(sol2_x, sol2_y);
				found = true;
			}
		}
	}
	return found;
} // End of calculateSetPlayReceiverOnLobShotLinePosition


// ---------------------------------------------------------------------
bool RolePosition::calculateSetPlayReceiverConservativePosition(const FieldConfig& fieldConfig,
		const Vector2D& globalBallPosition, const PlannerOptions& plannerOptions, Vector2D& receiverPosition)
{
	bool found = false;
	double outsideFieldMargin = plannerOptions.outsideFieldMargin; // distance to stay from side of field
	double distBallReciever = plannerOptions.restart_receiver_ball_dist; // distance between receiver and ball

	// ball between our backline and lobshot line.
	// place robot on same y as ball, preferable to nearest sideline.
	int closest_direction = 1;
	if (globalBallPosition.m_x < 0) {
		closest_direction = -closest_direction; // ball is left on the field
	}
	double closest_sidelineX = globalBallPosition.m_x + (closest_direction * distBallReciever);
	if (fieldConfig.isInField(closest_sidelineX, globalBallPosition.m_y, outsideFieldMargin)) {
		// place receiver towards sideline
		receiverPosition = Vector2D(globalBallPosition.m_x + (closest_direction * distBallReciever), globalBallPosition.m_y);
	}
	else {
		// place receiver towards center of field
		receiverPosition = Vector2D(globalBallPosition.m_x - (closest_direction * distBallReciever), globalBallPosition.m_y);
	}
	found = true;
	return found;
}

// ---------------------------------------------------------------------
void RolePosition::calculateSetPlayPosition(Vector2D& shooterPosition, Vector2D& receiverPosition,
											const std::vector<TeamPlannerRobot>& Team,
		                                    const Vector2D& globalBallPosition,
											const previous_used_ball_by_planner_t& previous_global_ball,
											game_state_e gamestate,
											const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig)  {


	receiverPosition = calculateSetPlayReceiverPosition(Team, globalBallPosition, previous_global_ball,
												gamestate, plannerOptions, fieldConfig);

	// calculate kicker position on line between ball and receiver
	// and check if position of shoot is reachable, otherwise try other found alternative position
	double distBallShooter = plannerOptions.restart_shooter_ball_dist; // distance between shooter and ball
	double distBallReciever = plannerOptions.restart_receiver_ball_dist; // distance between receiver and ball

	double alfaKicker = globalBallPosition.angle(receiverPosition);
	bool shootPositionValid = false; // indicate if kicker position is valid
	double shootX = 0.0;
	double shootY = 0.0;
	double actualDistBallShooter = distBallShooter;

	while (shootPositionValid == false) {
		double distShooterReceiver = actualDistBallShooter + distBallReciever;
		shootX = receiverPosition.m_x + (cos(alfaKicker) * distShooterReceiver);
		shootY = receiverPosition.m_y + (sin(alfaKicker) * distShooterReceiver);
		if (fieldConfig.isInReachableField(shootX, shootY)) {
			// shooter position is reachable for player
			shootPositionValid = true;
		}
		else {
			// shooter position is not reachable for player
			// adjust position for the shooter

			actualDistBallShooter = actualDistBallShooter - 0.10; // try again with distance to ball 10 cm shorter.
			if (actualDistBallShooter < (fieldConfig.ROBOTSIZE * 0.5 + 0.10) ) {
				// can not move close to ball then 10 cm from the front of the robot.
				// use same x position as ball and position the front of the player cm from the ball
				shootX = globalBallPosition.m_x;
				shootY = globalBallPosition.m_y - (fieldConfig.ROBOTSIZE);
				shootPositionValid = true;
			}
		}
	}
	shooterPosition = Vector2D(shootX, shootY);
}

Vector2D RolePosition::calculateSetPlayReceiverPosition(const std::vector<TeamPlannerRobot>& Team,
		                                    const Vector2D& globalBallPosition,
											const previous_used_ball_by_planner_t& previous_global_ball,
											game_state_e gamestate,
											const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig)
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

	Vector2D receiverPosition;

	const double margin_to_side_line = 0.5;
	double distBallReciever = plannerOptions.restart_receiver_ball_dist; // distance between receiver and ball
	Vector2D goalPos = Vector2D(0, fieldConfig.getMaxFieldY());
	double topPenaltyAreaY = fieldConfig.getMaxFieldY()- (fieldConfig.getPenaltyAreaLength() + fieldConfig.getRobotRadius());
	// create situation for lob shot: receiver on line between ball and goal x meter behind ball facing to goal.
	// This only when the ball is more then x meter on opponent half (otherwise lob shot on goal is not allowed).
	// if location is outside the field, then the player will be located inside the field.
	double outsideFieldMargin = plannerOptions.outsideFieldMargin; // distance to stay from side of field

	Vector2D previousEndPos;
	for (unsigned int idx = 0; idx < Team.size(); idx++) {
		if (Team[idx].previous_result.previous_result_present == 1 && static_cast<dynamic_role_e>(Team[idx].previous_result.dynamic_role) == dr_SETPLAY_RECEIVER) {
			Vector2D previousEndPos = Vector2D(Team[idx].previous_result.end_position.x, Team[idx].previous_result.end_position.y);
			Vector2D previousBall = Vector2D(previous_global_ball.x, previous_global_ball.y);

			if (globalBallPosition.distanceTo(previousBall) < 0.26) {
				receiverPosition = previousEndPos;
				receiverPosition.m_x = receiverPosition.m_x - (globalBallPosition.m_x - previous_global_ball.x);
				receiverPosition.m_y = receiverPosition.m_y - (globalBallPosition.m_y - previous_global_ball.y);
				if (fieldConfig.isInField(receiverPosition, margin_to_side_line)) {
					return receiverPosition;
				}
			}
		}
	}


	// Prefer lobshot when possible
	// Default position is in line with ball and opponent goal.
	double alfa = globalBallPosition.angle(goalPos);
	double behind_ball_x = globalBallPosition.m_x + (cos(alfa) * distBallReciever);
	double behind_ball_y = globalBallPosition.m_y + (sin(alfa) * distBallReciever);
	double min_y_for_default_position = plannerOptions.min_y_for_lob_shot;
	bool found = false;
	bool default_in_field = fieldConfig.isInField(behind_ball_x, behind_ball_y, outsideFieldMargin);
	bool default_in_own_penalty_area = fieldConfig.isInOwnPenaltyArea(behind_ball_x, behind_ball_y, outsideFieldMargin);

	if (default_in_field && !default_in_own_penalty_area && behind_ball_y > min_y_for_default_position) {
		// lob-shot: receiver will be located at opponent half and within the field
		receiverPosition = Vector2D(behind_ball_x, behind_ball_y);
		found = true;
	}
	else {
		// no straight lob-shot possible
		found = calculateSetPlayReceiverMinTurnPosition(fieldConfig, globalBallPosition,plannerOptions, receiverPosition);
		if (found && receiverPosition.m_y < plannerOptions.min_y_for_lob_shot)
		{
			if (globalBallPosition.m_y > plannerOptions.min_y_for_lob_shot) {
				// ball is close to <plannerOptions.min_y_for_lob_shot>, but not behind it.
				// place receiver on the line <plannerOptions.min_y_for_lob_shot>
				found = calculateSetPlayReceiverOnLobShotLinePosition(fieldConfig, globalBallPosition,plannerOptions, receiverPosition);
				if (!found) {
					// could not find valid position on on the line <plannerOptions.min_y_for_lob_shot>
					found = calculateSetPlayReceiverConservativePosition(fieldConfig, globalBallPosition,plannerOptions, receiverPosition);
				}
			}
			else {
				// ball is between <plannerOptions.min_y_for_lob_shot> and our backline
				found = calculateSetPlayReceiverConservativePosition(fieldConfig, globalBallPosition,plannerOptions, receiverPosition);
			}
		}
		else {
			if ((globalBallPosition.m_y > plannerOptions.min_y_for_lob_shot) &&
			    fabs(globalBallPosition.m_y - plannerOptions.min_y_for_lob_shot) < distBallReciever) {
				// check if place on lobshot line is possible
				found = calculateSetPlayReceiverOnLobShotLinePosition(fieldConfig, globalBallPosition,plannerOptions, receiverPosition);

			}
		}
	}

	if (!found) {
		// no good position found, use original 90 degree position
		// select position 90 degrees to the line between the ball and the opponent goal
		receiverPosition = setplay_receiver_position_90deg_to_ball_goal(globalBallPosition, plannerOptions, fieldConfig);
	}

	// A position for the setplay receiver has be calculated.
	// Avoid position where the setplay receiver is positioned in front of own goal.
	// If the setplay receiver position is in front of own goal then calculate a safer position for the setplay receiver.
	// The setplay receiver should not be position in area between side of penalty area and middle circle at own half.
	// Then position the setplay receiver to nearest side-line.
	if (((fabs(receiverPosition.m_x) < (fieldConfig.getPenaltyAreaWidth() * 0.5)) &&
		 (receiverPosition.m_y < -(fieldConfig.getCenterCirleDiameter()*0.5))))
	{
		// Update receiver location because it is in front of own goal

		// find nearest side-line
		int closest_direction = 1;
		if (globalBallPosition.m_x < 0) {
			closest_direction = -closest_direction; // ball is left on the field
		}
		double closest_sidelineX = closest_direction * (fieldConfig.getMaxFieldX() - margin_to_side_line);

		// calculate intersection points of the nearest side-line with circle with radius distBallReciever with ball as center
		// use one of the intersection points as new position (position must be in the field)
		double intersection1x, intersection1y, intersection2x, intersection2y = 0.0;
		int nr_intersections_with_sideline = findLineCircleIntersections(
				globalBallPosition.m_x, globalBallPosition.m_y,distBallReciever,
				closest_sidelineX, fieldConfig.getMaxFieldY(),
				closest_sidelineX, -fieldConfig.getMaxFieldY(),
				intersection1x, intersection1y, intersection2x, intersection2y);
		// if no intersections are found: then keep original result, but this should not occur

		if (nr_intersections_with_sideline == 1) {
			if (fieldConfig.isInField(intersection1x, intersection1y, margin_to_side_line)) {
				receiverPosition = Vector2D(intersection1x, intersection1y); // select the only intersection
			}
		}
		else if (nr_intersections_with_sideline == 2) {
			// two intersections: select the intersection closest to the middle line (check if position is in the field).
			if ((intersection2y < intersection1y) && fieldConfig.isInField(intersection1x, intersection1y, margin_to_side_line)){
				receiverPosition = Vector2D(intersection1x, intersection1y);
			}
			else{
				if (fieldConfig.isInField(intersection2x, intersection2y, margin_to_side_line)) {
					receiverPosition = Vector2D(intersection2x, intersection2y);
				}
			}
		}
	}

	if (fieldConfig.isInOpponentPenaltyArea(receiverPosition.m_x, receiverPosition.m_y) ) {
		// receiver position is in Opponent Goal area AND game-state is not penalty
		// Then place the receiver on the line on top of the opponent penalty area
		double preffered_Y_receiver = topPenaltyAreaY;
		double a = preffered_Y_receiver - globalBallPosition.m_y;
		double relX = sqrt(distBallReciever*distBallReciever - a*a);
		// two positions are possible, select position closest to middle of the field
		if (fabs(globalBallPosition.m_x + relX) < fabs(globalBallPosition.m_x - relX)) {
			receiverPosition = Vector2D(globalBallPosition.m_x + relX, preffered_Y_receiver);
		}
		else {
			receiverPosition = Vector2D(globalBallPosition.m_x - relX, preffered_Y_receiver);
		}
	}
	return receiverPosition;
}

//---------------------------------------------------------------------------------------------------------------------
/**
 */
void RolePosition::GetFixedPositions(vector<Vector2D>& playerPositions, game_state_e gamestate,
		const MovingObject& globalBall, bool searchForBall,const std::vector<Vector2D>& parking_positions,
		const FieldConfig& fieldConfig, const PlannerOptions& plannerOptions) {

	bool parking_is_left = true;
	if (!parking_positions.empty()) {
		parking_is_left = (parking_positions[0].m_x < 0);
	}

	if (gamestate == game_state_e::BEGIN_POSITION) {
		/**
		 * Assign algorithm used for begin of a half (first half/second half). Assign all players to a begin position
		 * Internally a list of begin positions is created (most imported position first). Then the players will be assigned to
		 * the positions, with use of the path-calculation.
		 */
		Vector2D fordwardRight = Vector2D( 1.0, -(fieldConfig.getCenterCirleRadius() + fieldConfig.getRobotRadius())-1);
		Vector2D fordwardLeft  = Vector2D(-1.0, -(fieldConfig.getCenterCirleRadius() + fieldConfig.getRobotRadius())-1);
		if (parking_is_left) {
			playerPositions.push_back(fordwardRight);
			playerPositions.push_back(fordwardLeft);
		}
		else {
			playerPositions.push_back(fordwardLeft);
			playerPositions.push_back(fordwardRight);
		}
		Vector2D backRight = Vector2D(  (fieldConfig.getMaxFieldX()*0.5) + 0.5, -(fieldConfig.getMaxFieldY()*0.5)-0.3);
		Vector2D backLeft  = Vector2D( -(fieldConfig.getMaxFieldX()*0.5) - 0.5, -(fieldConfig.getMaxFieldY()*0.5)-0.3);
		if (parking_is_left) {
			playerPositions.push_back(backRight);
			playerPositions.push_back(backLeft);
		}
		else {
			playerPositions.push_back(backLeft);
			playerPositions.push_back(backRight);
		}
	}
	else if (gamestate == game_state_e::PARKING) {
		// select position closest to default goalie position as parking position for the goalie
		Vector2D goalieDefaultPosition = Vector2D(0, -fieldConfig.getMaxFieldY());
		Vector2D goalieParkingPosition = goalieDefaultPosition.closestTo(parking_positions);

		std::vector<Vector2D> fieldplayer_parking_positions = vector<Vector2D>();
		for (auto it = parking_positions.begin(); it != parking_positions.end(); ++it) {
			Vector2D parking_pos = *it;
			if (parking_pos.equals(goalieParkingPosition) == false) {
				fieldplayer_parking_positions.push_back(parking_pos);
			}
		}

		/**
		 * Assign algorithm used for parking game situation. Assign all players to a place in the parking lot.
		 * Internally a list of parking positions is created (most imported position first). Then the players will be assigned to
		 * the positions, with use of the path-calculation.
		 */
		if (parking_positions.size() > 0) {
			playerPositions.push_back(fieldplayer_parking_positions[0]);
		}
		if (parking_positions.size() > 1) {
			playerPositions.push_back(fieldplayer_parking_positions[1]);
		}
		if (parking_positions.size() > 2) {
			playerPositions.push_back(fieldplayer_parking_positions[2]);
		}
		if (parking_positions.size() > 3) {
			playerPositions.push_back(fieldplayer_parking_positions[3]);
		}
	}
	else if (gamestate == game_state_e::KICKOFF) {

		/**
		 * Assign players to positions for an offensive kickoff
		 * Kick off setup is determined by the KickOffSide
		 * TODO: find a better solution to integrate the 2m rule...
		 */
		playerPositions.push_back( Vector2D(plannerOptions.kickoff_fp1_x, plannerOptions.kickoff_fp1_y));
		playerPositions.push_back( Vector2D(plannerOptions.kickoff_fp2_x, plannerOptions.kickoff_fp2_y));
		playerPositions.push_back( Vector2D(plannerOptions.kickoff_fp3_x, plannerOptions.kickoff_fp3_y));
		playerPositions.push_back( Vector2D(plannerOptions.kickoff_fp4_x, plannerOptions.kickoff_fp4_y));
	}
	else if (gamestate == game_state_e::KICKOFF_AGAINST) {
		/**
		 * Static assignment of defensive positions during an kickoff against
		 *
		 * MSL rule RC-8.3:
		 *  - team taking kickoff: must be minimal 2 meter from ball except the kicker
		 *  - team with kickoff-against: Minimal 3 meters from the ball
		 */

		playerPositions.push_back( Vector2D(plannerOptions.kickoff_against_fp1_x, plannerOptions.kickoff_against_fp1_y));
		playerPositions.push_back( Vector2D(plannerOptions.kickoff_against_fp2_x, plannerOptions.kickoff_against_fp2_y));
		playerPositions.push_back( Vector2D(plannerOptions.kickoff_against_fp3_x, plannerOptions.kickoff_against_fp3_y));
		playerPositions.push_back( Vector2D(plannerOptions.kickoff_against_fp4_x, plannerOptions.kickoff_against_fp4_y));
		}
	else if (searchForBall) {
		getSearchForBallPositions(playerPositions, gamestate, fieldConfig);
	}
	//print_provided_position(gamestate, playerPositions);
}

void RolePosition::getSearchForBallPositions(vector<Vector2D>& playerPositions, game_state_e gamestate, const FieldConfig& fieldConfig)
{
	double topPenaltyAreaSearchPos = fieldConfig.getMaxFieldY()-(fieldConfig.getPenaltyAreaLength()+0.5);

	if (gamestate == game_state_e::CORNER) {
		playerPositions.push_back(Vector2D(-fieldConfig.getMaxFieldX()+1.0, +fieldConfig.getMaxFieldY()-1.0));  // left near opponent corner
		playerPositions.push_back(Vector2D( fieldConfig.getMaxFieldX()-1.0, +fieldConfig.getMaxFieldY()-1.0));  // right near opponent corner
		playerPositions.push_back(Vector2D( fieldConfig.getMaxFieldX()*0.5, -fieldConfig.getMaxFieldY()*0.5));  // right of field, just middle of own half in Y
		playerPositions.push_back(Vector2D(-fieldConfig.getMaxFieldX()*0.5, -fieldConfig.getMaxFieldY()*0.5));  // left of field just middle of own half in Y
	}
	else if (gamestate == game_state_e::CORNER_AGAINST) {
		playerPositions.push_back(Vector2D(-fieldConfig.getMaxFieldX()+1.5, -fieldConfig.getMaxFieldY()+1.5));  // left near own corner
		playerPositions.push_back(Vector2D( fieldConfig.getMaxFieldX()-1.5, -fieldConfig.getMaxFieldY()+1.5));  // right near own corner
		playerPositions.push_back(Vector2D( fieldConfig.getMaxFieldX()*0.5, +fieldConfig.getMaxFieldY()*0.5));  // right of field, just middle of opponent half in Y
		playerPositions.push_back(Vector2D(-fieldConfig.getMaxFieldX()*0.5, +fieldConfig.getMaxFieldY()*0.5));  // left of field just middle of opponent half in Y
	}
	else if ((gamestate == game_state_e::GOALKICK) || (gamestate == game_state_e::PENALTY_AGAINST) || (gamestate == game_state_e::PENALTY_SHOOTOUT_AGAINST)) {
		playerPositions.push_back(Vector2D(-fieldConfig.getMaxFieldX()*0.5, -topPenaltyAreaSearchPos));  // left near top own penalty area
		playerPositions.push_back(Vector2D( fieldConfig.getMaxFieldX()*0.5, -topPenaltyAreaSearchPos));  // right near top own penalty area
		playerPositions.push_back(Vector2D( fieldConfig.getMaxFieldX()*0.5, +fieldConfig.getMaxFieldY()*0.5));  // right of field, just middle of opponent half in Y
		playerPositions.push_back(Vector2D(-fieldConfig.getMaxFieldX()*0.5, +fieldConfig.getMaxFieldY()*0.5));  // left of field just middle of opponent half in Y
	}
	else if ((gamestate == game_state_e::GOALKICK_AGAINST) || (gamestate == game_state_e::PENALTY) || (gamestate == game_state_e::PENALTY_SHOOTOUT)) {
		playerPositions.push_back(Vector2D(-fieldConfig.getMaxFieldX()*0.5, +topPenaltyAreaSearchPos));  // left near top opponent corner
		playerPositions.push_back(Vector2D( fieldConfig.getMaxFieldX()*0.5, +topPenaltyAreaSearchPos));  // right near top opponent corner
		playerPositions.push_back(Vector2D( fieldConfig.getMaxFieldX()*0.5, -fieldConfig.getMaxFieldY()*0.5));  // right of field, just middle of own half in Y
		playerPositions.push_back(Vector2D(-fieldConfig.getMaxFieldX()*0.5, -fieldConfig.getMaxFieldY()*0.5));  // left of field just middle of own half in Y

	}
	else if ((gamestate == game_state_e::THROWIN) || (gamestate == game_state_e::THROWIN_AGAINST)) {
		double x_dist = fieldConfig.getMaxFieldX()-1.5;
		playerPositions.push_back(Vector2D(-x_dist, -(fieldConfig.getMaxFieldY()*0.5)));  // left of field, just middle of own half in Y
		playerPositions.push_back(Vector2D( x_dist, +(fieldConfig.getMaxFieldY()*0.5)));  // right of field, just middle of opponent half in Y
		playerPositions.push_back(Vector2D( x_dist, -(fieldConfig.getMaxFieldY()*0.5)));  // right of field, just middle of own half in Y
		playerPositions.push_back(Vector2D(-x_dist, +(fieldConfig.getMaxFieldY()*0.5)));  // left of field, just middle of opponent half in Y
	}
	else {
		// any other situation
		playerPositions.push_back(Vector2D(-fieldConfig.getMaxFieldX()*0.5, -(fieldConfig.getMaxFieldY()*0.5)));  // left of field, just middle of own half in Y
		playerPositions.push_back(Vector2D( fieldConfig.getMaxFieldX()*0.5, +(fieldConfig.getMaxFieldY()*0.5)));  // right of field, just middle of opponent half in Y
		playerPositions.push_back(Vector2D( fieldConfig.getMaxFieldX()*0.5, -(fieldConfig.getMaxFieldY()*0.5)));  // right of field, just middle of own half in Y
		playerPositions.push_back(Vector2D(-fieldConfig.getMaxFieldX()*0.5, +(fieldConfig.getMaxFieldY()*0.5)));  // left of field, just middle of opponent half in Y
	}
}

//---------------------------------------------------------------------------------------------------------------------
void RolePosition::print_provided_position(game_state_e gamestate, const vector<vector<Vector2D>>& positions) {
//	logAlways("Provided position info: ");
//	logAlways("game-state: %d (%s)",gamestate, GameStateAsString(gamestate).c_str());
//	for (unsigned int idx = 0; idx < positions.size(); idx++) {
//		logAlways("position- vector [%d]", idx);
//		for (unsigned int v_idx = 0; v_idx < positions[idx].size(); v_idx++) {
//			logAlways("\tposition[%d] = %s", v_idx, positions[idx][v_idx].toString().c_str());
//		}
//	}
}

Vector2D RolePosition::InterceptorNormalPlayPosition(planner_target_e& planner_target,
		const MovingObject& globalBall, const std::vector<TeamPlannerRobot>& Team, const std::vector<TeamPlannerOpponent>& Opponents,
		const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig) {
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

	Vector2D ballPos = globalBall.getPosition().getVector2D();
	Vector2D rolePosition = ballPos; // set default position to the ball position


	// determine the smallest distance from any opponent to the ball
	double smallestDistOpponentToBall = fieldConfig.getMaxPossibleFieldDistance();
	for (auto it = Opponents.begin(); it != Opponents.end(); ++it) {
		smallestDistOpponentToBall = min(it->position.getPosition().getVector2D().distanceTo(ballPos), smallestDistOpponentToBall);
	}

	bool ballInArea = plannerOptions.priority_block_check_ball_in_area && (ballPos.m_y <= plannerOptions.priority_block_max_ball_y);
	bool opponentCloseToBall = plannerOptions.priority_block_check_opponent_close_to_ball && smallestDistOpponentToBall < plannerOptions.priority_block_max_opponent_to_ball_dist;

	// CHECK if priority block must be performed
	if (plannerOptions.priority_block_apply && ballInArea && opponentCloseToBall) {
		// priority block only when:
		// - ball.Y is below threshold (original default own half)
		// - opponent is close to the ball
		// - priority block option is enabled

		Vector2D goalPos = Vector2D(0.0, -fieldConfig.getMaxFieldY());
		double distBall2Goal = goalPos.distanceTo(ballPos);
		double smallestDistToBallOnTheDefenseLine = fieldConfig.getMaxPossibleFieldDistance();
		unsigned nr_field_players_between_ball_and_own_goal = 0;
		double maxDistanceToBall = plannerOptions.priority_block_max_distance;  // safe distance to ball (used to prevent players move around ball).
		double minDistanceToBall = plannerOptions.priority_block_min_distance;  // safe distance to ball (used to prevent players move around ball).


		// count number of field-players on the line between ball and own goal.
		for (auto it = Team.begin(); it != Team.end(); ++it) {
			if( it->player_type == player_type_e::FIELD_PLAYER && !(it->assigned)) {
				// only unassigned field-players

				Vector2D playerPos = it->position.getPosition().getVector2D();
				double distToDefenseLine = getDistanceFromPointToLine(ballPos.m_x, ballPos.m_y, goalPos.m_x, goalPos.m_y, playerPos.m_x, playerPos.m_y);
				double distToGoal = playerPos.distanceTo(goalPos);
				double distToBall = playerPos.distanceTo(ballPos);

				if ((distToGoal < distBall2Goal) &&
				    (distToDefenseLine < plannerOptions.priority_block_max_distance_to_defense_line) &&
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
		double goalToBallAngle = ballPos.angle(goalPos);
		if (nr_field_players_between_ball_and_own_goal >= 1) {
			// Select position on the defense line, close to the ball (intended: drive towards the ball if on defense line)
			double targetDistToBall = min(smallestDistToBallOnTheDefenseLine, plannerOptions.priority_block_max_distance); // take smallest: to prevent moving backwards
			targetDistToBall = max(targetDistToBall, plannerOptions.priority_block_min_distance); // take biggest: to prevent moving too much forward
			double priorityBlockX = ballPos.m_x - (cos(goalToBallAngle) * targetDistToBall);
			double priorityBlockY = ballPos.m_y - (sin(goalToBallAngle) * targetDistToBall);
			rolePosition = Vector2D(priorityBlockX, priorityBlockY); // set role position on the priority block location
		}
		else {
			// No field-player on the defense line
			// calculate the safe priority block position (last position on line between ball and own goal)
			// calculate distance from maxPriorityBlockPoint to goal
			double maxPriorityBlockX = ballPos.m_x - (cos(goalToBallAngle) * maxDistanceToBall);
			double maxPriorityBlockY = ballPos.m_y - (sin(goalToBallAngle) * maxDistanceToBall);
			Vector2D maxPriorityBlockPoint = Vector2D(maxPriorityBlockX, maxPriorityBlockY);
			double minPriorityBlockX = ballPos.m_x - (cos(goalToBallAngle) * minDistanceToBall);
			double minPriorityBlockY = ballPos.m_y - (sin(goalToBallAngle) * minDistanceToBall);
			Vector2D minPriorityBlockPoint = Vector2D(minPriorityBlockX, minPriorityBlockY);


			// find team-mate which is closest to defense line then the priority block position
			// (from ball to safe priority block position - (between ball and own goal))
			Vector2D bestPos = maxPriorityBlockPoint;
			for (auto it = Team.begin(); it != Team.end(); ++it) {
				if( it->player_type == player_type_e::FIELD_PLAYER && !(it->assigned)) {
					// only unassigned field-players
					Vector2D teamMatePos = it->position.getPosition().getVector2D();
					// calculate second location of on the perpendicular to the defense line
					double tx1 = teamMatePos.m_x + (cos(goalToBallAngle+M_PI*0.5) * 1.0);
					double ty1 = teamMatePos.m_y + (sin(goalToBallAngle+M_PI*0.5) * 1.0);

					// calculate intersection of the perpendicular of the defense line with the defense line
					double intersectX = 0;
					double intersectY = 0;
					bool linesIntersect = getIntersectionOfTwoLines(intersectX, intersectY, tx1, ty1, teamMatePos.m_x, teamMatePos.m_y,
							maxPriorityBlockX, maxPriorityBlockY, goalPos.m_x, goalPos.m_y);
					Vector2D intersectionPoint = Vector2D(intersectX, intersectY);
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

int RolePosition::FindMostDangerousOpponentAndNotAssigned(
		const MovingObject& globalBall, const FieldConfig& fieldConfig,
		const PlannerOptions& plannerOptions,
		const std::vector<TeamPlannerOpponent>& Opponents)
/* find the opponent who is most dangerous and not yet assigned (= defended)
 * return index of opponent in Opponents vector, return -1 if no opponent found to defend.
 */
{
	int closest_to_idx = -1;

	// find closest to ball
	vector<double> oppenent_to_ball_dist = vector<double>();
	vector<double> oppenent_to_goal_dist = vector<double>();
	Vector2D ballPos = globalBall.getPosition().getVector2D();
	Vector2D goalPos = Vector2D(0, -fieldConfig.getMaxFieldY());
	// get distances for each opponent to ball and our goal.
	for (unsigned idx = 0; idx < Opponents.size(); idx++) {
		Vector2D opponent_position = Opponents[idx].position.getXYlocation();
		oppenent_to_ball_dist.push_back(ballPos.distanceTo(opponent_position));
		oppenent_to_goal_dist.push_back(goalPos.distanceTo(opponent_position));
	}

	// assume opponent closest to our goal is most dangerous
	Vector2D opponentGoal = Vector2D(0, fieldConfig.getMaxFieldY());
	double closest_dist_to = std::numeric_limits<double>::infinity();
	for (unsigned idx = 0; idx < Opponents.size(); idx++) {
		if (Opponents[idx].position.getXYlocation().distanceTo(opponentGoal) > plannerOptions.dist_to_goal_to_mark_opponent_as_goalie) {
			// Not opponent goalie  (too far from opponent goal)
			if (oppenent_to_goal_dist[idx] < closest_dist_to
					&& Opponents[idx].assigned == false) {
				// opponent is not defended yet and closest to the ball
				closest_dist_to = oppenent_to_goal_dist[idx];
				closest_to_idx = idx;
			}
		}

	}
	return closest_to_idx;
}

int RolePosition::FindOpponentClostestToPositionAndNotAssigned(
		const Vector2D& targetPos, const FieldConfig& fieldConfig,
		const PlannerOptions& plannerOptions,
		const std::vector<TeamPlannerOpponent>& Opponents)
/* find the opponent closest to the ball and not yet assigned (= defended)
 * return index of opponent in Opponents vector, return -1 if no opponent found to defend.
 */
{
	int closest_to_idx = -1;

	// find closest to ball
	vector<double> oppenent_to_ball_dist = vector<double>();
	vector<double> oppenent_to_goal_dist = vector<double>();
	Vector2D goalPos = Vector2D(0, -fieldConfig.getMaxFieldY());
	// get distances for each opponent to ball and our goal.
	for (unsigned idx = 0; idx < Opponents.size(); idx++) {
		Vector2D opponent_position = Opponents[idx].position.getXYlocation();
		oppenent_to_ball_dist.push_back(targetPos.distanceTo(opponent_position));
		oppenent_to_goal_dist.push_back(goalPos.distanceTo(opponent_position));
	}

	Vector2D opponentGoal = Vector2D(0, fieldConfig.getMaxFieldY());
	double closest_dist_to = std::numeric_limits<double>::infinity();
	for (unsigned idx = 0; idx < Opponents.size(); idx++) {
		if (Opponents[idx].position.getXYlocation().distanceTo(opponentGoal) > plannerOptions.dist_to_goal_to_mark_opponent_as_goalie) {
			// Not opponent goalie  (too far from opponent goal)
			if (oppenent_to_ball_dist[idx] < closest_dist_to
					&& Opponents[idx].assigned == false) {
				// opponent is not defended yet and closest to the ball
				closest_dist_to = oppenent_to_ball_dist[idx];
				closest_to_idx = idx;
			}
		}
	}
	return closest_to_idx;
}

//--------------------------------------------------------------------------
Vector2D RolePosition::calculateManToManDefensePosition(
		defend_info_t& rDefend_info, dynamic_role_e dynamic_role,
		const MovingObject& globalBall, const FieldConfig& fieldConfig,
		game_state_e gamestate, const PlannerOptions& plannerOptions,
		std::vector<TeamPlannerOpponent>& Opponents, std::vector<TeamPlannerRobot>& Team, int& r_gridFileNumber,
		bool setPlayActive, bool teamControlBall) {
	// man coverage for all opponents
	Vector2D rolePosition = Vector2D();

	rDefend_info.valid = false;
	int closest_to_ball_idx = FindMostDangerousOpponentAndNotAssigned(globalBall, fieldConfig, plannerOptions, Opponents);
	if (closest_to_ball_idx >= 0) {
		// Found opponent closest to the ball. Apply man defense for this opponent
		Opponents[closest_to_ball_idx].assigned = true;
		rolePosition = TeamPlanner_Grid::findManToManDefensivePosition(dynamic_role,
				Opponents[closest_to_ball_idx].position.getXYlocation(), Team,
				gamestate, plannerOptions, globalBall, Opponents,
				r_gridFileNumber++, fieldConfig, setPlayActive, teamControlBall);
		rDefend_info.valid = true;
		rDefend_info.defending_id = Opponents[closest_to_ball_idx].position.getLabel();
		rDefend_info.dist_from_defending_id = Opponents[closest_to_ball_idx].position.getXYlocation().distanceTo(rolePosition);
		rDefend_info.between_ball_and_defending_pos = plannerOptions.manDefenseBetweenBallAndPlayer;

	} else {
		// no opponent to defend left, find defensive position
		rolePosition = TeamPlanner_Grid::findDefensivePosition(Team, gamestate,
				plannerOptions, globalBall, Opponents, r_gridFileNumber++,
				fieldConfig);
	}
	return rolePosition;
}
