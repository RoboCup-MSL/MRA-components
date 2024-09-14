/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */
#include "RolePosition.hpp"

#include "TeamPlannerGrid.hpp"  // grid related teamplanner functions
#include "planner_types.hpp"
#include "Dynamics.hpp"
#include "MathUtils.hpp"

#include "logging.hpp"
#include "geometry.hpp"

#include <cmath>
#include <iostream>
#include <limits>


using namespace std;
using namespace MRA;


MRA::Geometry::Point RolePosition::determineDynamicRolePosition(defend_info_t& rDefend_info, planner_target_e& planner_target, int& r_gridFileNumber, dynamic_role_e dynamic_role,
        TeamPlannerData& r_teamplannerData, bool playerPassedBall, bool& r_role_position_is_end_position_of_pass) {
    MRA::Geometry::Point rolePosition = MRA::Geometry::Point(0,0);
    rDefend_info.defending_id = -1;    // no item to defend
    rDefend_info.valid = false;
    r_role_position_is_end_position_of_pass = false;

    if (r_teamplannerData.parameters.man_to_man_defense_during_setplay_against &&
        (r_teamplannerData.gamestate == game_state_e::GOALKICK_AGAINST || r_teamplannerData.gamestate == game_state_e::CORNER_AGAINST ||
                r_teamplannerData.gamestate == game_state_e::FREEKICK_AGAINST || r_teamplannerData.gamestate == game_state_e::THROWIN_AGAINST     ) )
    {
        // Setplay against: man coverage for all opponents
        rolePosition = calculateManToManDefensePosition(rDefend_info, dynamic_role, r_teamplannerData, r_gridFileNumber, true /* setplay active */);
        return rolePosition;
    }


    switch (dynamic_role) {
    case dr_PENALTY_KICKER: {
        //        In RoboCup, the following modification apply:
        //        • The defending goal keeper stays within his own goal area until the ball
        //        is even slightly moved.
        //        • The goalie can move at any time as long as it does no leave its goal area.
        //        • In case of an end-of-game penalty sequence, the kicker is located inside
        //        the center circle. For an in-game penalty the kicker is located on a line
        //        from the penalty spot to the center circle, approximately one meter from
        //        the penalty spot.
        //        • In case of an end-of-game penalty sequence, the players other than the
        //        kicker and goalie are located inside the field of play, outside the center
        //        circle, and behind the center line on the opposite side of the defending
        //        goal keeper. For an in-game penalty, the players other than the kicker
        //        and goalie are located inside the field of play, at least three meters away
        //        from the penalty spot and not in the penalty area.
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
        MRA::Geometry::Position ballPositionToUse = r_teamplannerData.ball.position;
        if (r_teamplannerData.parameters.use_pass_to_position_for_attack_support && r_teamplannerData.pass_data.valid) {
            ballPositionToUse.x = r_teamplannerData.pass_data.target_pos.x;
            ballPositionToUse.y = r_teamplannerData.pass_data.target_pos.y;
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
            TeamPlanner_Grid::findAttackSupportPosition(rolePosition, r_teamplannerData, ballPositionToUse, r_gridFileNumber++, position_close_to_ball);
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
            r_teamplannerData.gamestate == game_state_e::THROWIN_AGAINST     )
        {
            rolePosition = TeamPlanner_Grid::findInterceptorPositionDuringRestart(r_teamplannerData, r_gridFileNumber++);
        }
        else {
            MRA::Geometry::Point intercept_position = r_teamplannerData.ball.position;
            if ((r_teamplannerData.ball_status == ball_status_e::FREE or r_teamplannerData.ball_status == ball_status_e::OWNED_BY_TEAMMATE) and
                r_teamplannerData.parameters.interceptor_assign_use_ball_velocity and
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
    case dr_IS_ALIVE:
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
    MRA::Geometry::Point receiverPosition;
    double distBallReceiver = r_teamplannerData.parameters.restart_receiver_ball_dist; // distance between receiver and ball
    MRA::Geometry::Point goalPos = MRA::Geometry::Point(0, r_teamplannerData.fieldConfig.getMaxFieldY());
    double alfa = MRA::Geometry::Point(r_teamplannerData.ball.position.x, r_teamplannerData.ball.position.y).angle(goalPos);

    double outsideFieldMargin = r_teamplannerData.parameters.outsideFieldMargin; // distance to stay from side of field
    double recieveX_plus_90deg = r_teamplannerData.ball.position.x + (cos(alfa + 0.5 * M_PI) * distBallReceiver);
    double recieveY_plus_90deg = r_teamplannerData.ball.position.y + (sin(alfa + 0.5 * M_PI) * distBallReceiver);
    double recieveX_min_90deg = r_teamplannerData.ball.position.x  + (cos(alfa - 0.5 * M_PI) * distBallReceiver);
    double recieveY_min_90deg = r_teamplannerData.ball.position.y  + (sin(alfa - 0.5 * M_PI) * distBallReceiver);
    bool min90_inField = r_teamplannerData.fieldConfig.isInField(recieveX_min_90deg, recieveY_min_90deg, outsideFieldMargin);
    bool plus90_inField = r_teamplannerData.fieldConfig.isInField(recieveX_plus_90deg, recieveY_plus_90deg, outsideFieldMargin);
    if (min90_inField && plus90_inField) {
        // both positions are in the field, select the one closest to opponent goal.
        if (recieveY_min_90deg > recieveY_plus_90deg) {
            receiverPosition = MRA::Geometry::Point(recieveX_min_90deg, recieveY_min_90deg);
        }
        else {
            receiverPosition = MRA::Geometry::Point(recieveX_plus_90deg, recieveY_plus_90deg);
        }
    } else if (min90_inField) {
        receiverPosition = MRA::Geometry::Point(recieveX_min_90deg, recieveY_min_90deg);
    } else {
        receiverPosition = MRA::Geometry::Point(recieveX_plus_90deg, recieveY_plus_90deg);
    }

    // in corner area it might go wrong and a Y position outside the field is selected.
    if (!r_teamplannerData.fieldConfig.isInField(receiverPosition.x, receiverPosition.y, outsideFieldMargin)) {
        if (!r_teamplannerData.fieldConfig.isInField(0, receiverPosition.y,    outsideFieldMargin)) {
            // y-location is outside the field.
            if (recieveY_min_90deg > recieveY_plus_90deg) {
                receiverPosition = MRA::Geometry::Point(recieveX_plus_90deg,
                        recieveY_plus_90deg);
            } else {
                receiverPosition = MRA::Geometry::Point(recieveX_min_90deg,
                        recieveY_min_90deg);
            }
        }
        // x-position can be field
        if (receiverPosition.x < -(r_teamplannerData.fieldConfig.getMaxFieldX() - outsideFieldMargin)) {
            receiverPosition.x = -(r_teamplannerData.fieldConfig.getMaxFieldX()    - outsideFieldMargin);
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
    double distBallReceiver = r_teamplannerData.parameters.restart_receiver_ball_dist; // distance between receiver and ball
    double sol1_x, sol1_y, sol2_x, sol2_y = 0;
    double maxFieldX = r_teamplannerData.fieldConfig.getMaxFieldX();
    double maxFieldY = r_teamplannerData.fieldConfig.getMaxFieldY();
    int nr_crossings = 0;
    double backline_y = -r_teamplannerData.fieldConfig.getMaxFieldY() + outsideFieldMargin;
    // check crossings with last y-line where receiver putting back is allowed.
    nr_crossings = findLineCircleIntersections(r_teamplannerData.ball.position.x,
            r_teamplannerData.ball.position.y, distBallReceiver, -maxFieldX, backline_y,
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
                distBallReceiver, closest_sidelineX, -maxFieldY,
                closest_sidelineX, maxFieldY, side_sol1_x, side_sol1_y,
                side_sol2_x, side_sol2_y);
        double paw = r_teamplannerData.fieldConfig.getPenaltyAreaWidth();
        double pal = r_teamplannerData.fieldConfig.getPenaltyAreaLength();
        double dist_to_side_penalty_area = 0.5 * paw
                + r_teamplannerData.parameters.setplay_margin_to_penalty_area_side;
        // find intersection with penalty area. side and top
        int nr_crossings_penalty = findLineCircleIntersections(
                r_teamplannerData.ball.position.x, r_teamplannerData.ball.position.y,
                distBallReceiver, closest_direction * dist_to_side_penalty_area,
                -maxFieldY, closest_direction * dist_to_side_penalty_area,
                -(maxFieldY - pal), penalty_sol1_x, penalty_sol1_y,
                penalty_sol2_x, penalty_sol2_y);
        int nr_crossings_penalty2 = findLineCircleIntersections(
                r_teamplannerData.ball.position.x, r_teamplannerData.ball.position.y,
                distBallReceiver, -paw, -(maxFieldY - pal), +paw,
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
    double distBallReceiver = r_teamplannerData.parameters.restart_receiver_ball_dist; // distance between receiver and ball
    double sol1_x, sol1_y, sol2_x, sol2_y = 0;
    double maxFieldX = r_teamplannerData.fieldConfig.getMaxFieldX();
    int nr_crossings = 0;
    double lobshot_line = r_teamplannerData.parameters.min_y_for_lob_shot;
    // check crossings with lobshot line where receiver putting back is allowed.
    nr_crossings = findLineCircleIntersections(r_teamplannerData.ball.position.x,
            r_teamplannerData.ball.position.y, distBallReceiver, -maxFieldX, lobshot_line,
            maxFieldX, lobshot_line, sol1_x, sol1_y, sol2_x, sol2_y);
    if (nr_crossings > 0) {
        // at least 1 intersection found the lobshot line
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
    }
    return found;
} // End of calculateSetPlayReceiverOnLobShotLinePosition


// ---------------------------------------------------------------------
bool RolePosition::calculateSetPlayReceiverConservativePosition(const TeamPlannerData& r_teamplannerData, MRA::Geometry::Point& receiverPosition)
{
    bool found = false;
    double outsideFieldMargin = r_teamplannerData.parameters.outsideFieldMargin; // distance to stay from side of field
    double distBallReceiver = r_teamplannerData.parameters.restart_receiver_ball_dist; // distance between receiver and ball

    // ball between our backline and lobshot line.
    // place robot on same y as ball, preferable to nearest sideline.
    int closest_direction = 1;
    if (r_teamplannerData.ball.position.x < 0) {
        closest_direction = -closest_direction; // ball is left on the field
    }
    double closest_sidelineX = r_teamplannerData.ball.position.x + (closest_direction * distBallReceiver);
    if (r_teamplannerData.fieldConfig.isInField(closest_sidelineX, r_teamplannerData.ball.position.y, outsideFieldMargin)) {
        // place receiver towards sideline
        receiverPosition = MRA::Geometry::Point(r_teamplannerData.ball.position.x + (closest_direction * distBallReceiver), r_teamplannerData.ball.position.y);
    }
    else {
        // place receiver towards center of field
        receiverPosition = MRA::Geometry::Point(r_teamplannerData.ball.position.x - (closest_direction * distBallReceiver), r_teamplannerData.ball.position.y);
    }
    found = true;
    return found;
}

// ---------------------------------------------------------------------
void RolePosition::calculateSetPlayPosition(MRA::Geometry::Point& shooterPosition, MRA::Geometry::Point& receiverPosition, const TeamPlannerData& r_teamplannerData)  {

    // calculate kicker position on line between ball and receiver
    // and check if position of shoot is reachable, otherwise try other found alternative position

    receiverPosition = calculateSetPlayReceiverPosition(r_teamplannerData);
    shooterPosition = calculateSetPlayerKickerPosition(receiverPosition, r_teamplannerData);
}

MRA::Geometry::Point RolePosition::calculateSetPlayerKickerPosition(const MRA::Geometry::Point& receiverPosition, const TeamPlannerData& r_teamplannerData) {
    double distBallShooter = r_teamplannerData.parameters.restart_shooter_ball_dist; // distance between shooter and ball
    double distBallReceiver = r_teamplannerData.ball.position.distanceTo(receiverPosition);

    double alfaKicker = MRA::Geometry::Point(r_teamplannerData.ball.position.x, r_teamplannerData.ball.position.y).angle(receiverPosition);
    bool shootPositionValid = false; // indicate if kicker position is valid
    double shootX = 0.0;
    double shootY = 0.0;
    double actualDistBallShooter = distBallShooter;

    while (shootPositionValid == false) {
        double distShooterReceiver = actualDistBallShooter + distBallReceiver;
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

    return MRA::Geometry::Point(shootX, shootY);

}

MRA::Geometry::Point RolePosition::calculateSetPlayReceiverPosition(const TeamPlannerData& r_teamplannerData)
{
    // CODE only valid for replay !!!
    // calculation assume that no opponent or team mate is at calculated position. This is only true for restart.
    // Ball position will be the base of the positioning.
    // Default players will be on the line between ball and opponent goal
    //     Except if ball is close to the back-line. The player will be on the line between the ball and half of the opponent half (par example during corner)
    //  Step 1: calculate position for receiver (as described above)
    //  If the best position of the receiver is in the penalty area (or behind the goal)
    //          then the receiver will be located on top of the penalty area.
    //  Step 2: locate shooter on the line between ball  and receiver.

    MRA::Geometry::Point receiverPosition;

    const double margin_to_side_line = 0.5;
    double distBallReceiver = r_teamplannerData.parameters.restart_receiver_ball_dist; // distance between receiver and ball
    MRA::Geometry::Point goalPos = MRA::Geometry::Point(0, r_teamplannerData.fieldConfig.getMaxFieldY());
    double topPenaltyAreaY = r_teamplannerData.fieldConfig.getMaxFieldY()- (r_teamplannerData.fieldConfig.getPenaltyAreaLength() + r_teamplannerData.fieldConfig.getRobotRadius());
    // create situation for lob shot: receiver on line between ball and goal x meter behind ball facing to goal.
    // This only when the ball is more then x meter on opponent half (otherwise lob shot on goal is not allowed).
    // if location is outside the field, then the player will be located inside the field.
    double outsideFieldMargin = r_teamplannerData.parameters.outsideFieldMargin; // distance to stay from side of field

    MRA::Geometry::Point previousEndPos;
    for (unsigned int idx = 0; idx < r_teamplannerData.team.size(); idx++) {
        if (r_teamplannerData.team[idx].previous_result.previous_result_present == 1 && static_cast<dynamic_role_e>(r_teamplannerData.team[idx].previous_result.dynamic_role) == dr_SETPLAY_RECEIVER) {
            MRA::Geometry::Point previousEndPos = MRA::Geometry::Point(r_teamplannerData.team[idx].previous_result.end_position.x, r_teamplannerData.team[idx].previous_result.end_position.y);
            MRA::Geometry::Point previousBall = MRA::Geometry::Point(r_teamplannerData.previous_ball.x, r_teamplannerData.previous_ball.y);

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
    double alfa = MRA::Geometry::Point(r_teamplannerData.ball.position.x, r_teamplannerData.ball.position.y).angle(goalPos);
    double behind_ball_x = r_teamplannerData.ball.position.x + (cos(alfa) * distBallReceiver);
    double behind_ball_y = r_teamplannerData.ball.position.y + (sin(alfa) * distBallReceiver);
    double min_y_for_default_position = r_teamplannerData.parameters.min_y_for_lob_shot;
    bool found = false;
    bool default_in_field = r_teamplannerData.fieldConfig.isInField(behind_ball_x, behind_ball_y, outsideFieldMargin);
    bool default_in_own_penalty_area = r_teamplannerData.fieldConfig.isInOwnPenaltyArea(behind_ball_x, behind_ball_y, outsideFieldMargin);

    if (default_in_field && !default_in_own_penalty_area && behind_ball_y > min_y_for_default_position) {
        // lob-shot: receiver will be located at opponent half and within the field
        receiverPosition = MRA::Geometry::Point(behind_ball_x, behind_ball_y);
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
                fabs(r_teamplannerData.ball.position.y - r_teamplannerData.parameters.min_y_for_lob_shot) < distBallReceiver) {
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

        // calculate intersection points of the nearest side-line with circle with radius distBallReceiver with ball as center
        // use one of the intersection points as new position (position must be in the field)
        double intersection1x, intersection1y, intersection2x, intersection2y = 0.0;
        int nr_intersections_with_sideline = findLineCircleIntersections(
                r_teamplannerData.ball.position.x, r_teamplannerData.ball.position.y,distBallReceiver,
                closest_sidelineX, r_teamplannerData.fieldConfig.getMaxFieldY(),
                closest_sidelineX, -r_teamplannerData.fieldConfig.getMaxFieldY(),
                intersection1x, intersection1y, intersection2x, intersection2y);
        // if no intersections are found: then keep original result, but this should not occur

        if (nr_intersections_with_sideline == 1) {
            if (r_teamplannerData.fieldConfig.isInField(intersection1x, intersection1y, margin_to_side_line)) {
                receiverPosition = MRA::Geometry::Point(intersection1x, intersection1y); // select the only intersection
            }
        }
        else if (nr_intersections_with_sideline == 2) {
            // two intersections: select the intersection closest to the middle line (check if position is in the field).
            if ((intersection2y < intersection1y) && r_teamplannerData.fieldConfig.isInField(intersection1x, intersection1y, margin_to_side_line)){
                receiverPosition = MRA::Geometry::Point(intersection1x, intersection1y);
            }
            else{
                if (r_teamplannerData.fieldConfig.isInField(intersection2x, intersection2y, margin_to_side_line)) {
                    receiverPosition = MRA::Geometry::Point(intersection2x, intersection2y);
                }
            }
        }
    }

    if (r_teamplannerData.fieldConfig.isInOpponentPenaltyArea(receiverPosition.x, receiverPosition.y) ) {
        // receiver position is in Opponent Goal area AND game-state is not penalty
        // Then place the receiver on the line on top of the opponent penalty area
        double preffered_Y_receiver = topPenaltyAreaY;
        double a = preffered_Y_receiver - r_teamplannerData.ball.position.y;
        double relX = legRightTriangle(distBallReceiver, a);
        // two positions are possible, select position closest to middle of the field
        if (fabs(r_teamplannerData.ball.position.x + relX) < fabs(r_teamplannerData.ball.position.x - relX)) {
            receiverPosition = MRA::Geometry::Point(r_teamplannerData.ball.position.x + relX, preffered_Y_receiver);
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
    // provide list of fixed positions for the field-players for the defined game-situation.

    if (r_teamplannerData.gamestate == game_state_e::BEGIN_POSITION) {
        // provide list for fixed positions in case of BEGIN_POSITION for the field-players
        // BEGIN_POSITION normally occurs before begin of a half (first half/second half).

        // Begin-positions:
        // - Goalie in goal.
        // - For field players: 5 positions:
        //   - 2 (inner) positions in middle near are close the middle circle (forward-positions: Left, Right)
        //   - 2 (outer) positions more close to own goal (back-positions: Left, Right)
        //   - goalie position  in case their is not goalie set but team have 5 players

        // define forward positions based on parking location.
        MRA::Geometry::Point fordwardRight = MRA::Geometry::Point( 1.0, -(r_teamplannerData.fieldConfig.getCenterCirleRadius() + r_teamplannerData.fieldConfig.getRobotRadius())-1);
        MRA::Geometry::Point fordwardLeft  = MRA::Geometry::Point(-1.0, -(r_teamplannerData.fieldConfig.getCenterCirleRadius() + r_teamplannerData.fieldConfig.getRobotRadius())-1);
        playerPositions.push_back(fordwardRight);
        playerPositions.push_back(fordwardLeft);
        // define back position based on parking position
        MRA::Geometry::Point backRight = MRA::Geometry::Point(  (r_teamplannerData.fieldConfig.getMaxFieldX()*0.5) + 0.5, -(r_teamplannerData.fieldConfig.getMaxFieldY()*0.5)-0.3);
        MRA::Geometry::Point backLeft  = MRA::Geometry::Point( -(r_teamplannerData.fieldConfig.getMaxFieldX()*0.5) - 0.5, -(r_teamplannerData.fieldConfig.getMaxFieldY()*0.5)-0.3);
        playerPositions.push_back(backRight);
        playerPositions.push_back(backLeft);

        // add goalie begin position for the case there is no goalie present
        double goalieYPosition = -r_teamplannerData.fieldConfig.getMaxFieldY()+0.5;
        MRA::Geometry::Point goaliePosition = MRA::Geometry::Point(0, goalieYPosition); // center of the goal;
        playerPositions.push_back(goaliePosition);
    }
    else if (r_teamplannerData.gamestate == game_state_e::PARKING) {
        // provide list for fixed positions in case of PARKING

        // select position closest to default goalie position as parking position for the goalie
        MRA::Geometry::Point goalieDefaultPosition = MRA::Geometry::Point(0, -r_teamplannerData.fieldConfig.getMaxFieldY());
        MRA::Geometry::Point goalieParkingPosition = closestTo(goalieDefaultPosition, r_teamplannerData.parking_positions);

        // provide list for fixed positions in case of PARKING for the field-players
        // Parking-positions:
        // - Parking-position for Goalie: closest to own goal.
        // - For field players: 5 positions:
        //   - add parking position in order of provided list of parking positions, but skip the position for goalie
        //   - Parking-position for Goalie in case their is not goalie set but team have 5 players

        std::vector<MRA::Geometry::Point> fieldplayer_parking_positions = vector<MRA::Geometry::Point>();
        // add parking spots in reversed order
        for (auto it = r_teamplannerData.parking_positions.rbegin(); it != r_teamplannerData.parking_positions.rend(); ++it) {
            MRA::Geometry::Point parking_pos = *it;
            if (parking_pos.equals(goalieParkingPosition) == false) {
                playerPositions.push_back(parking_pos);
            }
        }
        // add goalie parking position for the case there is no goalie present
        playerPositions.push_back(goalieParkingPosition);

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

void RolePosition::getSearchForBallPositions(std::vector<Geometry::Point>& playerPositions, const TeamPlannerData& r_teamplannerData)
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
void RolePosition::print_provided_position(game_state_e gamestate, const vector<vector<MRA::Geometry::Point>>& positions) {
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
// Set role-position to ball-position (default)
//
// iF ball_status is OWNED BY OPPONENT THEN
//     IF any unassigned field-player is on the the defense line (= the line between the opponent closest to the ball and the own goal)
//        THEN role-position will be <options.priority_block_min_distance> m from the ball on the defense line.
//     ELSE
//        position unassigned field-player as fast as possible on the defense line.
//        -     determine cross-section of the perpendicular (dutch: loodlijn) of the defense line with the defense line
//          (end point <options.priority_block_max_distance> meter from opponent closest to the ball (= A) and middle of the goal-line (B).
//             <options.priority_block_max_distance> meter is chosen to prevent fast path from other side of the ball)
//      -   if cross-section is between A and B then use distance from player to defense line as cost.
//        -   if cross-section is NOT between A and B then use 4 times distance from player to defense line as cost (and select A as cross-section).
//        -  cross-section point of the field player with lowest cost becomes the the intercepter player
//

    MRA::Geometry::Point ballPos(r_teamplannerData.ball.position.x, r_teamplannerData.ball.position.y);
    MRA::Geometry::Point rolePosition = ballPos; // set default position to the ball position

    // CHECK if priority block must be performed:
    if (r_teamplannerData.ball_status == ball_status_e::OWNED_BY_OPPONENT) {
        // determine the smallest distance from any opponent to the ball
        MRA::Geometry::Point opponent_with_ball_pos = ballPos;
        double smallestDistOpponentToBall = r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance ();

        for (auto idx = 0u; idx < r_teamplannerData.opponents.size (); idx++) {
            auto opponent_pos = r_teamplannerData.opponents[idx].position;
            auto dist = opponent_pos.distanceTo (ballPos);
            if (dist < smallestDistOpponentToBall) {
                smallestDistOpponentToBall = dist;
                opponent_with_ball_pos = opponent_pos;
            }
        }

        MRA::Geometry::Point goalPos = MRA::Geometry::Point (0.0, -r_teamplannerData.fieldConfig.getMaxFieldY ());
        double smallestDistToBallOnTheDefenseLine = r_teamplannerData.fieldConfig.getMaxPossibleFieldDistance ();
        MRA::Geometry::Point posOfInterest = opponent_with_ball_pos;  // center of opponent with ball.(for now)
        double distPosOfInterest2Goal = goalPos.distanceTo (posOfInterest);
        double maxDistanceToPOI = r_teamplannerData.parameters.priority_block_max_distance; // safe distance to ball (used to prevent players move around ball).
        double minDistanceToPOI = r_teamplannerData.parameters.priority_block_min_distance; // safe distance to ball (used to prevent players move around ball).

        int nr_field_players_between_posOfInterest_and_own_goal = 0;

        // count number of field-players on the line between ball and own goal.
        for (auto it = r_teamplannerData.team.begin (); it != r_teamplannerData.team.end (); ++it) {
            if (it->player_type == player_type_e::FIELD_PLAYER and !(it->assigned)) {
                // only unassigned field-players

                MRA::Geometry::Point playerPos(it->position.x, it->position.y);
                double distToDefenseLine = getDistanceFromPointToLineSegment(posOfInterest.x, posOfInterest.y,
                                                                      goalPos.x, goalPos.y, playerPos.x, playerPos.y);
                double distToGoal = playerPos.distanceTo (goalPos);
                double distToPosOfInterest = playerPos.distanceTo (posOfInterest);

                if ((distToGoal < distPosOfInterest2Goal)
                     and (distToDefenseLine < r_teamplannerData.parameters.priority_block_max_distance_to_defense_line)
                     and (distToPosOfInterest < maxDistanceToPOI)) {
                    // field player is on line between goal and ball AND close enough to the ball (<safeDistanceToBall):
                    // -> closer than the ball to the goal and small distance to defense line
                    double distToPosOfInterest = playerPos.distanceTo (posOfInterest);
                    smallestDistToBallOnTheDefenseLine = min (distToPosOfInterest, smallestDistToBallOnTheDefenseLine);
                    nr_field_players_between_posOfInterest_and_own_goal++;
                }
            }
        }
        double goalToPosOfInterestAngle = posOfInterest.angle (goalPos);
        if (nr_field_players_between_posOfInterest_and_own_goal >= 1) {
            // Select position on the defense line, close to the ball (intended: drive towards the ball if on defense line)
            double targetDistToBall = min(smallestDistToBallOnTheDefenseLine, r_teamplannerData.parameters.priority_block_max_distance); // take smallest: to prevent moving backwards
            targetDistToBall = max(targetDistToBall, r_teamplannerData.parameters.priority_block_min_distance); // take biggest: to prevent moving too much forward
            double priorityBlockX = posOfInterest.x - (cos (goalToPosOfInterestAngle) * targetDistToBall);
            double priorityBlockY = posOfInterest.y - (sin (goalToPosOfInterestAngle) * targetDistToBall);
            rolePosition = MRA::Geometry::Point (priorityBlockX, priorityBlockY); // set role position on the priority block location
        }
        else {
            // No field-player on the defense line
            // calculate the safe priority block position (last position on line between ball and own goal)
            // calculate distance from maxPriorityBlockPoint to goal
            double   maxPriorityBlockX     = posOfInterest.x - (cos (goalToPosOfInterestAngle) * maxDistanceToPOI);
            double   maxPriorityBlockY     = posOfInterest.y - (sin (goalToPosOfInterestAngle) * maxDistanceToPOI);
            MRA::Geometry::Point maxPriorityBlockPoint = MRA::Geometry::Point (maxPriorityBlockX, maxPriorityBlockY);
            double   minPriorityBlockX     = posOfInterest.x - (cos (goalToPosOfInterestAngle) * minDistanceToPOI);
            double   minPriorityBlockY     = posOfInterest.y - (sin (goalToPosOfInterestAngle) * minDistanceToPOI);
            MRA::Geometry::Point minPriorityBlockPoint = MRA::Geometry::Point (minPriorityBlockX, minPriorityBlockY);

            // find team-mate which is closest to defense line then the priority block position
            // (from ball to safe priority block position - (between ball and own goal))
            MRA::Geometry::Point bestPos = maxPriorityBlockPoint;
            for (auto it = r_teamplannerData.team.begin (); it != r_teamplannerData.team.end (); ++it) {
                if (it->player_type == player_type_e::FIELD_PLAYER and !(it->assigned)) {
                    // only unassigned field-players
                    MRA::Geometry::Point teamMatePos(it->position.x, it->position.y);
                    // calculate second location of on the perpendicular to the defense line
                    double tx1 = teamMatePos.x + (cos (goalToPosOfInterestAngle + M_PI * 0.5) * 1.0);
                    double ty1 = teamMatePos.y + (sin (goalToPosOfInterestAngle + M_PI * 0.5) * 1.0);

                    // calculate intersection of the perpendicular of the defense line with the defense line
                    double intersectX = 0;
                    double intersectY = 0;
                    bool linesIntersect = getIntersectionOfTwoLines (intersectX, intersectY, tx1, ty1, teamMatePos.x,
                                                                     teamMatePos.y, maxPriorityBlockX, maxPriorityBlockY, goalPos.x, goalPos.y);
                    MRA::Geometry::Point intersectionPoint = MRA::Geometry::Point (intersectX, intersectY);
                    if (linesIntersect
                        and intersectionPoint.distanceTo (posOfInterest) < bestPos.distanceTo (posOfInterest)
                        and intersectionPoint.distanceTo (goalPos) < distPosOfInterest2Goal) {
                        // intersection point is closer to ball then best position found so far. And intersectionPoint is between ball and own goal
                        if (intersectionPoint.distanceTo (posOfInterest) >= minDistanceToPOI
                            and intersectionPoint.distanceTo (posOfInterest) <= maxDistanceToPOI) {
                            // intersection point is between min and max allowed dist
                            bestPos = intersectionPoint;
                        }
                        else {
                            if (intersectionPoint.distanceTo (posOfInterest) < minDistanceToPOI) {
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
            planner_target = planner_target_e::PRIORITY_BLOCK; // indicate to followpath that path purpose is priority block

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
    vector<double> oppenent_to_ball_dist = vector<double>();
    vector<double> oppenent_to_goal_dist = vector<double>();
    MRA::Geometry::Point ballPos = r_teamplannerData.ball.position;
    MRA::Geometry::Point goalPos = MRA::Geometry::Point(0, -r_teamplannerData.fieldConfig.getMaxFieldY());
    // get distances for each opponent to ball and our goal.
    for (unsigned idx = 0; idx < r_teamplannerData.opponents.size(); idx++) {
        MRA::Geometry::Point opponent_position = r_teamplannerData.opponents[idx].position;
        oppenent_to_ball_dist.push_back(ballPos.distanceTo(opponent_position));
        oppenent_to_goal_dist.push_back(goalPos.distanceTo(opponent_position));
    }

    // assume opponent closest to our goal is most dangerous
    MRA::Geometry::Point opponentGoal = MRA::Geometry::Point(0, r_teamplannerData.fieldConfig.getMaxFieldY());
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
    vector<double> oppenent_to_ball_dist = vector<double>();
    vector<double> oppenent_to_goal_dist = vector<double>();
    MRA::Geometry::Point goalPos = MRA::Geometry::Point(0, -r_teamplannerData.fieldConfig.getMaxFieldY());
    // get distances for each opponent to ball and our goal.
    for (unsigned idx = 0; idx < r_teamplannerData.opponents.size(); idx++) {
        MRA::Geometry::Point opponent_position = r_teamplannerData.opponents[idx].position;
        oppenent_to_ball_dist.push_back(targetPos.distanceTo(opponent_position));
        oppenent_to_goal_dist.push_back(goalPos.distanceTo(opponent_position));
    }

    MRA::Geometry::Point opponentGoal = MRA::Geometry::Point(0, r_teamplannerData.fieldConfig.getMaxFieldY());
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
MRA::Geometry::Point RolePosition::calculateManToManDefensePosition(
        defend_info_t& rDefend_info, dynamic_role_e dynamic_role,
        TeamPlannerData& r_teamplannerData, int& r_gridFileNumber, bool setPlayActive) {
    // man coverage for all opponents
    MRA::Geometry::Point rolePosition = MRA::Geometry::Point();

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

MRA::Geometry::Point RolePosition::determineSetplayRolePosition_2024(int assignment_nr, defend_info_t& rDefend_info, planner_target_e& planner_target, int& r_gridFileNumber,
                                                         dynamic_role_e dynamic_role,  TeamPlannerData& r_teamplannerData,
                                                         bool playerPassedBall, bool& r_role_position_is_end_position_of_pass) {
    MRA::Geometry::Point rolePosition = MRA::Geometry::Point(0,0);
    rDefend_info.defending_id = -1; // no item to defend
    rDefend_info.valid = false;
    r_role_position_is_end_position_of_pass = false;
    MRA::Geometry::Point ballPos = r_teamplannerData.ball.position;
    static MRA::Geometry::Point optimal_role_pos;

    // calculate number of field players
    auto nr_field_players = r_teamplannerData.team.size();
    for (unsigned idx = 0; idx < r_teamplannerData.team.size(); idx++) {
        if (r_teamplannerData.team[idx].player_type == GOALIE or r_teamplannerData.team[idx].result.dynamic_role == dynamic_role_e::dr_GOALKEEPER) {
            nr_field_players--;
        }
    }

    switch (dynamic_role) {
    case dr_SETPLAY_KICKER: {
        if (ballPos.y < 0 and nr_field_players  > 2) {
            // ball at own side and more than 2 field players, aim kicker to center of opponent half
            auto aimPoint = MRA::Geometry::Point(0.0, r_teamplannerData.fieldConfig.getMaxFieldY() * 0.5);
            rolePosition =  calculateSetPlayerKickerPosition(aimPoint, r_teamplannerData);
        }
        else {

            // calculate receiver position for the assigned receiver position
            MRA::Geometry::Point receiverPosition = calculateSetPlayReceiverPosition(r_teamplannerData);
            rolePosition =  calculateSetPlayerKickerPosition(receiverPosition, r_teamplannerData);
        }
    }
    break;
    case dr_SETPLAY_RECEIVER: {
        // look up position if possible
        rolePosition = calculateSetPlayReceiverPosition(r_teamplannerData);
    }
    break;
    case dr_PENALTY_DEFENDER:
    case dr_DEFENDER:
    case dr_ATTACKSUPPORTER:
    case dr_PENALTY_KICKER:
    case dr_INTERCEPTOR:
    case dr_BALLPLAYER:
    case dr_SWEEPER:
    {
        // all situations: normal set-play kicker and receiver.
        // --> robot-planner should choose long pass if ball is at own half, to set-play receiver at opponent half
        // 3rd player
        //    - ball at own half: player at other side of the field than side where ball is (keep field play wide if only three players are on the field)
        //    - ball at opponent half: near goal ready to act on rebound. slightly at the other side than ball
        // 4th player
        //    - ball at own half: at same side as ball
        //    - ball at opponent half: at side of ball at top side of the half if ball is in bottom side or visa versa
        //
        //
        // when ball is at own half: non-setplayers are preferred to be  at opponent half (if not to far - maximum distance to ball)
        //

        const double MIN_DIST_BALL_ATTACKER = 6.0;  // minimum distance for player to the ball : otherwise to close to the ball
        const double BEST_DIST_BALL_ATTACKER = 12.0; // preferred (best) distance for player to the ball
        const double MAX_DIST_BALL_ATTACKER = 16.0; // maximum distance for player to the ball : otherwise to far from the ball to handle pas
        const double MIN_Y_WHEN_BALL_ON_OWN_HALF = 1.0;  // minimum y for player if ball is at own half (y > 0 is at opponent half)

        // maximum y for player if ball is at own half (y > 0 is at opponent half)
        const double MAX_Y_WHEN_BALL_ON_OWN_HALF = r_teamplannerData.fieldConfig.getTopPenaltyAreaY() - 1.0;


        double max_field_x = r_teamplannerData.fieldConfig.getMaxFieldX();
        MRA::Geometry::Point optimal_role_pos;
        if (assignment_nr == 3) {
            if (ballPos.y <= 0) {
                // calculate optimal position
                auto best_x = max_field_x - 1.0; // 1 meter from side
                optimal_role_pos.x = (ballPos.x < 0) ? best_x : -best_x;
                auto dist_x = fabs(ballPos.x - optimal_role_pos.x );
                auto best_y = ballPos.y + legRightTriangle(BEST_DIST_BALL_ATTACKER, dist_x);
                if (best_y >= MIN_Y_WHEN_BALL_ON_OWN_HALF) {
                    optimal_role_pos.y = best_y;
                }
                else {
                    optimal_role_pos.y = MIN_Y_WHEN_BALL_ON_OWN_HALF;
                    if (ballPos.distanceTo(optimal_role_pos) > MAX_DIST_BALL_ATTACKER) {
                        // limit y: dist ball-optimal_role_pos = MAX_DIST_BALL_ATTACKER
                        optimal_role_pos.y = ballPos.y + legRightTriangle(MAX_DIST_BALL_ATTACKER, dist_x);
                    }
                }

                if (ballPos.distanceTo(optimal_role_pos) < MIN_DIST_BALL_ATTACKER) {
                    // distance is too short; adjust y position
                    optimal_role_pos.y += (MIN_DIST_BALL_ATTACKER - ballPos.distanceTo(optimal_role_pos));
                }

                // clip y position if too close to opponent goal
                optimal_role_pos.y = std::min(optimal_role_pos.y, MAX_Y_WHEN_BALL_ON_OWN_HALF);

                rolePosition = TeamPlanner_Grid::findSetPlayPosition(dynamic_role, r_teamplannerData, optimal_role_pos, r_gridFileNumber, false, true, true);
            }
            else {
                // ball at opponent half: setplay receiver is at best possible shooting position
                // Player must be ready to pickup rebounced ball: locate near opponent goal
                optimal_role_pos.x = 0.0;
                auto fc = r_teamplannerData.fieldConfig;
                optimal_role_pos.y = fc.getMaxFieldY() - fc.getPenaltySpotToBackline();

                // calculate position with as target the optimal position : stronger preference for Y location than X, ready for rebound (no ready for pass needed)
                rolePosition = TeamPlanner_Grid::findSetPlayPosition(dynamic_role, r_teamplannerData, optimal_role_pos, r_gridFileNumber, false, true, false);
            }
        }
        else if (assignment_nr == 4) {
            // calculate optimal position
            if (ballPos.y <= 0) {
                auto best_x = max_field_x - 1.0; // 1 meter from side
                optimal_role_pos.x = (ballPos.x < 0) ? -best_x : best_x;
                auto dist_x = fabs(ballPos.x- optimal_role_pos.x);
                auto best_y = ballPos.y + legRightTriangle(BEST_DIST_BALL_ATTACKER, dist_x);
                if (best_y >= MIN_Y_WHEN_BALL_ON_OWN_HALF) {
                    optimal_role_pos.y = best_y;
                }
                else {
                    optimal_role_pos.y = MIN_Y_WHEN_BALL_ON_OWN_HALF;
                    if (ballPos.distanceTo(optimal_role_pos) > MAX_DIST_BALL_ATTACKER) {
                        // limit y: dist ball-pos = MAX_DIST_BALL_ATTACKER
                        optimal_role_pos.y = ballPos.y + legRightTriangle(MAX_DIST_BALL_ATTACKER, dist_x);
                    }
                }

                if (ballPos.distanceTo(optimal_role_pos) < MIN_DIST_BALL_ATTACKER) {
                    // distance is too short; adjust y position
                    optimal_role_pos.y += (MIN_DIST_BALL_ATTACKER - ballPos.distanceTo(optimal_role_pos));
                }

                // clip y position if too close to opponent goal
                optimal_role_pos.y = std::min(optimal_role_pos.y, MAX_Y_WHEN_BALL_ON_OWN_HALF);

                rolePosition = TeamPlanner_Grid::findSetPlayPosition(dynamic_role, r_teamplannerData, optimal_role_pos, r_gridFileNumber, false, true, true);
            }
            else {
                auto best_x = max_field_x - 2.5; // X meter from side
                MRA::Geometry::Point optimal_role_pos;
                optimal_role_pos.x = (ballPos.x < 0) ? best_x : -best_x;
                optimal_role_pos.y = r_teamplannerData.fieldConfig.getTopPenaltyAreaY() - 2.0;  // 2 meter from top of opponent penalty area
                rolePosition = TeamPlanner_Grid::findSetPlayPosition(dynamic_role, r_teamplannerData, optimal_role_pos, r_gridFileNumber, true, false, true);
            }
        }
        else
        {
            // use original role position
            rolePosition = determineDynamicRolePosition(rDefend_info, planner_target, r_gridFileNumber, dynamic_role,
                                                        r_teamplannerData, playerPassedBall, r_role_position_is_end_position_of_pass);
        }

    }
    break;
    // not supported roles: use default position
    case dr_GOALKEEPER: // empty
    case dr_SEARCH_FOR_BALL: // empty: only used for diagnostics
    case dr_BEGIN_POSITION: // empty: only used for diagnostics
    case dr_PARKING: // empty: only used for diagnostics
    case dr_IS_ALIVE:
    case dr_LOB_CALIBRATION:
    case dr_NONE:
    {
        // use original role position
        rolePosition = determineDynamicRolePosition(rDefend_info, planner_target, r_gridFileNumber, dynamic_role,
                r_teamplannerData, playerPassedBall, r_role_position_is_end_position_of_pass);
    }
    break;
    }
    return rolePosition;
}
