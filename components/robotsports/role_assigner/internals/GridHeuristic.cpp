/*
 * GridHeuristic.cpp
 *
 *  Created on: Sep 12, 2016
 *      Author: jurge
 */

#include "GridHeuristic.h"
#include <cmath>
#include <limits>
#include <iostream>
#include "planner_common.h"
#include "MathUtils.h"
using namespace trs;

double grid_eps = 1e-3; // 1 mm

//-------------------------------------------------------------------------------------------------------
GridHeuristic::GridHeuristic(const char * id_str, double w, PlannerGridInfoData& pgid) :
																id(id_str), weight(w)
{
	pgid.name.push_back(id_str);
	pgid.weight.push_back(w);
}

GridHeuristic::~GridHeuristic() {

}

//-------------------------------------------------------------------------------------------------------
InSquareHeuristic::InSquareHeuristic(const char * id, double weight, PlannerGridInfoData& pgid,
		double x_left, double y_top, double x_right, double y_bottom, bool invert) :
								GridHeuristic(id, weight, pgid),
								m_x_left(x_left),
								m_y_top(y_top),
								m_x_right(x_right),
								m_y_bottom(y_bottom),
								m_invert(invert)
{
	// empty
}

double InSquareHeuristic::getValue(double x, double y) {
	double value = 0.0;
	if ((x >= m_x_left) && (x <= m_x_right) && (y <= m_y_top) && (y >= m_y_bottom)) {
		value = 1.0; // don't allow locations in the defined square
	}
	if (m_invert) {
		value = fabs(value - 1.0);
	}
	return value;
}

//-------------------------------------------------------------------------------------------------------
InCircleHeuristic::InCircleHeuristic(const char * id, double weight, PlannerGridInfoData& pgid,
		double cx, double cy, double radius, bool invert) :
								GridHeuristic(id, weight, pgid),
								m_cx(cx),
								m_cy(cy),
								m_radius(radius),
								m_invert(invert)
{
	// empty
}

double InCircleHeuristic::getValue(double x, double y) {
	double value = 0.0;
	if (((x-m_cx)*(x-m_cx) + (y-m_cy)*(y-m_cy)) < (m_radius*m_radius)) {
		value = 1.0;
	}
	if (m_invert) {
		value = fabs(value - 1.0);
	}
	return value;
}

//-------------------------------------------------------------------------------------------------------
BallSetplayAgainstHeuristic::BallSetplayAgainstHeuristic(const char * id, double weight, PlannerGridInfoData& pgid,
		double cx, double cy, double radius, const FieldConfig& fieldConfig) :
								GridHeuristic(id, weight, pgid),
								m_cx(cx),
								m_cy(cy),
								m_radius(radius),
								m_fieldConfig(fieldConfig)
{
	// empty
}

double BallSetplayAgainstHeuristic::getValue(double x, double y) {
	double value = 0.0;
	if (m_fieldConfig.isInOwnPenaltyArea(x, y)) {
			value = 0.0;
	}
	else if (((x-m_cx)*(x-m_cx) + (y-m_cy)*(y-m_cy)) < (m_radius*m_radius)) {
		value = 1.0;
	}
	return value;
}


//-------------------------------------------------------------------------------------------------------
DistanceToLineHeuristic::DistanceToLineHeuristic(const char * id, double weight, PlannerGridInfoData& pgid,
		double x1, double y1, double x2, double y2, double scaling, bool invert) :
								GridHeuristic(id, weight, pgid),
								m_x1(x1),
								m_y1(y1),
								m_x2(x2),
								m_y2(y2),
								m_scaling(scaling),
								m_invert(invert)
{
	// empty
}

double DistanceToLineHeuristic::getValue(double x, double y) {
	// Penalty is distance from x,y to the line between x1,y1 and x2,y2
	double value = 0.0;
	if (fabs(m_scaling) > 1e-16) {
		value = getDistanceFromPointToLine(m_x1, m_y1, m_x2, m_y2, x, y) / m_scaling;
	}
	if (m_invert) {
		value = fabs(value - 1.0);
	}
	return value;
}

//-------------------------------------------------------------------------------------------------------
InTriangleHeuristic::InTriangleHeuristic(const char * id, double weight, PlannerGridInfoData& pgid,
		double x1, double y1, double x2, double y2, double x3, double y3, bool invert) :
										GridHeuristic(id, weight, pgid),
										m_x1(x1),
										m_y1(y1),
										m_x2(x2),
										m_y2(y2),
										m_x3(x3),
										m_y3(y3),
										m_invert(invert)
{

}

double InTriangleHeuristic::getValue(double x, double y)
// algorithm is described on: https://stackoverflow.com/questions/13300904/determine-whether-point-lies-inside-triangle
// This is done by calculating the barycentric coordinates of the fourth point given the three points of your triangle.
{
	double value = 0.0;
	if (inTriangle(m_x1, m_y1, m_x2, m_y2, m_x3, m_y3, x, y)) {
		value = 1.0;
	}
	if (m_invert) {
		value = fabs(value - 1.0);
	}
	return value;

}

//-------------------------------------------------------------------------------------------------------
// Calculate penalty for position in opponent penaltyArea
InOppenentPenaltyAreaHeuristic::InOppenentPenaltyAreaHeuristic(const char * id, double weight, PlannerGridInfoData& pgid,
		const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig) :
						InSquareHeuristic(id, weight, pgid,
								-(plannerOptions.grid_opponent_goal_clearance_x*0.5), fieldConfig.getMaxFieldY(),
								+(plannerOptions.grid_opponent_goal_clearance_x*0.5), fieldConfig.getMaxFieldY() - plannerOptions.grid_opponent_goal_clearance_y)
{
	// empty
}

//-------------------------------------------------------------------------------------------------------
// Calculate penalty for position in own penaltyArea
InOwnPenaltyAreaHeuristic::InOwnPenaltyAreaHeuristic(const char * id, double weight, PlannerGridInfoData& pgid,
		const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig) :
								InSquareHeuristic(id, weight, pgid,
										-(plannerOptions.grid_opponent_goal_clearance_x*0.5), (-fieldConfig.getMaxFieldY()) + plannerOptions.grid_opponent_goal_clearance_y,
										+(plannerOptions.grid_opponent_goal_clearance_x*0.5), -fieldConfig.getMaxFieldY())
{
	// empty
}

//-------------------------------------------------------------------------------------------------------
// Calculate penalty when already assigned field player has end position in own penaltyArea
AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic::AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic(const char * id, double weight,
		PlannerGridInfoData& pgid, 	const std::vector<TeamPlannerRobot>& Team, const FieldConfig& fieldConfig) :
																		GridHeuristic(id, weight, pgid),
																		m_alreadyPlayerAssignedToOwnPenaltyArea(false),
																		m_fieldConfig(fieldConfig) {
	for (unsigned idx = 0; idx < Team.size() && m_alreadyPlayerAssignedToOwnPenaltyArea == false; idx++) {
		if (Team[idx].assigned  && Team[idx].player_type == player_type_e::FIELD_PLAYER) {
			// field player is assigned and has a path
			double end_x = Team[idx].result.target.m_x;
			double end_y = Team[idx].result.target.m_y;
			if  (m_fieldConfig.isInOwnPenaltyArea(end_x, end_y)) {
				m_alreadyPlayerAssignedToOwnPenaltyArea = true;
			}
		}
	}
}

double AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic::getValue(double x, double y) {
	double value = 0.0;
	if (m_alreadyPlayerAssignedToOwnPenaltyArea && m_fieldConfig.isInOwnPenaltyArea(x,y)) {
		value = 1.0; // don't allow locations in own penalty area
	}
	return value;
}

//-------------------------------------------------------------------------------------------------------
// Calculate penalty when already assigned field player has end position in opponent penaltyArea
AlreadyPlayerAssignedToOpponentPenaltyAreaHeuristic::AlreadyPlayerAssignedToOpponentPenaltyAreaHeuristic(const char * id, double weight,
		PlannerGridInfoData& pgid, 	const std::vector<TeamPlannerRobot>& Team, const FieldConfig& fieldConfig) :
																		GridHeuristic(id, weight, pgid),
																		m_fieldConfig(fieldConfig) {
	for (unsigned idx = 0; idx < Team.size() && m_alreadyPlayerAssignedToOpponentPenaltyArea == false; idx++) {
		if (Team[idx].assigned  && Team[idx].player_type == player_type_e::FIELD_PLAYER) {
			// field player is assigned and has a path
			double end_x = Team[idx].result.target.m_x;
			double end_y = Team[idx].result.target.m_y;
			if  (m_fieldConfig.isInOpponentPenaltyArea(end_x, end_y)) {
				m_alreadyPlayerAssignedToOpponentPenaltyArea = true;
			}
		}
	}
}

double AlreadyPlayerAssignedToOpponentPenaltyAreaHeuristic::getValue(double x, double y) {
	double value = 0.0;
	if (m_alreadyPlayerAssignedToOpponentPenaltyArea && m_fieldConfig.isInOpponentPenaltyArea(x,y)) {
		value = 1.0; // don't allow locations in own penalty area
	}
	return value;
}

//-------------------------------------------------------------------------------------------------------
// Calculate penalty for position in own goalArea
InOwnGoalAreaHeuristic::InOwnGoalAreaHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
		const PlannerOptions& plannerOptions, const FieldConfig& fieldConfig) :
												InSquareHeuristic(id, weight, pgid,
														-(plannerOptions.grid_own_goal_clearance_x*0.5), (-fieldConfig.getMaxFieldY()) + plannerOptions.grid_own_goal_clearance_y,
														+(plannerOptions.grid_own_goal_clearance_x*0.5), -fieldConfig.getMaxFieldY()) {

}


//-------------------------------------------------------------------------------------------------------
// Calculate penalty for position close to ball
DistanceToBallHeuristic::DistanceToBallHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
		double ball_x, double ball_y, double radius) : InCircleHeuristic(id, weight, pgid, ball_x, ball_y,radius, true /*invert */)
{

}

//-------------------------------------------------------------------------------------------------------
// Calculate penalty for position close to ball
InfluenceBallHeuristic::InfluenceBallHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
		double ball_x, double ball_y, double radius)  : InCircleHeuristic(id, weight, pgid, ball_x, ball_y,radius)
{

}

// ----------------------------------------------------------------------------------------
// Calculate influence  (penalty) for the position with respect to corner of the field
InfluenceCornerHeuristic::InfluenceCornerHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
		const FieldConfig& fieldConfig) :
																GridHeuristic(id, weight, pgid),
																m_fieldConfig(fieldConfig)
{
	// emptys
}

double InfluenceCornerHeuristic::getValue(double x, double y) {
	// add influence corner : 0.08 linear decreasing till 4 meters from corner (inside the field)
	// out side the field: 1.0
	double value = 1.0; // outside field
	if (m_fieldConfig.isInField(x, y, -0.01)) {
		// x, y are in the field with 1 cm margin

		double distXtoCorner = m_fieldConfig.getMaxFieldX();
		double distYtoCorner = m_fieldConfig.getMaxFieldY();
		double distXFactor = (x < 0) ? -1.0 : 1.0;
		double distYFactor = (y < 0) ? -1.0 : 1.0;
		Vector2D closestCorner = Vector2D(distXtoCorner*distXFactor, distYtoCorner*distYFactor);
		double dist2corner = closestCorner.distanceTo(Vector2D(x,y));
		if (dist2corner > 4.0) {
			value = 0.0;
		}
		else {
			// close to corner
			value = 0.08*((4.0-dist2corner)/ 4.0); // 0.8 if x,y is in closest corner, else linear decreasing to 0
		}

	}
	return value;
}


// ----------------------------------------------------------------------------------------
CollideTeamMateHeuristic::CollideTeamMateHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
		const std::vector<TeamPlannerRobot>& Team, double radius) :
																		GridHeuristic(id, weight, pgid),
																		m_Team(Team),
																		m_radius(radius) {

}

double CollideTeamMateHeuristic::getValue(double x, double y) {
	// add influence: team mate also supporter (half ball influence?)
	// TODO: rewrite this function to not have ax_sqr and ball_c, but radius and penalty.
	double value = 0.0;
	for (unsigned idx = 0; idx < m_Team.size(); idx++) {
		if (m_Team[idx].assigned) {
			// player is assigned and has a path
			if (m_Team[idx].result.target.distanceTo(Vector2D(x,y)) < m_radius) {
				value = 1.0;
			}
		}
	}
	return value;
}

// ----------------------------------------------------------------------------------------
// Calculate penalty for the position with respect to the opponents
InfluenceOpponentsHeuristic::InfluenceOpponentsHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
		const std::vector<TeamPlannerOpponent>& Opponents, double radius) :
												GridHeuristic(id, weight, pgid),
												m_Opponents(Opponents),
												m_radius(radius) {
	// empty
}

double InfluenceOpponentsHeuristic::getValue(double x, double y) {
	double value = 0.0;
	// add influence other players. Opponents twice team meat. 1/10 support influence (20x² ?)
	for (unsigned bar_idx = 0; bar_idx < m_Opponents.size(); bar_idx++) {
		Vector2D bar_pos = m_Opponents[bar_idx].position.getPosition().getVector2D();
		if (bar_pos.distanceTo(Vector2D(x,y)) < m_radius) {
			value = 1.0;
		}
	}
	return value;
}
// ----------------------------------------------------------------------------------------
// Calculate penalty for position outside play field
OutsidePlayFieldHeuristic::OutsidePlayFieldHeuristic(const char *id, double weight, PlannerGridInfoData& pgid, const FieldConfig& fieldConfig, double extra_distance_to_sideline) :
						InSquareHeuristic(id, weight, pgid,
								-(fieldConfig.getMaxFieldX() - fieldConfig.getRobotRadius()- extra_distance_to_sideline),
								 (fieldConfig.getMaxFieldY() - fieldConfig.getRobotRadius()- extra_distance_to_sideline),
								 (fieldConfig.getMaxFieldX() - fieldConfig.getRobotRadius()- extra_distance_to_sideline),
								-(fieldConfig.getMaxFieldY() - fieldConfig.getRobotRadius()- extra_distance_to_sideline), true /* inverted */) {
	// empty
}


// ----------------------------------------------------------------------------------------
OnLineBetweenPointsHeuristic::OnLineBetweenPointsHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
		double x1, double y1, double x2, double y2, double maxPossibleFieldDistance) :
											GridHeuristic(id, weight, pgid),
											m_x1(x1),
											m_y1(y1),
											m_x2(x2),
											m_y2(y2),
											m_dScaling(5.0 * maxPossibleFieldDistance){
}

double OnLineBetweenPointsHeuristic::getValue(double x, double y){
	// Penalty is distance from x,y to the line between x1,y1 and x2,y2 (with rotation)
	double value = 0.0;
	if (fabs(m_dScaling) > 1e-16) {
		double distToLinePiece = 0;
		double px = 0;
		double py = 0;
		intersectPerpendicular(px, py, m_x1, m_y1, m_x2, m_y2, x, y);
		Vector2D Pintersect(px,py);
		Vector2D P1(m_x1, m_y1);
		Vector2D P2(m_x2, m_y2);
		double distPintoP1 = Pintersect.distanceTo(P1);
		double distPintoP2 = Pintersect.distanceTo(P2);
		double distP1toP2 = P1.distanceTo(P2);
		if ((distPintoP1 >  distP1toP2) || (distPintoP2 >  distP1toP2) ) {
			// P is not between P1 and P2. use shortest distance to line piece
			Vector2D p(x,y);
			distToLinePiece = min(p.distanceTo(P1), p.distanceTo(P2));
		}
		else {
			distToLinePiece = getDistanceFromPointToLine(m_x1, m_y1, m_x2, m_y2, x, y);
		}
		if (distToLinePiece > 0) {
			value =  (5.0 * distToLinePiece) / m_dScaling;
		}
		else {
			value = 1.0;
		}
	}
	return value;
}

// ----------------------------------------------------------------------------------------
DesiredYSweeper::DesiredYSweeper(const char *id, double weight, PlannerGridInfoData& pgid,
		double desired_y, const FieldConfig& fieldConfig) :
						DistanceToLineHeuristic(id, weight, pgid,
								-fieldConfig.getMaxFullFieldX(), desired_y, fieldConfig.getMaxFullFieldX(), desired_y, fieldConfig.getMaxPossibleFieldDistance())
{

}

// ----------------------------------------------------------------------------------------
DesiredXSweeper::DesiredXSweeper(const char *id, double weight, PlannerGridInfoData& pgid,
		double desired_x, const FieldConfig& fieldConfig) :
								DistanceToLineHeuristic(id, weight, pgid,
										desired_x, -fieldConfig.getMaxFullFieldY(), desired_x, fieldConfig.getMaxFullFieldY(), fieldConfig.getMaxPossibleFieldDistance())
{

}

// ----------------------------------------------------------------------------------------
class DesiredXSweeper : public GridHeuristic
{
public:
	DesiredXSweeper(const char *id, double weight, PlannerGridInfoData& pgid,
			double desired_x, const FieldConfig& fieldConfig);
	virtual ~DesiredXSweeper() {};
	double getValue(double x, double y);
private:
	const double m_desired_x;
	const double m_dHalfLength;
	const double m_dScaling;
};

// ----------------------------------------------------------------------------------------
DistanceToHeuristic::DistanceToHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
		const Vector2D& pos, double dScaling):
												GridHeuristic(id, weight, pgid),
												m_pos(pos),
												m_dScaling(dScaling)
{

}

double DistanceToHeuristic::getValue(double x, double y) {
	double value = 0.0;
	if (fabs(m_dScaling) > 1e-16) {
		value =  m_pos.distanceTo(Vector2D(x, y)) / m_dScaling;  // distance closer to position is better (lower)
	}
	return value;
}


// ----------------------------------------------------------------------------------------
DistanceToPointHeuristic::DistanceToPointHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
		const Vector2D& pos, double dScaling, double maxRange, bool inverted) :
												GridHeuristic(id, weight, pgid),
												m_pos(pos),
												m_dScaling(dScaling),
												m_dMaxRange(maxRange),
												m_bInverted(inverted)
{

}

double DistanceToPointHeuristic::getValue(double x, double y) {
	double value = 0.0;
	if (fabs(m_dScaling) > 1e-16) {
		value =  m_pos.distanceTo(Vector2D(x, y));
		if (value > m_dMaxRange) {
			value = m_dMaxRange;
		}
		value = value / m_dScaling;  // distance closer to position is better (lower)
	}
	if (m_bInverted)
	{
		value = fabs(value - 1.0);
	}
	return value;
}

// ----------------------------------------------------------------------------------------
DistanceToEnemyGoalHeuristic::DistanceToEnemyGoalHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
		double penAreaY, double dScaling) :
										GridHeuristic(id, weight, pgid),
										m_penaltyAreaY(penAreaY),
										m_dScaling(dScaling)
{

}
double DistanceToEnemyGoalHeuristic::getValue(double x, double y) {
	// add small value for y distance to enemy goal:
	double distToMiddlePenaltyArea = hypot(x, (m_penaltyAreaY-y));
	double value = 0.0;
	if (fabs(m_dScaling) > 1e-16) {
		value = (distToMiddlePenaltyArea*0.05) / m_dScaling;
	}
	return value;
}


// ----------------------------------------------------------------------------------------
NotOnLineBetweenBallAndOpponentGoalHeuristic::NotOnLineBetweenBallAndOpponentGoalHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
		double ball_x, double ball_y, double left_pole_x, double left_pole_y, double right_pole_x, double right_pole_y):
							InTriangleHeuristic(id, weight, pgid,
									ball_x, ball_y, right_pole_x, right_pole_y,  left_pole_x, left_pole_y, false /*inverted: 1.0 if outside triangle */) {
	// empty
}

// ----------------------------------------------------------------------------------------
InterceptionThreatHeuristic::InterceptionThreatHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
		const MovingObject& ball,
		const std::vector<TeamPlannerRobot>& Team,
		const std::vector<TeamPlannerOpponent>& Opponents,
		double interceptionChanceStartDistance,
		double interceptionChanceIncreasePerMeter,
		double interceptionChancePenaltyFactor) :
											GridHeuristic(id, weight, pgid),
											m_ball(ball),
											m_Team(Team),
											m_Opponents(vector<MovingObject>()),
											m_interceptionChanceStartDistance(interceptionChanceStartDistance),
											m_interceptionChanceIncreasePerMeter(interceptionChanceIncreasePerMeter),
											m_interceptionChancePenaltyFactor(interceptionChancePenaltyFactor)
{
	for (unsigned idx = 0; idx < Opponents.size(); idx++) {
		m_Opponents.push_back(Opponents[idx].position);
	}
	// avoid being intercepted by own team mate (attack supporter)
	for (unsigned idx = 0; idx < Team.size(); idx++) {
		if (Team[idx].assigned && Team[idx].dynamic_role == dr_ATTACKSUPPORTER) {
			m_Opponents.push_back(Team[idx].position);
		}
	}

}

double InterceptionThreatHeuristic::getValue(double x, double y) {
	//// Improves standing free by looking at the intercept by enemy thread

	// player with ball
	Vector2D ballPos = m_ball.getPosition().getVector2D();

	// receiving position
	Vector2D receivingPos = Vector2D(x, y);

	return chance_of_intercept(ballPos, receivingPos, m_Opponents,
			m_interceptionChanceStartDistance, m_interceptionChanceIncreasePerMeter, m_interceptionChancePenaltyFactor);
}

// ----------------------------------------------------------------------------------------
InfluenceCurrentPositionsHeuristic::InfluenceCurrentPositionsHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
		const std::vector<TeamPlannerRobot>& Team, double maxPossibleFieldDistance) :
																		GridHeuristic(id, weight, pgid),
																		m_Team(Team),
																		m_dScaling(maxPossibleFieldDistance) {

}

double InfluenceCurrentPositionsHeuristic::getValue(double x, double y) {
	// add influence of the current own positions of all players of our team. Penalty increases linearly with distance.
	double value = 1.0;
	// Add cones around all current positions
	for (unsigned idx = 0; idx < m_Team.size(); idx++) {
		if (!m_Team[idx].assigned ) {
			// player is not yet assigned. If he's assigned, his position does not have an advantage anymore, so no cone for him.
			double temp_value = m_Team[idx].position.getPosition().getVector2D().distanceTo( Vector2D(x,y) ) / m_dScaling;
			value = ( value < temp_value ) ? value : temp_value;
		}
	}
	return value;
}

// ----------------------------------------------------------------------------------------
InfluencePreviousAssignedPositionsHeuristic::InfluencePreviousAssignedPositionsHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
		const std::vector<TeamPlannerRobot>& Team, double maxPossibleFieldDistance, dynamic_role_e dynamic_role) :
																		GridHeuristic(id, weight, pgid),
																		m_Team(Team),
																		m_dScaling(maxPossibleFieldDistance),
																		m_dynamic_role(dynamic_role){
}

double InfluencePreviousAssignedPositionsHeuristic::getValue(double x, double y) {
	// add influence of the previous positions of any players of our team. Penalty increases linearly with distance.
	double value = 0.0;
	for (unsigned idx = 0; idx < m_Team.size(); idx++) {
		if (m_Team[idx].previous_result.previous_result_present) {
			if (m_Team[idx].previous_result.dynamic_role == m_dynamic_role) {
				Vector2D prevPos = Vector2D(m_Team[idx].previous_result.end_position.x, m_Team[idx].previous_result.end_position.y);
				// check if previous assigned
				value += prevPos.distanceTo( Vector2D(x,y) ) / m_dScaling;
			}
		}
	}
	return value;
}


// ----------------------------------------------------------------------------------------
ShootOnGoalHeuristic::ShootOnGoalHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
		const std::vector<TeamPlannerRobot>& Team,
		const std::vector<TeamPlannerOpponent>& Opponents,
		const FieldConfig& fieldConfig,
		const ball_pickup_position_t& ball_pickup_position) :
								GridHeuristic(id, weight, pgid),
								m_Team(Team),
								m_Opponents(Opponents),
								m_opponentGoal(fieldConfig.getOpponentGoal()),
								m_rightPole_x(fieldConfig.getGoalAreaWidth()*0.5),
								m_leftPole_x(-m_rightPole_x),
								m_robotRadius(fieldConfig.getRobotRadius()),
								m_ball_pickup_position(ball_pickup_position),
								m_min_own_y_pos(-fieldConfig.getMaxFieldY()*0.5) // half own half

{
}

double ShootOnGoalHeuristic::getValue(double x, double y) {
	//	Shoot on goal
	//	-- time in possession goal success factor: 0 to 2 sec. x 1.0, from 2.0 to 5.0 seconds x0.5 , after 5.0 seconds: 1.0x (force shot on goal)
	double value = 0.0;
	if (y >= m_min_own_y_pos) {
		value = 1.0;
		// on distance to shoot on goal (no shots on goal if too far away)

		for( unsigned int i = 0; i < m_Opponents.size(); i++){
			Vector2D opponent = m_Opponents[i].position.getPosition().getVector2D();
			// TODO prefer locations further from any robots

			//	- Filter opponents on keeper. Robot within 1.5 meter from opponent goal
			if (opponent.distanceTo(m_opponentGoal) > 1.5) {
				// only opponent more than 1.5 meter from goal are considered as opponent field-players
				bool corner1_in_triangle = inTriangle(m_rightPole_x, m_opponentGoal.m_y, m_leftPole_x, m_opponentGoal.m_y, x, y, opponent.m_x+m_robotRadius, opponent.m_y+m_robotRadius);
				bool corner2_in_triangle = inTriangle(m_rightPole_x, m_opponentGoal.m_y, m_leftPole_x, m_opponentGoal.m_y, x, y, opponent.m_x+m_robotRadius, opponent.m_y-m_robotRadius);
				bool corner3_in_triangle = inTriangle(m_rightPole_x, m_opponentGoal.m_y, m_leftPole_x, m_opponentGoal.m_y, x, y, opponent.m_x-m_robotRadius, opponent.m_y+m_robotRadius);
				bool corner4_in_triangle = inTriangle(m_rightPole_x, m_opponentGoal.m_y, m_leftPole_x, m_opponentGoal.m_y, x, y, opponent.m_x-m_robotRadius, opponent.m_y-m_robotRadius);
				if (corner1_in_triangle || corner2_in_triangle || corner3_in_triangle || corner4_in_triangle) {
					//	-- each opponent (except goalie) in goal-triangle (ball-goal-posts) decrease probability with 40 %
					// only valid for flat shots.
					// for lob-shots: opponent must be within the triangle and less then x meters away (distance to shoot over player)
					value *= 0.4;
				}
			}
		}
	}
	else {
		//	- - on own half: 1%
		value = 0.01;
	}

	// invert value
	value = fabs(value - 1.0);
	return value;
}



// ----------------------------------------------------------------------------------------
PassHeuristic::PassHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
		const std::vector<TeamPlannerRobot>& Team,
		const std::vector<TeamPlannerOpponent>& Opponents,
		const FieldConfig& fieldConfig,
		const ball_pickup_position_t& ball_pickup_position,
		const PlannerOptions& plannerOptions) :
										GridHeuristic(id, weight, pgid),
										m_Team(Team),
										m_Opponents(vector<MovingObject>()),
										m_robotRadius(fieldConfig.getRobotRadius()),
										m_ball_pickup_position(ball_pickup_position),
										m_interceptionChanceStartDistance(plannerOptions.interceptionChanceStartDistance),
										m_interceptionChanceIncreasePerMeter(plannerOptions.interceptionChanceIncreasePerMeter),
										m_interceptionChancePenaltyFactor(plannerOptions.interceptionChancePenaltyFactor),
										m_numberOfFieldPlayers(0)
{
	// calculate number of own fieldplayers
	for( unsigned int team_idx = 1; team_idx < m_Team.size(); team_idx++){
		if (m_Team[team_idx].player_type == FIELD_PLAYER) {
			m_numberOfFieldPlayers++;
		}
	}
	for (unsigned idx = 0; idx < Opponents.size(); idx++) {
		m_Opponents.push_back(Opponents[idx].position);
	}
}

double PassHeuristic::getValue(double x, double y) {
	double value = 0.0;

	Vector2D dribbelPos = Vector2D(x, y);
	if (m_numberOfFieldPlayers == 0) {
		// no team mate; So give all positions same value
		return 1.0;
	}

	double pointPenalty = 0.0;
	for( unsigned int team_idx = 1; team_idx < m_Team.size(); team_idx++){
		if (m_Team[team_idx].player_type != FIELD_PLAYER) {
			continue; // skip this player; not a field player.
		}
		Vector2D teamMatePos = m_Team[team_idx].position.getPosition().getVector2D();

		// calculate interception chance (default algorithm used to make pass by ball player and attack supporter for finding position.
		double interChange = chance_of_intercept(dribbelPos, teamMatePos, m_Opponents,
						m_interceptionChanceStartDistance, m_interceptionChanceIncreasePerMeter, m_interceptionChancePenaltyFactor);
		// if inception chance is 0, then no interception is expected. > 2.0 interception likely
		double penalty = 1.0;
		if (interChange < 1.0) {
			// small change on interception
			penalty = 0.5;
			if (interChange < 0.25) {
				penalty = 0.25;
			}
			if (interChange < 0.05) {
				// no change on interception
				penalty = 0.0;
			}
		}
		pointPenalty += penalty;

	}

	// make value between 0 and 1
	value = pointPenalty / m_numberOfFieldPlayers;  // div by zero prevented in begin of function.

	return value;
}


// ----------------------------------------------------------------------------------------
// Calculate penalty for the position with respect to the opponents
StayAwayFromOpponentsHeuristic::StayAwayFromOpponentsHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
		const Vector2D& ballPlayerPos,
		const MovingObject& ball,
		const std::vector<TeamPlannerOpponent>& Opponents, const double radius) :
												GridHeuristic(id, weight, pgid),
												m_ballPlayerPos(ballPlayerPos),
												m_ball(ball.getXYlocation()),
												m_Opponents(Opponents),
												m_radius(radius) {
	// calculate min and max angle. If opponent is this angle then it will be ignored (behind ball player)
	m_angle_ball_ballplayer_min = from_pitopi(m_ball.angle(m_ballPlayerPos) - deg2rad(60));
	m_angle_ball_ballplayer_max = from_pitopi(m_ball.angle(m_ballPlayerPos) + deg2rad(60));
}

double StayAwayFromOpponentsHeuristic::getValue(double x, double y) {
	double value = 0.0;
	for (unsigned idx = 0; idx < m_Opponents.size(); idx++) {
		Vector2D opponentPos = m_Opponents[idx].position.getPosition().getVector2D();
		if (opponentPos.distanceTo(Vector2D(x,y)) < m_radius) {
			double ang = m_ball.angle(opponentPos);
			bool behind_ballPlayer = (ang > m_angle_ball_ballplayer_min && ang < m_angle_ball_ballplayer_max);
			if (!behind_ballPlayer) {
				// opponent is in front of ball player
				double penalty = 1.0 - (opponentPos.distanceTo(Vector2D(x,y)) / m_radius);
				value = max(value, penalty);
			}
			else {
				// point is behind ball player so it will be ignored, but need to prevent go to this opponent.
				if (opponentPos.distanceTo(Vector2D(x,y)) < 1.0)
				{
					// point is close to opponent to ignore. : prevent to go to this point
					value = 1.0;
				}
			}
		}
	}
	return value;
}



