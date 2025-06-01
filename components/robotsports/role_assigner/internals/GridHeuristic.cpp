/*
 * GridHeuristic.cpp
 *
 *  Created on: Sep 12, 2016
 *      Author: jurge
 */

#include "GridHeuristic.hpp"
#include "MathUtils.hpp"
#include "RoleAssignerData.hpp"
#include "RoleAssignerGridInfoData.hpp"

#include <cmath>
#include <limits>
#include <iostream>
#include "Environment.hpp"

using namespace MRA;

double grid_eps = 1e-3; // 1 mm

//-------------------------------------------------------------------------------------------------------
GridHeuristic::GridHeuristic(const char * id_str, double w, RoleAssignerGridInfoData& pgid) :
                                                                id(id_str), weight(w)
{
    pgid.name.push_back(id_str);
    pgid.weight.push_back(w);
}

GridHeuristic::~GridHeuristic() {

}

//-------------------------------------------------------------------------------------------------------
InSquareHeuristic::InSquareHeuristic(const char * id, double weight, RoleAssignerGridInfoData& pgid,
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
InCircleHeuristic::InCircleHeuristic(const char * id, double weight, RoleAssignerGridInfoData& pgid,
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
BallSetplayAgainstHeuristic::BallSetplayAgainstHeuristic(const char * id, double weight, RoleAssignerGridInfoData& pgid,
        double cx, double cy, double radius, const Environment& rEnvironment) :
                                GridHeuristic(id, weight, pgid),
                                m_cx(cx),
                                m_cy(cy),
                                m_radius(radius),
                                m_rEnvironment(rEnvironment)
{
    // empty
}

double BallSetplayAgainstHeuristic::getValue(double x, double y) {
    // minimal distance to ball in own penalty area (RC 13.4) : 75 cm + small margin (10 cm) [issue #110)
    const double MIN_DIST_TO_BALL_IN_OWN_PENALTY_AREA = 0.75 + 0.1;

    double value = 0.0;
    if (m_rEnvironment.isInOwnPenaltyArea(x, y)) {
        double dist2Ball = Geometry::Position(x,y).distanceTo(Geometry::Position(m_cx, m_cy));
        if (dist2Ball < MIN_DIST_TO_BALL_IN_OWN_PENALTY_AREA) {
            // too close too the ball in penalty area
            value = 1.0;
        }
        else {
            // more then minimum distance to the ball
            value = 0.0;
        }
    }
    else if (((x-m_cx)*(x-m_cx) + (y-m_cy)*(y-m_cy)) < (m_radius*m_radius)) {
        // too close too the ball (outside penalty area)
        value = 1.0;
    }
    return value;
}


//-------------------------------------------------------------------------------------------------------
DistanceToLineHeuristic::DistanceToLineHeuristic(const char * id, double weight, RoleAssignerGridInfoData& pgid,
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
        value = getDistanceFromPointToLineSegment(m_x1, m_y1, m_x2, m_y2, x, y) / m_scaling;
    }
    if (m_invert) {
        value = fabs(value - 1.0);
    }
    return value;
}

//-------------------------------------------------------------------------------------------------------
InTriangleHeuristic::InTriangleHeuristic(const char * id, double weight, RoleAssignerGridInfoData& pgid,
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
InOppenentPenaltyAreaHeuristic::InOppenentPenaltyAreaHeuristic(const char * id, double weight, RoleAssignerGridInfoData& pgid,
                                                               const RoleAssignerData& r_role_assigner_data) :
                        InSquareHeuristic(id, weight, pgid,
                                -(r_role_assigner_data.parameters.grid_opponent_goal_clearance_x*0.5), r_role_assigner_data.environment.getMaxFieldY(),
                                +(r_role_assigner_data.parameters.grid_opponent_goal_clearance_x*0.5), r_role_assigner_data.environment.getMaxFieldY() - r_role_assigner_data.parameters.grid_opponent_goal_clearance_y)
{
    // empty
}

//-------------------------------------------------------------------------------------------------------
// Calculate penalty for position in own penaltyArea
InOwnPenaltyAreaHeuristic::InOwnPenaltyAreaHeuristic(const char * id, double weight, RoleAssignerGridInfoData& pgid,
                                                     const RoleAssignerData& r_role_assigner_data) :
                                InSquareHeuristic(id, weight, pgid,
                                        -(r_role_assigner_data.parameters.grid_opponent_goal_clearance_x*0.5), (-r_role_assigner_data.environment.getMaxFieldY()) + r_role_assigner_data.parameters.grid_opponent_goal_clearance_y,
                                        +(r_role_assigner_data.parameters.grid_opponent_goal_clearance_x*0.5),  -r_role_assigner_data.environment.getMaxFieldY())
{
    // empty
}

//-------------------------------------------------------------------------------------------------------
// Calculate penalty when already assigned field player has end position in own penaltyArea
AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic::AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic(const char * id, double weight,
        RoleAssignerGridInfoData& pgid, const RoleAssignerData& r_role_assigner_data) :
                                                                        GridHeuristic(id, weight, pgid),
                                                                        m_alreadyPlayerAssignedToOwnPenaltyArea(false),
                                                                        m_rEnvironment(r_role_assigner_data.environment) {
    for (unsigned idx = 0; idx < r_role_assigner_data.team.size(); idx++) {
        if (r_role_assigner_data.team_admin[idx].assigned && r_role_assigner_data.team[idx].player_type == player_type_e::FIELD_PLAYER) {
            // field player is assigned and has a path
            double end_x = r_role_assigner_data.team_admin[idx].result.target.x;
            double end_y = r_role_assigner_data.team_admin[idx].result.target.y;
            if (m_rEnvironment.isInOwnPenaltyArea(end_x, end_y) or m_rEnvironment.isInOwnGoalArea(end_x, end_y)) {
                m_alreadyPlayerAssignedToOwnPenaltyArea = true;
                break; // break out the loop
            }
        }
    }
}

double AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic::getValue(double x, double y) {
    double value = 0.0;
    if (m_alreadyPlayerAssignedToOwnPenaltyArea && m_rEnvironment.isInOwnPenaltyArea(x,y)) {
        value = 1.0; // don't allow locations in own penalty area
    }
    return value;
}

//-------------------------------------------------------------------------------------------------------
// Calculate penalty when already assigned field player has end position in opponent penaltyArea
AlreadyPlayerAssignedToOpponentPenaltyAreaHeuristic::AlreadyPlayerAssignedToOpponentPenaltyAreaHeuristic(const char * id, double weight,
        RoleAssignerGridInfoData& pgid, const RoleAssignerData& r_role_assigner_data) :
                                                                        GridHeuristic(id, weight, pgid),
                                                                        m_alreadyPlayerAssignedToOpponentPenaltyArea(false),
                                                                        m_rEnvironment(r_role_assigner_data.environment) {
    for (unsigned idx = 0; idx < r_role_assigner_data.team.size(); idx++) {
        if (r_role_assigner_data.team_admin[idx].assigned  && r_role_assigner_data.team[idx].player_type == player_type_e::FIELD_PLAYER) {
            // field player is assigned and has a path
            double end_x = r_role_assigner_data.team_admin[idx].result.target.x;
            double end_y = r_role_assigner_data.team_admin[idx].result.target.y;
            if  (m_rEnvironment.isInOpponentPenaltyArea(end_x, end_y)) {
                m_alreadyPlayerAssignedToOpponentPenaltyArea = true;
                break; // break out the loop
            }
        }
    }
}

double AlreadyPlayerAssignedToOpponentPenaltyAreaHeuristic::getValue(double x, double y) {
    double value = 0.0;
    if (m_alreadyPlayerAssignedToOpponentPenaltyArea && m_rEnvironment.isInOpponentPenaltyArea(x,y)) {
        value = 1.0; // don't allow locations in own penalty area
    }
    return value;
}

//-------------------------------------------------------------------------------------------------------
// Calculate penalty for position in own goalArea
InOwnGoalAreaHeuristic::InOwnGoalAreaHeuristic(const char *id, double weight, RoleAssignerGridInfoData& pgid,
                                               const RoleAssignerData& r_role_assigner_data) :
                                                InSquareHeuristic(id, weight, pgid,
                                                        -(r_role_assigner_data.parameters.grid_own_goal_clearance_x*0.5), (-r_role_assigner_data.environment.getMaxFieldY()) + r_role_assigner_data.parameters.grid_own_goal_clearance_y,
                                                        +(r_role_assigner_data.parameters.grid_own_goal_clearance_x*0.5), -r_role_assigner_data.environment.getMaxFieldY()) {

}


//-------------------------------------------------------------------------------------------------------
// Calculate penalty for position close to ball
DistanceToBallHeuristic::DistanceToBallHeuristic(const char *id, double weight, RoleAssignerGridInfoData& pgid,
        double ball_x, double ball_y, double radius) : InCircleHeuristic(id, weight, pgid, ball_x, ball_y,radius, true /*invert */)
{

}

//-------------------------------------------------------------------------------------------------------
// Calculate penalty for position close to ball
InfluenceBallHeuristic::InfluenceBallHeuristic(const char *id, double weight, RoleAssignerGridInfoData& pgid,
        double ball_x, double ball_y, double radius)  : InCircleHeuristic(id, weight, pgid, ball_x, ball_y,radius)
{

}

// ----------------------------------------------------------------------------------------
// Calculate influence  (penalty) for the position with respect to corner of the field
InfluenceCornerHeuristic::InfluenceCornerHeuristic(const char *id, double weight, RoleAssignerGridInfoData& pgid,
        const Environment& rEnvironment) :
                                                                GridHeuristic(id, weight, pgid),
                                                                m_rEnvironment(rEnvironment)
{
    // emptys
}

double InfluenceCornerHeuristic::getValue(double x, double y) {
    // add influence corner : 0.08 linear decreasing till 4 meters from corner (inside the field)
    // out side the field: 1.0
    double value = 1.0; // outside field
    if (m_rEnvironment.isInField(x, y, -0.01)) {
        // x, y are in the field with 1 cm margin

        double distXtoCorner = m_rEnvironment.getMaxFieldX();
        double distYtoCorner = m_rEnvironment.getMaxFieldY();
        double distXFactor = (x < 0) ? -1.0 : 1.0;
        double distYFactor = (y < 0) ? -1.0 : 1.0;
        Geometry::Position closestCorner = Geometry::Position(distXtoCorner*distXFactor, distYtoCorner*distYFactor);
        double dist2corner = closestCorner.distanceTo(Geometry::Position(x,y));
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
CollideTeamMateHeuristic::CollideTeamMateHeuristic(const char *id, double weight, RoleAssignerGridInfoData& pgid,
                                                   const RoleAssignerData& r_role_assigner_data, double radius, bool assigned_attack_supporters) :
                                                                        GridHeuristic(id, weight, pgid),
                                                                        m_r_role_assigner_data(r_role_assigner_data),
                                                                        m_radius(radius),
                                                                        m_assigned_attack_supporters(assigned_attack_supporters)
{

}

double CollideTeamMateHeuristic::getValue(double x, double y) {
    // add influence: team mate also supporter (half ball influence?)
    // TODO: rewrite this function to not have ax_sqr and ball_c, but radius and penalty.
    double value = 0.0;
    for (unsigned idx = 0; idx < m_r_role_assigner_data.team_admin.size(); idx++) {
        if (m_r_role_assigner_data.team_admin[idx].assigned) {
            if (m_assigned_attack_supporters and  m_r_role_assigner_data.team_admin[idx].result.role != MRA::Datatypes::DynamicRole::ATTACKER_GENERIC)
            {
                // role is not attack supporter
            }
            // player is assigned and has a path
            else if (m_r_role_assigner_data.team_admin[idx].result.target.distanceTo(Geometry::Position(x,y)) < m_radius) {
                value = 1.0;
            }
        }
    }
    return value;
}

// ----------------------------------------------------------------------------------------
// Calculate penalty for the position with respect to the opponents
InfluenceOpponentsHeuristic::InfluenceOpponentsHeuristic(const char *id, double weight, RoleAssignerGridInfoData& pgid,
        const std::vector<RoleAssignerOpponent>& Opponents, double radius) :
                                                GridHeuristic(id, weight, pgid),
                                                m_Opponents(Opponents),
                                                m_radius(radius) {
    // empty
}

double InfluenceOpponentsHeuristic::getValue(double x, double y) {
    double value = 0.0;
    // add influence other players. Opponents twice team meat. 1/10 support influence (20x² ?)
    for (unsigned bar_idx = 0; bar_idx < m_Opponents.size(); bar_idx++) {
        Geometry::Position bar_pos = m_Opponents[bar_idx].position;
        if (bar_pos.distanceTo(Geometry::Position(x,y)) < m_radius) {
            value = 1.0;
        }
    }
    return value;
}
// ----------------------------------------------------------------------------------------
// Calculate penalty for position outside play field
OutsidePlayFieldHeuristic::OutsidePlayFieldHeuristic(const char *id, double weight, RoleAssignerGridInfoData& pgid, const Environment& rEnvironment, double extra_distance_to_sideline) :
                        InSquareHeuristic(id, weight, pgid,
                                -(rEnvironment.getMaxFieldX() - rEnvironment.getRobotRadius()- extra_distance_to_sideline),
                                 (rEnvironment.getMaxFieldY() - rEnvironment.getRobotRadius()- extra_distance_to_sideline),
                                 (rEnvironment.getMaxFieldX() - rEnvironment.getRobotRadius()- extra_distance_to_sideline),
                                -(rEnvironment.getMaxFieldY() - rEnvironment.getRobotRadius()- extra_distance_to_sideline), true /* inverted */) {
    // empty
}


// ----------------------------------------------------------------------------------------
OnLineBetweenPointsHeuristic::OnLineBetweenPointsHeuristic(const char *id, double weight, RoleAssignerGridInfoData& pgid,
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
        Geometry::Position Pintersect(px,py);
        Geometry::Position P1(m_x1, m_y1);
        Geometry::Position P2(m_x2, m_y2);
        double distPintoP1 = Pintersect.distanceTo(P1);
        double distPintoP2 = Pintersect.distanceTo(P2);
        double distP1toP2 = P1.distanceTo(P2);
        if ((distPintoP1 >  distP1toP2) || (distPintoP2 >  distP1toP2) ) {
            // P is not between P1 and P2. use shortest distance to line piece
            Geometry::Position p(x,y);
            distToLinePiece = std::min(p.distanceTo(P1), p.distanceTo(P2));
        }
        else {
            distToLinePiece = getDistanceFromPointToLineSegment(m_x1, m_y1, m_x2, m_y2, x, y);
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
DesiredY::DesiredY(const char *id, double weight, RoleAssignerGridInfoData& pgid,
        double desired_y, const Environment& rEnvironment) :
                        DistanceToLineHeuristic(id, weight, pgid,
                                -rEnvironment.getMaxFullFieldX(), desired_y, rEnvironment.getMaxFullFieldX(), desired_y, fabs(desired_y)+rEnvironment.getMaxFieldY())
                        // scale-factor set to max-y distance within playing field to have good sensibility
{

}

// ----------------------------------------------------------------------------------------
DesiredX::DesiredX(const char *id, double weight, RoleAssignerGridInfoData& pgid,
        double desired_x, const Environment& rEnvironment) :
                                DistanceToLineHeuristic(id, weight, pgid,
                                        desired_x, -rEnvironment.getMaxFullFieldY(), desired_x, rEnvironment.getMaxFullFieldY(), fabs(desired_x)+rEnvironment.getMaxFieldX())
                                // scale-factor set to max-x distance within playing field to have good sensibility
{

}

// ----------------------------------------------------------------------------------------
class DesiredXSweeper : public GridHeuristic
{
public:
    DesiredXSweeper(const char *id, double weight, RoleAssignerGridInfoData& pgid,
            double desired_x, const Environment& rEnvironment);
    virtual ~DesiredXSweeper() {};
    double getValue(double x, double y);
private:
    const double m_desired_x;
    const double m_dHalfLength;
    const double m_dScaling;
};

// ----------------------------------------------------------------------------------------
DistanceToHeuristic::DistanceToHeuristic(const char *id, double weight, RoleAssignerGridInfoData& pgid,
        const Geometry::Position& pos, double dScaling):
                                                GridHeuristic(id, weight, pgid),
                                                m_pos(pos),
                                                m_dScaling(dScaling)
{

}

double DistanceToHeuristic::getValue(double x, double y) {
    double value = 0.0;
    if (fabs(m_dScaling) > 1e-16) {
        value =  m_pos.distanceTo(Geometry::Position(x, y)) / m_dScaling;  // distance closer to position is better (lower)
    }
    return value;
}


// ----------------------------------------------------------------------------------------
DistanceToPointHeuristic::DistanceToPointHeuristic(const char *id, double weight, RoleAssignerGridInfoData& pgid,
        const Geometry::Position& pos, double dScaling, double maxRange, bool inverted) :
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
        value =  m_pos.distanceTo(Geometry::Position(x, y));
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
DistanceToEnemyGoalHeuristic::DistanceToEnemyGoalHeuristic(const char *id, double weight, RoleAssignerGridInfoData& pgid,
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
NotOnLineBetweenBallAndOpponentGoalHeuristic::NotOnLineBetweenBallAndOpponentGoalHeuristic(const char *id, double weight, RoleAssignerGridInfoData& pgid,
        double ball_x, double ball_y, double left_pole_x, double left_pole_y, double right_pole_x, double right_pole_y):
                            InTriangleHeuristic(id, weight, pgid,
                                    ball_x, ball_y, right_pole_x, right_pole_y,  left_pole_x, left_pole_y, false /*inverted: 1.0 if outside triangle */) {
    // empty
}

// ----------------------------------------------------------------------------------------
InterceptionThreatHeuristic::InterceptionThreatHeuristic(const char *id, double weight, RoleAssignerGridInfoData& pgid,
        const MRA::Geometry::Point& ball,
        const RoleAssignerData& r_role_assigner_data, bool skipOwnTeam) :
                                            GridHeuristic(id, weight, pgid),
                                            m_ball(ball),
                                            m_interceptionChanceStartDistance(r_role_assigner_data.parameters.interceptionChanceStartDistance),
                                            m_interceptionChanceIncreasePerMeter(r_role_assigner_data.parameters.interceptionChanceIncreasePerMeter),
                                            m_interceptionChancePenaltyFactor(r_role_assigner_data.parameters.interceptionChancePenaltyFactor)
{
    for (unsigned idx = 0; idx < r_role_assigner_data.opponents.size(); idx++) {
        m_Opponents.push_back(r_role_assigner_data.opponents[idx].position);
    }
    // avoid being intercepted by own team mate (attack supporter)
    if (not skipOwnTeam) {
        for (unsigned idx = 0; idx < r_role_assigner_data.team.size(); idx++) {
            if (r_role_assigner_data.team_admin[idx].assigned and
                r_role_assigner_data.team_admin[idx].result.role == MRA::Datatypes::DynamicRole::ATTACKER_GENERIC) {
                m_Opponents.push_back(r_role_assigner_data.team[idx].position);
            }
        }
    }
}

double InterceptionThreatHeuristic::getValue(double x, double y) {
   // Improves standing free by looking at the intercept by enemy thread

    // receiving position
    Geometry::Point receivingPos = Geometry::Position(x, y);
    return chance_of_intercept(m_ball, receivingPos, m_Opponents,
            m_interceptionChanceStartDistance, m_interceptionChanceIncreasePerMeter, m_interceptionChancePenaltyFactor);
}

// ----------------------------------------------------------------------------------------
InfluenceCurrentPositionsHeuristic::InfluenceCurrentPositionsHeuristic(const char *id, double weight, RoleAssignerGridInfoData& pgid,
                                                                       const RoleAssignerData& r_role_assigner_data,
                                                                       double dScaling):
                                                                        GridHeuristic(id, weight, pgid),
                                                                        m_r_role_assigner_data(r_role_assigner_data),
                                                                        m_dScaling(dScaling) {

}

double InfluenceCurrentPositionsHeuristic::getValue(double x, double y) {
    // add influence of the current own positions of all players of our team. Penalty increases linearly with distance.
    double value = 1.0;
    // Add cones around all current positions
    for (unsigned idx = 0; idx < m_r_role_assigner_data.team_admin.size(); idx++) {
        if (not m_r_role_assigner_data.team_admin[idx].assigned ) {
            // player is not yet assigned. If he's assigned, his position does not have an advantage anymore, so no cone for him.
            double temp_value = m_r_role_assigner_data.team[idx].position.distanceTo( Geometry::Position(x,y) ) / m_dScaling;
            value = ( value < temp_value ) ? value : temp_value;
        }
    }
    return value;
}

// ----------------------------------------------------------------------------------------
InfluencePreviousAssignedPositionsHeuristic::InfluencePreviousAssignedPositionsHeuristic(const char *id, double weight, RoleAssignerGridInfoData& pgid,
                                                                                         const RoleAssignerData& r_role_assigner_data,
                                                                double maxPossibleFieldDistance, MRA::Datatypes::DynamicRole role) :
                                                                        GridHeuristic(id, weight, pgid),
                                                                        m_r_role_assigner_data(r_role_assigner_data),
                                                                        m_dScaling(maxPossibleFieldDistance),
                                                                        m_role(role){
}

double InfluencePreviousAssignedPositionsHeuristic::getValue(double x, double y) {
    // add influence of the previous positions of any players of our team. Penalty increases linearly with distance.
    double value = 0.0;
    for (unsigned idx = 0; idx < m_r_role_assigner_data.previous_results.size(); idx++) {
        if (m_r_role_assigner_data.previous_results[idx].present) {
            if (m_r_role_assigner_data.previous_results[idx].role == m_role) {
                Geometry::Position prevPos = Geometry::Position(m_r_role_assigner_data.previous_results[idx].end_position.x,
                												m_r_role_assigner_data.previous_results[idx].end_position.y);
                // check if previous assigned
                value += prevPos.distanceTo( Geometry::Position(x,y) ) / m_dScaling;
            }
        }
    }
    return value;
}


// ----------------------------------------------------------------------------------------
ShootOnGoalHeuristic::ShootOnGoalHeuristic(const char *id, double weight, RoleAssignerGridInfoData& pgid,
        const std::vector<RoleAssignerRobot>& Team,
        const std::vector<RoleAssignerOpponent>& Opponents,
        const Environment& rEnvironment,
        const ball_pickup_position_t& ball_pickup_position) :
                                GridHeuristic(id, weight, pgid),
                                m_Team(Team),
                                m_Opponents(Opponents),
                                m_opponentGoal(rEnvironment.getOpponentGoal()),
                                m_rightPole_x(rEnvironment.getGoalAreaWidth()*0.5),
                                m_leftPole_x(-m_rightPole_x),
                                m_robotRadius(rEnvironment.getRobotRadius()),
                                m_ball_pickup_position(ball_pickup_position),
                                m_min_own_y_pos(-rEnvironment.getMaxFieldY()*0.5) // half own half

{
}

double ShootOnGoalHeuristic::getValue(double x, double y) {
    //    Shoot on goal
    //    -- time in possession goal success factor: 0 to 2 sec. x 1.0, from 2.0 to 5.0 seconds x0.5 , after 5.0 seconds: 1.0x (force shot on goal)
    double value = 0.0;
    if (y >= m_min_own_y_pos) {
        value = 1.0;
        // on distance to shoot on goal (no shots on goal if too far away)

        for( unsigned int i = 0; i < m_Opponents.size(); i++){
            Geometry::Position opponent = m_Opponents[i].position;
            // TODO prefer locations further from any robots

            //    - Filter opponents on keeper. Robot within 1.5 meter from opponent goal
            if (opponent.distanceTo(m_opponentGoal) > 1.5) {
                // only opponent more than 1.5 meter from goal are considered as opponent field-players
                bool corner1_in_triangle = inTriangle(m_rightPole_x, m_opponentGoal.y, m_leftPole_x, m_opponentGoal.y, x, y, opponent.x+m_robotRadius, opponent.y+m_robotRadius);
                bool corner2_in_triangle = inTriangle(m_rightPole_x, m_opponentGoal.y, m_leftPole_x, m_opponentGoal.y, x, y, opponent.x+m_robotRadius, opponent.y-m_robotRadius);
                bool corner3_in_triangle = inTriangle(m_rightPole_x, m_opponentGoal.y, m_leftPole_x, m_opponentGoal.y, x, y, opponent.x-m_robotRadius, opponent.y+m_robotRadius);
                bool corner4_in_triangle = inTriangle(m_rightPole_x, m_opponentGoal.y, m_leftPole_x, m_opponentGoal.y, x, y, opponent.x-m_robotRadius, opponent.y-m_robotRadius);
                if (corner1_in_triangle || corner2_in_triangle || corner3_in_triangle || corner4_in_triangle) {
                    //    -- each opponent (except goalie) in goal-triangle (ball-goal-posts) decrease probability with 40 %
                    // only valid for flat shots.
                    // for lob-shots: opponent must be within the triangle and less then x meters away (distance to shoot over player)
                    value *= 0.4;
                }
            }
        }
    }
    else {
        //    - - on own half: 1%
        value = 0.01;
    }

    // invert value
    value = fabs(value - 1.0);
    return value;
}



// ----------------------------------------------------------------------------------------
PassHeuristic::PassHeuristic(const char *id, double weight, RoleAssignerGridInfoData& pgid,
        const std::vector<RoleAssignerRobot>& Team,
        const std::vector<RoleAssignerOpponent>& Opponents,
        const Environment& rEnvironment,
        const ball_pickup_position_t& ball_pickup_position,
        const RoleAssignerParameters& parameters) :
                                        GridHeuristic(id, weight, pgid),
                                        m_Team(Team),
                                        m_Opponents(std::vector<MRA::Geometry::Position>()),
                                        m_robotRadius(rEnvironment.getRobotRadius()),
                                        m_ball_pickup_position(ball_pickup_position),
                                        m_interceptionChanceStartDistance(parameters.interceptionChanceStartDistance),
                                        m_interceptionChanceIncreasePerMeter(parameters.interceptionChanceIncreasePerMeter),
                                        m_interceptionChancePenaltyFactor(parameters.interceptionChancePenaltyFactor),
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

    Geometry::Point dribbelPos = Geometry::Point(x, y);
    if (m_numberOfFieldPlayers == 0) {
        // no team mate; So give all positions same value
        return 1.0;
    }

    double pointPenalty = 0.0;
    for( unsigned int team_idx = 1; team_idx < m_Team.size(); team_idx++){
        if (m_Team[team_idx].player_type != FIELD_PLAYER) {
            continue; // skip this player; not a field player.
        }
        Geometry::Point teamMatePos = Geometry::Point(m_Team[team_idx].position.x, m_Team[team_idx].position.y);

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
StayAwayFromOpponentsHeuristic::StayAwayFromOpponentsHeuristic(const char *id, double weight, RoleAssignerGridInfoData& pgid,
        const Geometry::Position& ballPlayerPos,
        const MRA::Geometry::Position& ball,
        const std::vector<RoleAssignerOpponent>& Opponents, const double radius) :
                                                GridHeuristic(id, weight, pgid),
                                                m_ballPlayerPos(ballPlayerPos),
                                                m_ball(ball),
                                                m_Opponents(Opponents),
                                                m_radius(radius) {
    // calculate min and max angle. If opponent is this angle then it will be ignored (behind ball player)
    m_angle_ball_ballplayer_min = Geometry::wrap_pi(m_ball.angle(m_ballPlayerPos) - Geometry::deg_to_rad(60));
    m_angle_ball_ballplayer_max = Geometry::wrap_pi(m_ball.angle(m_ballPlayerPos) + Geometry::deg_to_rad(60));
}

double StayAwayFromOpponentsHeuristic::getValue(double x, double y) {
    double value = 0.0;
    for (unsigned idx = 0; idx < m_Opponents.size(); idx++) {
        Geometry::Position opponentPos = m_Opponents[idx].position;
        if (opponentPos.distanceTo(Geometry::Position(x,y)) < m_radius) {
            double ang = m_ball.angle(opponentPos);
            bool behind_ballPlayer = (ang > m_angle_ball_ballplayer_min && ang < m_angle_ball_ballplayer_max);
            if (!behind_ballPlayer) {
                // opponent is in front of ball player
                double penalty = 1.0 - (opponentPos.distanceTo(Geometry::Position(x,y)) / m_radius);
                value = std::max(value, penalty);
            }
            else {
                // point is behind ball player so it will be ignored, but need to prevent go to this opponent.
                if (opponentPos.distanceTo(Geometry::Position(x,y)) < 1.0)
                {
                    // point is close to opponent to ignore. : prevent to go to this point
                    value = 1.0;
                }
            }
        }
    }
    return value;
}


double MRA::chance_of_intercept(const Geometry::Point& pass_begin_vec, const Geometry::Point& pass_end_vec,
        const std::vector<Geometry::Position>& Opponents,
        double interceptionChanceStartDistance,
        double interceptionChanceIncreasePerMeter,
        double interceptionDistancePenaltyFactor)
{
    // Calculates the intercept by enemy threat
    double interceptionPenalty = 0;

    // check chance of intercept from begin to end position of the pass.
    // Check if in a trapezoid (smallest parallel side at begin of pass and largest parallel side at end of the pass.
    // Trapezoid is constructed with X meter from begin and X meter + pass-distance * increase_per_meter
    // Check for each opponent if the opponent is in the trapezoid. (divide trapezoid area in two triangle and
    // check if opponent is not in one of the triangles.
    // if opponent is  the trapezoid, the penalty is related to to distance of the pass-line

    double start_width = interceptionChanceStartDistance; // starting with for interception
    double increase_per_meter = interceptionChanceIncreasePerMeter;
    double dist_from_to = pass_end_vec.distanceTo(pass_begin_vec); // distance from pass begin to pass end point


    double angle_begin_to_end_pass = pass_end_vec.angle(pass_begin_vec); // angle from begin pass to end pass point
    double end_width = start_width + dist_from_to * increase_per_meter; // calculate size of largest parallel side of trapezoid

    // calculate largest parallel side of trapezoid
    double EndXp = pass_end_vec.x + (cos(angle_begin_to_end_pass + 0.5 * M_PI) * end_width);
    double EndYp = pass_end_vec.y + (sin(angle_begin_to_end_pass + 0.5 * M_PI) * end_width);
    double EndXm = pass_end_vec.x  + (cos(angle_begin_to_end_pass - 0.5 * M_PI) * end_width);
    double EndYm = pass_end_vec.y  + (sin(angle_begin_to_end_pass - 0.5 * M_PI) * end_width);


    // calculate smallest parallel side of trapezoid
    double BeginXp = pass_begin_vec.x + (cos(angle_begin_to_end_pass + 0.5 * M_PI) * start_width);
    double BeginYp = pass_begin_vec.y + (sin(angle_begin_to_end_pass + 0.5 * M_PI) * start_width);
    double BeginXm = pass_begin_vec.x  + (cos(angle_begin_to_end_pass - 0.5 * M_PI) * start_width);
    double BeginYm = pass_begin_vec.y  + (sin(angle_begin_to_end_pass - 0.5 * M_PI) * start_width);

    // loop for calculating angles between receiving positions and opponents
    for(Geometry::Position opponent : Opponents)
    {
        // return true if point (x,y) is in polygon defined by the points  else return false
        if (inTriangle(BeginXp, BeginYp, EndXm, EndYm, BeginXm, BeginYm, opponent.x , opponent.y) ||
                inTriangle(EndXm, EndYm, BeginXp, BeginYp, EndXp, EndYp, opponent.x , opponent.y) )
        {
            // Opponent is in the Trapezoid
            // calculate intercept point of the perpendicular from Opponent to the pass-line (shortest line to pass line for the opponent)
            Geometry::Point Per;
            intersectPerpendicular(Per.x, Per.y, pass_begin_vec.x, pass_begin_vec.y, pass_end_vec.x, pass_end_vec.y, opponent.x, opponent.y);
            // calculate distance from opponent intercept point (Vector Per) to pass begin point
            double distPer = pass_begin_vec.distanceTo(Per); //
            double ext_at_Per = start_width + distPer * increase_per_meter;
            double distOppToPer = opponent.distanceTo(Per); // distance opponent to  pass-line

            // penalty per opponent:
            // relative to distance to pass-line (divide by max distance to side of trapezoid at that distance
            double enemyPenalty = interceptionDistancePenaltyFactor * ((ext_at_Per-distOppToPer)/ext_at_Per);

            // sum all penalties, a point with a lot of intercept possibilities gets a high penalty
            interceptionPenalty = std::max(enemyPenalty, interceptionPenalty);
        }
    }

    return interceptionPenalty;
}

