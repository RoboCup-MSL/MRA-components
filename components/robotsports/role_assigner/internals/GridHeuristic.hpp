/*
 * GridHeuristic.h
 *
 *  Created on: Sep 12, 2016
 *      Author: jurge
 */

#ifndef PLANNER_GRID_GRIDHEURISTIC_H
#define PLANNER_GRID_GRIDHEURISTIC_H 1

#include "TeamPlay.hpp"
#include "PlannerGridInfoData.hpp"
#include "FieldConfig.h"
#include "WmTypes.h"
#include <string>

namespace MRA {

// ----------------------------------------------------------------------------------------
class GridHeuristic {
public:
    GridHeuristic(const char *  id_str, double w, PlannerGridInfoData& pgid);
    virtual ~GridHeuristic();
    virtual double getValue(double x, double y) = 0;
    std::string getId() { return id; }
    double getWeight() { return weight; }
private:
    std::string id;
    double weight;
};


// ----------------------------------------------------------------------------------------
// Calculate penalty for being in Square
class InSquareHeuristic : public GridHeuristic {

public:
    InSquareHeuristic(const char * id, double weight, PlannerGridInfoData& pgid,
            double x_left, double y_top, double x_right, double y_bottom, bool invert = false);
    virtual ~InSquareHeuristic() {};
    double getValue(double x, double y);
private:
    const double m_x_left;
    const double m_y_top;
    const double m_x_right;
    const double m_y_bottom;
    const bool m_invert;
};

// ----------------------------------------------------------------------------------------
// Calculate penalty for being in Circle
class InCircleHeuristic : public GridHeuristic {

public:
    InCircleHeuristic(const char * id, double weight, PlannerGridInfoData& pgid,
            double cx, double cy, double radius, bool invert = false);
    virtual ~InCircleHeuristic() {};
    double getValue(double x, double y);
private:
    const double m_cx;
    const double m_cy;
    const double m_radius;
    const bool m_invert;
};


// ----------------------------------------------------------------------------------------
// Calculate penalty for being in Circle, excluding own penalty area
class BallSetplayAgainstHeuristic : public GridHeuristic {

public:
    BallSetplayAgainstHeuristic(const char * id, double weight, PlannerGridInfoData& pgid,
            double cx, double cy, double radius, const FieldConfig& fieldConfig);
    virtual ~BallSetplayAgainstHeuristic() {};
    double getValue(double x, double y);
private:
    const double m_cx;
    const double m_cy;
    const double m_radius;
    const FieldConfig m_fieldConfig;
};



// ----------------------------------------------------------------------------------------
// Calculate penalty for distance from line between x1,y1 and x2, y2
class DistanceToLineHeuristic : public GridHeuristic {

public:
    DistanceToLineHeuristic(const char * id, double weight, PlannerGridInfoData& pgid,
            double x1, double y1, double x2, double y2, double scaling, bool invert = false);
    virtual ~DistanceToLineHeuristic() {};
    double getValue(double x, double y);
private:
    const double m_x1;
    const double m_y1;
    const double m_x2;
    const double m_y2;
    const double m_scaling;
    const bool m_invert;
};

// ----------------------------------------------------------------------------------------
// Calculate penalty when point is in given triangle
class InTriangleHeuristic : public GridHeuristic {

public:
    InTriangleHeuristic(const char * id, double weight, PlannerGridInfoData& pgid,
            double x1, double y1, double x2, double y2, double x3, double y3, bool invert = false);
    virtual ~InTriangleHeuristic() {};
    double getValue(double x, double y);
private:
    const double m_x1;
    const double m_y1;
    const double m_x2;
    const double m_y2;
    const double m_x3;
    const double m_y3;
    const bool m_invert;
};


// ----------------------------------------------------------------------------------------
// Calculate penalty for position in opponent penaltyArea
class InOppenentPenaltyAreaHeuristic : public InSquareHeuristic {

public:
    InOppenentPenaltyAreaHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            const TeamPlannerParameters& parameters, const FieldConfig& fieldConfig);
};

// ----------------------------------------------------------------------------------------
// Calculate penalty for position in own penaltyArea
class InOwnPenaltyAreaHeuristic : public InSquareHeuristic {

public:
    InOwnPenaltyAreaHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            const TeamPlannerParameters& parameters, const FieldConfig& fieldConfig);
};

//-------------------------------------------------------------------------------------------------------
// Calculate penalty when already assigned field player has end position in own penaltyArea
class AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic : public GridHeuristic {

public:
    AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            const std::vector<TeamPlannerRobot>& Team, const FieldConfig& fieldConfig);
    virtual ~AlreadyPlayerAssignedToOwnPenaltyAreaHeuristic() {};
    double getValue(double x, double y);
private:
    bool m_alreadyPlayerAssignedToOwnPenaltyArea;
    const FieldConfig& m_fieldConfig;
};

//-------------------------------------------------------------------------------------------------------
// Calculate penalty when already assigned field player has end position in opponent penaltyArea
class AlreadyPlayerAssignedToOpponentPenaltyAreaHeuristic : public GridHeuristic {

public:
    AlreadyPlayerAssignedToOpponentPenaltyAreaHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            const std::vector<TeamPlannerRobot>& Team, const FieldConfig& fieldConfig);
    virtual ~AlreadyPlayerAssignedToOpponentPenaltyAreaHeuristic() {};
    double getValue(double x, double y);
private:
    bool m_alreadyPlayerAssignedToOpponentPenaltyArea;
    const FieldConfig& m_fieldConfig;
};

// ----------------------------------------------------------------------------------------
// Calculate penalty for position in own goalArea
class InOwnGoalAreaHeuristic : public InSquareHeuristic
{
public:
    InOwnGoalAreaHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            const TeamPlannerParameters& parameters, const FieldConfig& fieldConfig);
};


// ----------------------------------------------------------------------------------------
class DistanceToBallHeuristic : public InCircleHeuristic
{
public:
    DistanceToBallHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            double ball_x, double ball_y, double radius);
};

// ----------------------------------------------------------------------------------------
class InfluenceBallHeuristic  : public InCircleHeuristic
{
public:
    InfluenceBallHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            double ball_x, double ball_y, double radius);
};


// ----------------------------------------------------------------------------------------
class InfluenceCornerHeuristic : public GridHeuristic
{
public:
    InfluenceCornerHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            const FieldConfig& m_fieldConfig);
    virtual ~InfluenceCornerHeuristic() {};
    double getValue(double x, double y);
private:
    const FieldConfig& m_fieldConfig;
};


// ----------------------------------------------------------------------------------------
class CollideTeamMateHeuristic : public GridHeuristic
{
public:
    CollideTeamMateHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            const std::vector<TeamPlannerRobot>& Team, double radius);
    virtual ~CollideTeamMateHeuristic() {};
    double getValue(double x, double y);
private:
    const std::vector<TeamPlannerRobot>& m_Team;
    const double m_radius;
};

// ----------------------------------------------------------------------------------------
class InfluenceOpponentsHeuristic : public GridHeuristic
{
public:
    InfluenceOpponentsHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            const std::vector<TeamPlannerOpponent>& Opponents, double radius);
    virtual ~InfluenceOpponentsHeuristic() {};
    double getValue(double x, double y);
private:
    const std::vector<TeamPlannerOpponent>& m_Opponents;
    const double m_radius;
};


// ----------------------------------------------------------------------------------------
class OutsidePlayFieldHeuristic : public InSquareHeuristic
{
public:
    OutsidePlayFieldHeuristic(const char *id, double weight, PlannerGridInfoData& pgid, const FieldConfig& fieldConfig, double extra_distance_to_sideline);
};

// ----------------------------------------------------------------------------------------
class OnLineBetweenPointsHeuristic : public GridHeuristic
{
public:
    OnLineBetweenPointsHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            double x1, double y1, double x2, double y2, double maxPossibleFieldDistance);
    virtual ~OnLineBetweenPointsHeuristic() {};
    double getValue(double x, double y);
private:
    const double m_x1;
    const double m_y1;
    double m_x2;
    double m_y2;
    const double m_dScaling;
};

// ----------------------------------------------------------------------------------------
class NotOnLineBetweenBallAndOpponentGoalHeuristic : public InTriangleHeuristic
{
public:
    NotOnLineBetweenBallAndOpponentGoalHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            double ball_x, double ball_y, double left_pole_x, double left_pole_y, double right_pole_x, double right_pole_y);
};

// ----------------------------------------------------------------------------------------
class DesiredY : public DistanceToLineHeuristic
{
public:
    DesiredY(const char *id, double weight, PlannerGridInfoData& pgid,
            double desired_y, const FieldConfig& fieldConfig);
};

// ----------------------------------------------------------------------------------------
class DesiredX : public DistanceToLineHeuristic
{
public:
    DesiredX(const char *id, double weight, PlannerGridInfoData& pgid,
            double desired_x, const FieldConfig& fieldConfig);
};


// ----------------------------------------------------------------------------------------
class DistanceToHeuristic : public GridHeuristic
{
public:
    DistanceToHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            const Geometry::Position& pos, double dScaling);
    virtual ~DistanceToHeuristic() {};
    double getValue(double x, double y);
private:
    const Geometry::Position m_pos;
    const double m_dScaling;
};


// ----------------------------------------------------------------------------------------
class DistanceToPointHeuristic : public GridHeuristic
{
public:
    DistanceToPointHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            const Geometry::Position& pos, double dScaling, double maxRange, bool inverted);

    virtual ~DistanceToPointHeuristic() {};
    double getValue(double x, double y);
private:
    const Geometry::Point m_pos;
    const double m_dScaling;
    double m_dMaxRange;
    bool m_bInverted;
};


// ----------------------------------------------------------------------------------------
class DistanceToEnemyGoalHeuristic : public GridHeuristic
{
public:
    DistanceToEnemyGoalHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            const double penaltyAreaY, double dScaling);
    virtual ~DistanceToEnemyGoalHeuristic() {};
    double getValue(double x, double y);
private:
    const double m_penaltyAreaY;
    const double m_dScaling;
};

//-----------------------------------------------------------------------------------------------------------------------------
class InterceptionThreatHeuristic : public GridHeuristic
{
public:
    InterceptionThreatHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            const Geometry::Point& ball,
            const std::vector<TeamPlannerRobot>& Team, const std::vector<TeamPlannerOpponent>& Opponents,
            double interceptionChanceStartDistance,
            double interceptionChanceIncreasePerMeter,
            double interceptionChancePenaltyFactor);
    virtual ~InterceptionThreatHeuristic() {};
    double getValue(double x, double y);
private:
    const Geometry::Point& m_ball;
    const std::vector<TeamPlannerRobot>& m_Team;
    std::vector<Geometry::Position> m_Opponents;
    const double m_interceptionChanceStartDistance;
    const double m_interceptionChanceIncreasePerMeter;
    const double m_interceptionChancePenaltyFactor;
};

// ----------------------------------------------------------------------------------------
class InfluenceCurrentPositionsHeuristic : public GridHeuristic
{
public:
    InfluenceCurrentPositionsHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            const std::vector<TeamPlannerRobot>& Team, double dScaling);
    virtual ~InfluenceCurrentPositionsHeuristic() {};
    double getValue(double x, double y);
private:
    const std::vector<TeamPlannerRobot>& m_Team;
    const double m_dScaling;
};

// ----------------------------------------------------------------------------------------
class InfluencePreviousAssignedPositionsHeuristic : public GridHeuristic
{
public:
    InfluencePreviousAssignedPositionsHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            const std::vector<TeamPlannerRobot>& Team, double dScaling, dynamic_role_e dynamic_role);
    virtual ~InfluencePreviousAssignedPositionsHeuristic() {};
    double getValue(double x, double y);
private:
    const std::vector<TeamPlannerRobot>& m_Team;
    const double m_dScaling;
    dynamic_role_e m_dynamic_role;
};

// ----------------------------------------------------------------------------------------
class ShootOnGoalHeuristic : public GridHeuristic
{
public:
    ShootOnGoalHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            const std::vector<TeamPlannerRobot>& Team,
            const std::vector<TeamPlannerOpponent>& Opponents,
            const FieldConfig& fieldConfig,
            const ball_pickup_position_t& ball_pickup_position);
    virtual ~ShootOnGoalHeuristic() {};
    double getValue(double x, double y);
private:
    const std::vector<TeamPlannerRobot>& m_Team;
    const std::vector<TeamPlannerOpponent>& m_Opponents;
    const Geometry::Point m_opponentGoal;
    const double m_rightPole_x;
    const double m_leftPole_x;
    const double m_robotRadius;
    ball_pickup_position_t m_ball_pickup_position;
    const double m_min_own_y_pos;
};

// ----------------------------------------------------------------------------------------
class PassHeuristic : public GridHeuristic
{
public:
    PassHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            const std::vector<TeamPlannerRobot>& Team,
            const std::vector<TeamPlannerOpponent>& Opponents,
            const FieldConfig& fieldConfig,
            const ball_pickup_position_t& ball_pickup_position,
            const TeamPlannerParameters& parameters);
    virtual ~PassHeuristic() {};
    double getValue(double x, double y);
private:
    const std::vector<TeamPlannerRobot>& m_Team;
    std::vector<Geometry::Position> m_Opponents;
    const double m_robotRadius;
    ball_pickup_position_t m_ball_pickup_position;
    const double m_interceptionChanceStartDistance;
    const double m_interceptionChanceIncreasePerMeter;
    const double m_interceptionChancePenaltyFactor;
    double m_numberOfFieldPlayers;
};

// ----------------------------------------------------------------------------------------
class StayAwayFromOpponentsHeuristic : public GridHeuristic
{
public:
    StayAwayFromOpponentsHeuristic(const char *id, double weight, PlannerGridInfoData& pgid,
            const Geometry::Position& ballPlayerPos,
            const Geometry::Position& ball,
            const std::vector<TeamPlannerOpponent>& Opponents,
            const double radius);
    virtual ~StayAwayFromOpponentsHeuristic() {};
    double getValue(double x, double y);
private:
    const Geometry::Position& m_ballPlayerPos;
    const Geometry::Point m_ball;
    const std::vector<TeamPlannerOpponent>& m_Opponents;
    const double m_radius;
    double m_angle_ball_ballplayer_min;
    double m_angle_ball_ballplayer_max;
};



} // namespace trs


#endif /* PLANNER_GRID_GRIDHEURISTIC_H  */