/**
 *  @file
 *  @brief   Represents field dimension for the planner
 *  @curator Jürge van Eijck
 */

#include "Environment.hpp"

#include <cmath>
#include <iostream>
#include <ostream>
#include <sstream>

using namespace std;
const int NRPLAYERS = 5; // max players per team

namespace MRA {

Environment::Environment(const EnvironmentParameters& env_params) : m_env_params(env_params) {
}

double Environment::getBallRadius() const {
    return m_env_params.ball_radius;
}

double Environment::getRobotSize()  const {
    return m_env_params.robot_size;
}

double Environment::getRobotRadius()  const {
    return m_env_params.robot_size * 0.5;
}


bool Environment::isPenaltyAreaPresent() const {
    return m_env_params.penalty_area_present;
}


double Environment::getFieldLength() const {
    return m_env_params.SLM.A;
}

double Environment::getFieldWidth()  const {
    return m_env_params.SLM.B;
}

double Environment::getFieldMargin()  const {
    return m_env_params.SLM.L;
}

double Environment::getPenaltySpotToBackline() const {
    return m_env_params.SLM.Q;
}

double Environment::getFieldMarkingsWidth() const {
    return m_env_params.SLM.K;
}

double Environment::getCornerCircleDiameter() const {
    return m_env_params.SLM.G;
}

double Environment::getCenterCirleDiameter()  const {
    return m_env_params.SLM.H;
}

double Environment::getGoalAreaWidth() const {
    return m_env_params.SLM.D;
}

double Environment::getGoalAreaLength() const {
    return m_env_params.SLM.F;
}

double Environment::getPenaltyAreaWidth() const {
    return m_env_params.SLM.C;
}

double Environment::getPenaltyAreaLength() const {
    return m_env_params.SLM.E;
}

double Environment::getGoalLength() const {
    return m_env_params.goal_length;
}

double Environment::getGoalWidth() const {
    return m_env_params.goal_width;
}

double Environment::getParkingAreaWidth() const {
    return m_env_params.parking_area_width;
}

double Environment::getParkingAreaLength() const {
    return m_env_params.parking_area_length;
}

double Environment::getParkingDistanceBetweenPlayers() const {
    return m_env_params.parking_distance_between_robots;
}

double Environment::getParkingDistanceToLine() const {
    return m_env_params.parking_distance_to_line;
}

double Environment::getFullFieldLength() const {
    return getFieldLength() + 2 * getFieldMargin();
}

double Environment::getFullFieldWidth()  const {
    return getFieldWidth()  + 2 * getFieldMargin();
}



double Environment::getCenterCirleRadius()  const {
    return getCenterCirleDiameter() * 0.5;
}


double Environment::getMaxFullFieldX() const  {
    return getMaxFieldX() + getFieldMargin();
}

double Environment::getMaxReachableFieldX() const {
    return max(getMaxFullFieldX() - getRobotRadius(), getMaxFieldX()); // max allowed position in field
}

double Environment::getMaxFieldX()  const {
    return getFieldWidth() * 0.5;
}

double Environment::getMaxFullFieldY()  const {
    return getMaxFieldY() + getFieldMargin();
}

double Environment::getMaxReachableFieldY() const  {
    return max(getMaxFullFieldY() - getRobotRadius(), getMaxFieldY()); // max allowed position in field
}

double Environment::getMaxFieldY()  const {
    return getFieldLength() * 0.5;
}

double Environment::getTopPenaltyAreaY()  const {
    return getMaxFieldY() - getPenaltyAreaLength();
}

double Environment::getMaxPossibleFieldDistance() const {
    return hypot(getFullFieldWidth(),getFullFieldLength());
}


MRA::Geometry::Point Environment::getOpponentGoal() const {
    MRA::Geometry::Point opponentGoal = MRA::Geometry::Point(0, getMaxFieldY());
    return opponentGoal;
}

MRA::Geometry::Point Environment::getOwnGoal() const {
    MRA::Geometry::Point ownGoal = MRA::Geometry::Point(0, -getMaxFieldY());
    return ownGoal;
}

bool Environment::isInField(const MRA::Geometry::Point& r_pos, double margin) const {
    return Environment::isInField(r_pos.x, r_pos.y, margin);
}

bool Environment::isInField(double x, double y, double margin) const {
    /* check if point is in the field */
    bool inField = (fabs(y) <= (0.5*getFieldLength())-margin) && (fabs(x) <= (0.5*getFieldWidth())-margin);
    return inField;
}

bool Environment::isInReachableField(const MRA::Geometry::Point& r_pos) const {
    return isInReachableField(r_pos.x, r_pos.y);
}

bool Environment::isInReachableField(double x, double y) const {
    /* check if point is in the reachable part of the  field */
    const double PLAYER_TO_BORDER_MARGIN = 0.05; // 5 cm space for turning etc.
    bool inField = (fabs(y) <= getMaxReachableFieldY() - PLAYER_TO_BORDER_MARGIN) && (fabs(x) <= getMaxReachableFieldX() - PLAYER_TO_BORDER_MARGIN);

    // Check if player is not behind own goal (not |x| < goal-area width and |y| > maxFieldY)
    if (inField && fabs(x) < (getGoalAreaWidth()*0.5)) {
        if (fabs(y) > getMaxFieldY()) {
            inField = false;
        }
    }
    return inField;
}


bool Environment::isInOpponentGoalArea(double x, double y) const {
    bool inGoalArea = isInOwnGoalArea(x, -y); // should not be in own goal if y is inverted. (prevent double formula)
    return inGoalArea;
}

bool Environment::isInOwnGoalArea(double x, double y)  const {
    bool inGoalArea = false;
    /* check if point is in the goal */
    double field_length = getFieldLength();
    double minimum_distance_to_goal_area = m_env_params.minimum_distance_to_goal_area;
    if ((y <= -(field_length/2)) && (y >= -(field_length/2) - (getGoalLength() + minimum_distance_to_goal_area)) &&
        (fabs(x) <= (getGoalWidth()/2 + minimum_distance_to_goal_area))) {
        inGoalArea = true;
    }
    /* check if point is in the goal-area */
    else if ((y <= -(field_length/2) + getGoalAreaLength() + minimum_distance_to_goal_area) &&
             (y >= -(field_length/2)) &&
                (fabs(x) <= (getGoalAreaWidth()/2 + minimum_distance_to_goal_area))) {
        inGoalArea = true;
    }
    return inGoalArea;
}


bool Environment::isInOpponentPenaltyArea(double x, double y) const {
    bool inPenaltyArea = false;
    double field_length = getFieldLength();
    /* check if point is in the penalty-area */
    if ((y >= (field_length/2) - getPenaltyAreaLength()) &&
        (y <= (field_length/2)) &&
           (fabs(x) <= getPenaltyAreaWidth()/2)) {
        inPenaltyArea = true;
    }
    return inPenaltyArea;
}


bool Environment::isInOwnPenaltyArea(double x, double y, double margin) const {
    bool inPenaltyArea = false;
    /* check if point is in the penalty-area */
    double field_length = getFieldLength();
    if ((y <= -(field_length/2) + getPenaltyAreaLength() + margin) &&
        (y >= -(field_length/2)) &&
           (fabs(x) <= getPenaltyAreaWidth()/2)+margin) {
        inPenaltyArea = true;
    }
    return inPenaltyArea;
}

bool Environment::isInOwnPenaltyArea(double x, double y) const {
    return Environment::isInOwnPenaltyArea(x, y, 0.0);
}

void Environment::getEnvironmentParameters(EnvironmentParameters& env_params) const {
	env_params = m_env_params;
}

string Environment::toString() const {
    return m_env_params.toString();
}

Environment FillDefaultEnvironment() {
    EnvironmentParameters environment_params = {};

    environment_params.SLM.A = 18.0; // field_length
    environment_params.SLM.B = 12.0;  // field width including lines (x)
    environment_params.SLM.C = 6.5;   // penalty area width including lines (x)
    environment_params.SLM.D = 3.5;   // goal area width including lines (x)
    environment_params.SLM.E = 2.25;  // penalty area length including lines (y)
    environment_params.SLM.F = 0.75;  // goal area length including lines (y)
    environment_params.SLM.G = 0.75;  // corner circle radius including lines
    environment_params.SLM.H = 4.0;   // inner circle diameter including lines
    environment_params.SLM.I = 3.5;   // penalty mark distance (y) including line to mark center (?)
    environment_params.SLM.J = 0.15;  // [ 0.15]  penalty- and center mark diameter
    environment_params.SLM.K = 0.125; // line width
    environment_params.SLM.L = 1.5;   // field border (x) (between outer line and black safety border)
    environment_params.SLM.M = 1.0;   // Technical Team Area width (x)
    environment_params.SLM.N = 7.5;   // Technical Team Area length (y) (between safety borders)
    environment_params.SLM.O = 1.0;   // Technical Team Area ramp length (y)
    environment_params.SLM.P = 0.5;   // Technical Team Area ramp width (x)
    environment_params.SLM.Q = 3.0;   // off-center distance to restart spots (x)

    environment_params.robot_size = 0.5;
    environment_params.ball_radius = 0.11;
    environment_params.goal_width = 2.0;
    environment_params.goal_length = 0.5;
    environment_params.penalty_area_present = true;
    environment_params.parking_distance_between_robots = 1.0;
    environment_params.parking_distance_to_line = 0.5;
    environment_params.parking_area_width =  0.5;
    environment_params.parking_area_length = (NRPLAYERS-1)*environment_params.parking_distance_between_robots + environment_params.robot_size;
    environment_params.minimum_distance_to_goal_area = 0.25; 

    return Environment(environment_params);
}


} // namespace MRA
