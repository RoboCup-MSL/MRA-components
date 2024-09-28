/**
 *  @file
 *  @brief   Represents field dimension for the planner
 *  @curator Jürge van Eijck
 */

#include <cmath>
#include <iostream>
#include <ostream>
#include <sstream>
#include "Environment.hpp"

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
    double field_length = getFieldWidth();
    const double m_minimum_distance_to_goal_area = 0.25; // TODO make parameter
    if ((y <= -(field_length/2)) && (y >= -(field_length/2) - (getGoalLength() + m_minimum_distance_to_goal_area)) &&
        (fabs(x) <= (getGoalWidth()/2 + m_minimum_distance_to_goal_area))) {
        inGoalArea = true;
    }
    /* check if point is in the goal-area */
    else if ((y <= -(field_length/2) + getGoalAreaLength() + m_minimum_distance_to_goal_area) &&
             (y >= -(field_length/2)) &&
                (fabs(x) <= (getGoalAreaWidth()/2 + m_minimum_distance_to_goal_area))) {
        inGoalArea = true;
    }
    return inGoalArea;
}


bool Environment::isInOpponentPenaltyArea(double x, double y) const {
    bool inPenaltyArea = false;
    double field_length = getFieldWidth();
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
    double field_length = getFieldWidth();
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


string Environment::toString() const {
    std::stringstream buffer;
    buffer << "FIELD_LENGTH = " << getFieldLength() << endl;
    buffer << "FIELD_WIDTH = " << getFieldWidth() << endl;
    buffer << "FIELD_MARGIN = " << getFieldMargin() << endl;
    buffer << "GOAL_WIDTH = " << getGoalWidth() << endl;
    buffer << "GOAL_LENGTH = " << getGoalLength() << endl;
    buffer << "CENTER_CIRCLE_DIAMETER = " << getCenterCirleDiameter() << endl;
    buffer << "GOAL_AREA_WIDTH = " << getGoalAreaWidth() << endl;
    buffer << "GOAL_AREA_LENGTH = " << getGoalAreaLength() << endl;
    buffer << "PENALTY_AREA_PRESENT = " << isPenaltyAreaPresent() << endl;
    buffer << "PENALTY_AREA_WIDTH = " << getPenaltyAreaWidth() << endl;
    buffer << "PENALTY_AREA_LENGTH = " << getPenaltyAreaLength() << endl;
    buffer << "ROBOTSIZE = " << getRobotSize() << endl;
    buffer << "BALL_RADIUS = " << getBallRadius() << endl;
    buffer << "FIELD_MARKINGS_WIDTH = " << getFieldMarkingsWidth() << endl;
    buffer << "PARKING_AREA_WIDTH = " << getParkingAreaWidth() << endl;
    buffer << "PARKING_AREA_LENGTH = " << getParkingAreaLength() << endl;
    buffer << "PARKING_DISTANCE_BETWEEN_ROBOTS = " << getParkingDistanceBetweenPlayers() << endl;
    buffer << "PARKING_DISTANCE_TO_LINE = " << getParkingDistanceToLine() << endl;

    return buffer.str();
}

}
