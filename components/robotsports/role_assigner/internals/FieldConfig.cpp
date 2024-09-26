/**
 *  @file
 *  @brief   Represents field dimension for the planner
 *  @curator Jürge van Eijck
 */

#include "FieldConfig.hpp"

#include <cmath>
#include <iostream>
#include <ostream>
#include <sstream>

using namespace std;
const int NRPLAYERS = 5; // max players per team

namespace MRA {

FieldConfig::FieldConfig(const FieldParameters& fp) : m_fp(fp) {
}

FieldParameters FieldConfig::getFieldParameters() const {
    return m_fp;
}

double FieldConfig::getBallRadius() const {
    return m_ball_radius;
}

double FieldConfig::getRobotSize()  const {
    return m_robot_size;
}

double FieldConfig::getRobotRadius()  const {
    return m_robot_size * 0.5;
}


bool FieldConfig::isPenaltyAreaPresent() const {
    return m_penalty_area_present;
}


double FieldConfig::getFieldLength() const {
    return m_fp.SLM.A;
}

double FieldConfig::getFieldWidth()  const {
    return m_fp.SLM.B;
}

double FieldConfig::getFieldMargin()  const {
    return m_fp.SLM.L;
}

double FieldConfig::getPenaltySpotToBackline() const {
    return m_fp.SLM.Q;
}

double FieldConfig::getFieldMarkingsWidth() const {
    return m_fp.SLM.K;
}

double FieldConfig::getCornerCircleDiameter() const {
    return m_fp.SLM.G;
}

double FieldConfig::getCenterCirleDiameter()  const {
    return m_fp.SLM.H;
}

double FieldConfig::getGoalAreaWidth() const {
    return m_fp.SLM.D;
}

double FieldConfig::getGoalAreaLength() const {
    return m_fp.SLM.F;
}

double FieldConfig::getPenaltyAreaWidth() const {
    return m_fp.SLM.C;
}

double FieldConfig::getPenaltyAreaLength() const {
    return m_fp.SLM.E;
}

double FieldConfig::getGoalLength() const {
    return m_fp.goal_length;
}

double FieldConfig::getGoalWidth() const {
    return m_fp.goal_width;
}

double FieldConfig::getParkingAreaWidth() const {
    return m_parking_area_width;
}

double FieldConfig::getParkingAreaLength() const {
    return m_parking_area_length;
}

double FieldConfig::getParkingDistanceBetweenPlayers() const {
    return m_parking_distance_between_robots;
}

double FieldConfig::getParkingDistanceToLine() const {
    return m_parking_distance_to_line;
}


double FieldConfig::getFullFieldLength() const {
    return getFieldLength() + 2 * getFieldMargin();
}

double FieldConfig::getFullFieldWidth()  const {
    return getFieldWidth()  + 2 * getFieldMargin();
}



double FieldConfig::getCenterCirleRadius()  const {
    return getCenterCirleDiameter() * 0.5;
}


double FieldConfig::getMaxFullFieldX() const  {
    return getMaxFieldX() + getFieldMargin();
}

double FieldConfig::getMaxReachableFieldX() const {
    return max(getMaxFullFieldX() - getRobotRadius(), getMaxFieldX()); // max allowed position in field
}

double FieldConfig::getMaxFieldX()  const {
    return getFieldWidth() * 0.5;
}

double FieldConfig::getMaxFullFieldY()  const {
    return getMaxFieldY() + getFieldMargin();
}

double FieldConfig::getMaxReachableFieldY() const  {
    return max(getMaxFullFieldY() - getRobotRadius(), getMaxFieldY()); // max allowed position in field
}

double FieldConfig::getMaxFieldY()  const {
    return getFieldLength() * 0.5;
}

double FieldConfig::getTopPenaltyAreaY()  const {
    return getMaxFieldY() - getPenaltyAreaLength();
}

double FieldConfig::getMaxPossibleFieldDistance() const {
    return hypot(getFullFieldWidth(),getFullFieldLength());
}



MRA::Geometry::Point FieldConfig::getOpponentGoal() const {
    MRA::Geometry::Point opponentGoal = MRA::Geometry::Point(0, getMaxFieldY());
    return opponentGoal;
}

MRA::Geometry::Point FieldConfig::getOwnGoal() const {
    MRA::Geometry::Point ownGoal = MRA::Geometry::Point(0, -getMaxFieldY());
    return ownGoal;
}




bool FieldConfig::isInField(const MRA::Geometry::Point& r_pos, double margin) const {
    return FieldConfig::isInField(r_pos.x, r_pos.y, margin);
}

bool FieldConfig::isInField(double x, double y, double margin) const {
    /* check if point is in the field */
    bool inField = (fabs(y) <= (0.5*getFieldLength())-margin) && (fabs(x) <= (0.5*getFieldWidth())-margin);
    return inField;
}

bool FieldConfig::isInReachableField(const MRA::Geometry::Point& r_pos) const {
    return isInReachableField(r_pos.x, r_pos.y);
}

bool FieldConfig::isInReachableField(double x, double y) const {
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


bool FieldConfig::isInOpponentGoalArea(double x, double y) const {
    bool inGoalArea = isInOwnGoalArea(x, -y); // should not be in own goal if y is inverted. (prevent double formula)
    return inGoalArea;
}

bool FieldConfig::isInOwnGoalArea(double x, double y)  const {
    bool inGoalArea = false;
    /* check if point is in the goal */
    double field_length = getFieldWidth();
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


bool FieldConfig::isInOpponentPenaltyArea(double x, double y) const {
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


bool FieldConfig::isInOwnPenaltyArea(double x, double y, double margin) const {
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

bool FieldConfig::isInOwnPenaltyArea(double x, double y) const {
    return FieldConfig::isInOwnPenaltyArea(x, y, 0.0);
}


string FieldConfig::toString() const {
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
