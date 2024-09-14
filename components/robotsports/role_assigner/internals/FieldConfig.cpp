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

double FieldConfig::getFieldLength() const {
    return m_field_length;
}

double FieldConfig::getFieldWidth()  const {
    return m_field_width;
}

double FieldConfig::getFullFieldLength() const {
    return getFieldLength() + 2 * getFieldMargin();
}

double FieldConfig::getFullFieldWidth()  const {
    return getFieldWidth()  + 2 * getFieldMargin();
}

double FieldConfig::getFieldMargin()  const {
    return m_field_margin;
}

double FieldConfig::getCenterCirleDiameter()  const {
    return m_center_circle_diameter;
}

double FieldConfig::getCenterCirleRadius()  const {
    return getCenterCirleDiameter() * 0.5;
}

double FieldConfig::getRobotSize()  const {
    return m_robot_size;
}

double FieldConfig::getRobotRadius()  const {
    return m_robot_size * 0.5;
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

double FieldConfig::getBallRadius() const {
    return m_ball_radius;
}

double FieldConfig::getGoalAreaWidth() const {
    return m_goal_area_width;
}

double FieldConfig::getGoalAreaLength() const {
    return m_goal_area_length;
}

bool FieldConfig::isPenaltyAreaPresent() const {
    return m_penalty_area_present;
}

double FieldConfig::getPenaltyAreaWidth() const {
    return m_penalty_area_width;
}

double FieldConfig::getPenaltyAreaLength() const {
    return m_penalty_area_length;
}

double FieldConfig::getGoalLength() const {
    return m_goal_length;
}

double FieldConfig::getGoalWidth() const {
    return m_goal_width;
}

MRA::Geometry::Point FieldConfig::getOpponentGoal() const {
    MRA::Geometry::Point opponentGoal = MRA::Geometry::Point(0, getMaxFieldY());
    return opponentGoal;
}

MRA::Geometry::Point FieldConfig::getOwnGoal() const {
    MRA::Geometry::Point ownGoal = MRA::Geometry::Point(0, -getMaxFieldY());
    return ownGoal;
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

double FieldConfig::getPenaltySpotToBackline() const {
    return m_penalty_spot_to_backline;
}

double FieldConfig::getFieldMarkingsWidth() const {
    return m_field_markings_width;
}

double FieldConfig::getCornerCircleDiameter() const {
    return m_corner_circle_diameter;
}

FieldConfig FillDefaultFieldConfig() {
    double robot_size = 0.5;
    double ball_radius = 0.11;

    double field_length =  18.0;
    double field_width = 12.0;
    double field_margin = 1.5;
    double goal_width = 2.0;
    double goal_length = 0.5;
    double center_circle_diameter = 4.0;
    double goal_area_width = 3.5;
    double goal_area_length = 0.75;
    bool penalty_area_present = true;
    double penalty_area_width = 6.5;
    double penalty_area_length = 2.25;
    double parking_distance_between_robots = 1.0;
    double parking_distance_to_line = 0.5;
    double parking_area_width =  0.5;
    double parking_area_length = (NRPLAYERS-1)*parking_distance_between_robots + robot_size;

    double field_markings_width = 0.125;
    double field_markings_width_internal = 0.125;
    double penalty_spot_to_backline = 3.0;
    return FieldConfig(field_length, field_width, field_margin, goal_width, goal_length,
            center_circle_diameter, goal_area_width, goal_area_length, penalty_area_present, penalty_area_width,
            penalty_area_length, parking_area_width, parking_area_length, parking_distance_between_robots, parking_distance_to_line, robot_size,
            ball_radius, field_markings_width, field_markings_width_internal, penalty_spot_to_backline);
}


FieldConfig::FieldConfig(double field_length, double field_width, double field_margin, double goal_width, double goal_length,
        double center_circle_diameter, double goal_area_width, double goal_area_length, bool penalty_area_present, double penalty_area_width,
        double penalty_area_length, double parking_area_width, double parking_area_length,
        double parking_distance_between_robots, double parking_distance_to_line,
        double robot_size, double ball_radius, double field_markings_width, double corner_circle_diameter,
        double penalty_spot_to_backline)
{
    setConfig(field_length, field_width, field_margin, goal_width, goal_length,
            center_circle_diameter, goal_area_width, goal_area_length, penalty_area_present, penalty_area_width,
            penalty_area_length, parking_area_width, parking_area_length,
            parking_distance_between_robots, parking_distance_to_line,
            robot_size, ball_radius, field_markings_width, corner_circle_diameter,
            penalty_spot_to_backline);
}


void FieldConfig::setConfig(double field_length, double field_width, double field_margin, double goal_width, double goal_length,
        double center_circle_diameter, double goal_area_width, double goal_area_length, bool penalty_area_present, double penalty_area_width,
        double penalty_area_length, double parking_area_width, double parking_area_length,
        double parking_distance_between_robots, double parking_distance_to_line,
        double robot_size, double ball_radius, double field_markings_width, double corner_circle_diameter,
        double penalty_spot_to_backline)
{

    m_field_length = field_length;
    m_field_width  = field_width;
    m_field_margin = field_margin;
    m_goal_width = goal_width;
    m_goal_length = goal_length;
    m_center_circle_diameter = center_circle_diameter;
    m_goal_area_width = goal_area_width;
    m_goal_area_length = goal_area_length;
    m_penalty_area_present = penalty_area_present;
    m_penalty_area_width = penalty_area_width;
    m_penalty_area_length = penalty_area_length;
    m_robot_size = robot_size;
    m_ball_radius = ball_radius;
    m_field_markings_width = field_markings_width;
    m_parking_area_width = parking_area_width;
    m_parking_area_length = parking_area_length;
    m_parking_distance_between_robots = parking_distance_between_robots;
    m_parking_distance_to_line = parking_distance_to_line;
    m_penalty_spot_to_backline = penalty_spot_to_backline;
    m_corner_circle_diameter = corner_circle_diameter;
    m_minimum_distance_to_goal_area = 0.25;
}

bool FieldConfig::isInField(const MRA::Geometry::Point& r_pos, double margin) const {
    return FieldConfig::isInField(r_pos.x, r_pos.y, margin);
}

bool FieldConfig::isInField(double x, double y, double margin) const {
    /* check if point is in the field */
    bool inField = (fabs(y) <= (0.5*m_field_length)-margin) && (fabs(x) <= (0.5*m_field_width)-margin);
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
    if ((y <= -(m_field_length/2)) && (y >= -(m_field_length/2) - (m_goal_length + m_minimum_distance_to_goal_area)) &&
        (fabs(x) <= (m_goal_width/2 + m_minimum_distance_to_goal_area))) {
        inGoalArea = true;
    }
    /* check if point is in the goal-area */
    else if ((y <= -(m_field_length/2) + m_goal_area_length + m_minimum_distance_to_goal_area) &&
             (y >= -(m_field_length/2)) &&
                (fabs(x) <= (m_goal_area_width/2 + m_minimum_distance_to_goal_area))) {
        inGoalArea = true;
    }
    return inGoalArea;
}


bool FieldConfig::isInOpponentPenaltyArea(double x, double y) const {
    bool inPenaltyArea = false;
    /* check if point is in the penalty-area */
    if ((y >= (m_field_length/2) - m_penalty_area_length) &&
        (y <= (m_field_length/2)) &&
           (fabs(x) <= m_penalty_area_width/2)) {
        inPenaltyArea = true;
    }
    return inPenaltyArea;
}


bool FieldConfig::isInOwnPenaltyArea(double x, double y, double margin) const {
    bool inPenaltyArea = false;
    /* check if point is in the penalty-area */
    if ((y <= -(m_field_length/2) + m_penalty_area_length + margin) &&
        (y >= -(m_field_length/2)) &&
           (fabs(x) <= m_penalty_area_width/2)+margin) {
        inPenaltyArea = true;
    }
    return inPenaltyArea;
}

bool FieldConfig::isInOwnPenaltyArea(double x, double y) const {
    return FieldConfig::isInOwnPenaltyArea(x, y, 0.0);
}


string FieldConfig::toString() const {
    std::stringstream buffer;
    buffer << "FIELD_LENGTH = " << m_field_length << endl;
    buffer << "FIELD_WIDTH = " << m_field_width << endl;
    buffer << "FIELD_MARGIN = " << m_field_margin << endl;
    buffer << "GOAL_WIDTH = " << m_goal_width << endl;
    buffer << "GOAL_LENGTH = " << m_goal_length << endl;
    buffer << "CENTER_CIRCLE_DIAMETER = " << m_center_circle_diameter << endl;
    buffer << "GOAL_AREA_WIDTH = " << m_goal_area_width << endl;
    buffer << "GOAL_AREA_LENGTH = " << m_goal_area_length << endl;
    buffer << "PENALTY_AREA_PRESENT = " << m_penalty_area_present << endl;
    buffer << "PENALTY_AREA_WIDTH = " << m_penalty_area_width << endl;
    buffer << "PENALTY_AREA_LENGTH = " << m_penalty_area_length << endl;
    buffer << "ROBOTSIZE = " << m_robot_size << endl;
    buffer << "BALL_RADIUS = " << m_ball_radius << endl;
    buffer << "FIELD_MARKINGS_WIDTH = " << m_field_markings_width << endl;
    buffer << "PARKING_AREA_WIDTH = " << m_parking_area_width << endl;
    buffer << "PARKING_AREA_LENGTH = " << m_parking_area_length << endl;
    buffer << "PARKING_DISTANCE_BETWEEN_ROBOTS = " << m_parking_distance_between_robots << endl;
    buffer << "PARKING_DISTANCE_TO_LINE = " << m_parking_distance_to_line << endl;


    return buffer.str();

}

}
