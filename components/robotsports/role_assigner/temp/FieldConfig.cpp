/**
 *  @file
 *  @brief   Represents field dimension for the planner
 *  @curator Jürge van Eijck
 */

#include "FieldConfig.h"
#include <cmath>
#include <iostream>
#include <ostream>
#include <sstream>

using namespace std;
const int NRPLAYERS = 5; // max players per team

namespace MRA {

double FieldConfig::getFieldLength() const {
	return FIELD_LENGTH;
}

double FieldConfig::getFieldWidth()  const {
	return FIELD_WIDTH;
}

double FieldConfig::getFullFieldLength() const {
	return getFieldLength() + 2 * getFieldMargin();
}

double FieldConfig::getFullFieldWidth()  const {
	return getFieldWidth()  + 2 * getFieldMargin();
}

double FieldConfig::getFieldMargin()  const {
	return FIELD_MARGIN;
}

double FieldConfig::getCenterCirleDiameter()  const {
	return CENTER_CIRCLE_DIAMETER;
}

double FieldConfig::getCenterCirleRadius()  const {
	return getCenterCirleDiameter() * 0.5;
}

double FieldConfig::getRobotSize()  const {
	return ROBOTSIZE;
}

double FieldConfig::getRobotRadius()  const {
	return ROBOTSIZE * 0.5;
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
	return BALL_RADIUS;
}

double FieldConfig::getGoalAreaWidth() const {
	return GOAL_AREA_WIDTH;
}

double FieldConfig::getGoalAreaLength() const {
	return GOAL_AREA_LENGTH;
}

bool FieldConfig::isPenaltyAreaPresent() const {
	return PENALTY_AREA_PRESENT;
}

double FieldConfig::getPenaltyAreaWidth() const {
	return PENALTY_AREA_WIDTH;
}

double FieldConfig::getPenaltyAreaLength() const {
	return PENALTY_AREA_LENGTH;
}

double FieldConfig::getGoalLength() const {
	return GOAL_LENGTH;
}

double FieldConfig::getGoalWidth() const {
	return GOAL_WIDTH;
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
	return PARKING_AREA_WIDTH;
}

double FieldConfig::getParkingAreaLength() const {
	return PARKING_AREA_LENGTH;
}

double FieldConfig::getParkingDistanceBetweenPlayers() const {
	return PARKING_DISTANCE_BETWEEN_ROBOTS;
}

double FieldConfig::getParkingDistanceToLine() const {
	return PARKING_DISTANCE_TO_LINE;
}

double FieldConfig::getPenaltySpotToBackline() const {
	return PENALTY_SPOT_TO_BACKLINE;
}

double FieldConfig::getFieldMarkingsWidth() const {
	return FIELD_MARKINGS_WIDTH;
}

double FieldConfig::getCornerCircleDiameter() const {
	return CORNER_CIRCLE_DIAMETER;
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
	bool is_mobile_field = false;
	return FieldConfig(field_length, field_width, field_margin, goal_width, goal_length,
			center_circle_diameter, goal_area_width, goal_area_length, penalty_area_present, penalty_area_width,
			penalty_area_length, parking_area_width, parking_area_length, parking_distance_between_robots, parking_distance_to_line, robot_size,
			ball_radius, field_markings_width, field_markings_width_internal, penalty_spot_to_backline, is_mobile_field);
}

FieldConfig::FieldConfig(double field_length, double field_width, double field_margin, double goal_width, double goal_length,
		double center_circle_diameter, double goal_area_width, double goal_area_length, bool penalty_area_present, double penalty_area_width,
		double penalty_area_length, double parking_area_width, double parking_area_length,
		double parking_distance_between_robots, double parking_distance_to_line,
		double robot_size, double ball_radius, double field_markings_width, double field_markings_width_internal,
		double penalty_spot_to_backline, bool is_mobile_field) :
		m_is_mobile_field(is_mobile_field)
{

	FIELD_LENGTH = field_length;
	FIELD_WIDTH  = field_width;
	FIELD_MARGIN = field_margin;
	GOAL_WIDTH = goal_width;
	GOAL_LENGTH = goal_length;
	CENTER_CIRCLE_DIAMETER = center_circle_diameter;
	GOAL_AREA_WIDTH = goal_area_width;
	GOAL_AREA_LENGTH = goal_area_length;
	PENALTY_AREA_PRESENT = penalty_area_present;
	PENALTY_AREA_WIDTH = penalty_area_width;
	PENALTY_AREA_LENGTH = penalty_area_length;
	ROBOTSIZE = robot_size;
	BALL_RADIUS = ball_radius;
	FIELD_MARKINGS_WIDTH = field_markings_width;
	FIELD_MARKINGS_WIDTH_INTERNAL = field_markings_width_internal;
	PARKING_AREA_WIDTH = parking_area_width;
	PARKING_AREA_LENGTH = parking_area_length;
	PARKING_DISTANCE_BETWEEN_ROBOTS = parking_distance_between_robots;
	PARKING_DISTANCE_TO_LINE = parking_distance_to_line;
	PENALTY_SPOT_TO_BACKLINE = penalty_spot_to_backline;
	MIN_DIST_TO_GOAL_AREA = 0.25;
	CORNER_CIRCLE_DIAMETER = 0.75;
}

bool FieldConfig::isInField(const MRA::Geometry::Point& r_pos, double margin) const {
	return FieldConfig::isInField(r_pos.x, r_pos.y, margin);
}

bool FieldConfig::isInField(double x, double y, double margin) const {
	/* check if point is in the field */
	bool inField = (fabs(y) <= (0.5*FIELD_LENGTH)-margin) && (fabs(x) <= (0.5*FIELD_WIDTH)-margin);
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
	if ((y <= -(FIELD_LENGTH/2)) && (y >= -(FIELD_LENGTH/2) - (GOAL_LENGTH + MIN_DIST_TO_GOAL_AREA)) &&
	    (fabs(x) <= (GOAL_WIDTH/2 + MIN_DIST_TO_GOAL_AREA))) {
		inGoalArea = true;
	}
	/* check if point is in the goal-area */
	else if ((y <= -(FIELD_LENGTH/2) + GOAL_AREA_LENGTH + MIN_DIST_TO_GOAL_AREA) &&
			 (y >= -(FIELD_LENGTH/2)) &&
   	         (fabs(x) <= (GOAL_AREA_WIDTH/2 + MIN_DIST_TO_GOAL_AREA))) {
		inGoalArea = true;
	}
	return inGoalArea;
}


bool FieldConfig::isInOpponentPenaltyArea(double x, double y) const {
	bool inPenaltyArea = false;
	/* check if point is in the penalty-area */
	if ((y >= (FIELD_LENGTH/2) - PENALTY_AREA_LENGTH) &&
	    (y <= (FIELD_LENGTH/2)) &&
   	    (fabs(x) <= PENALTY_AREA_WIDTH/2)) {
		inPenaltyArea = true;
	}
	return inPenaltyArea;
}


bool FieldConfig::isInOwnPenaltyArea(double x, double y, double margin) const {
	bool inPenaltyArea = false;
	/* check if point is in the penalty-area */
	if ((y <= -(FIELD_LENGTH/2) + PENALTY_AREA_LENGTH + margin) &&
	    (y >= -(FIELD_LENGTH/2)) &&
   	    (fabs(x) <= PENALTY_AREA_WIDTH/2)+margin) {
		inPenaltyArea = true;
	}
	return inPenaltyArea;
}

bool FieldConfig::isInOwnPenaltyArea(double x, double y) const {
	return FieldConfig::isInOwnPenaltyArea(x, y, 0.0);
}


string FieldConfig::toString() const {
	std::stringstream buffer;
	buffer << "FIELD_LENGTH = " << FIELD_LENGTH << endl;
	buffer << "FIELD_WIDTH = " << FIELD_WIDTH << endl;
	buffer << "FIELD_MARGIN = " << FIELD_MARGIN << endl;
	buffer << "GOAL_WIDTH = " << GOAL_WIDTH << endl;
	buffer << "GOAL_LENGTH = " << GOAL_LENGTH << endl;
	buffer << "CENTER_CIRCLE_DIAMETER = " << CENTER_CIRCLE_DIAMETER << endl;
	buffer << "GOAL_AREA_WIDTH = " << GOAL_AREA_WIDTH << endl;
	buffer << "GOAL_AREA_LENGTH = " << GOAL_AREA_LENGTH << endl;
	buffer << "PENALTY_AREA_PRESENT = " << PENALTY_AREA_PRESENT << endl;
	buffer << "PENALTY_AREA_WIDTH = " << PENALTY_AREA_WIDTH << endl;
	buffer << "PENALTY_AREA_LENGTH = " << PENALTY_AREA_LENGTH << endl;
	buffer << "ROBOTSIZE = " << ROBOTSIZE << endl;
	buffer << "BALL_RADIUS = " << BALL_RADIUS << endl;
	buffer << "FIELD_MARKINGS_WIDTH = " << FIELD_MARKINGS_WIDTH << endl;
	buffer << "FIELD_MARKINGS_WIDTH_INTERNAL = " << FIELD_MARKINGS_WIDTH_INTERNAL << endl;
	buffer << "PARKING_AREA_WIDTH = " << PARKING_AREA_WIDTH << endl;
	buffer << "PARKING_AREA_LENGTH = " << PARKING_AREA_LENGTH << endl;
	buffer << "PARKING_DISTANCE_BETWEEN_ROBOTS = " << PARKING_DISTANCE_BETWEEN_ROBOTS << endl;
	buffer << "PARKING_DISTANCE_TO_LINE = " << PARKING_DISTANCE_TO_LINE << endl;


	return buffer.str();

}

}
