/**
 *  @file
 *  @brief   Represents field dimension for the planner
 *  @curator Jürge van Eijck
 */

#ifndef FIELD_CONFIG_H
#define FIELD_CONFIG_H 1
#include <string>
#include "geometry.hpp"

namespace MRA {

class FieldConfig {

public:
    FieldConfig() {};
	FieldConfig(double field_length, double field_width, double field_margin, double goal_width, double goal_length,
			double center_circle_diameter, double goal_area_width, double goal_area_length, bool penalty_area_present, double penalty_area_width,
			double penalty_area_length, double parking_area_width, double parking_area_length,
			double parking_distance_between_robots, double parking_distance_to_line,
			double robot_size, double ball_radius, double field_markings_width, double field_markings_width_internal,
			double penalty_spot_to_backline, bool is_mobile_field);

	/* check if point is in the playing field and is minimal the given margin from the edge of the playing field. A positive margin is within the playing field.  */
	bool isInField(const MRA::Geometry::Point& r_pos, double margin) const;
	bool isInField(double x, double y, double margin) const;

	/* Check if position is within reachable field. This is a location where player can be including the safety area around the field and can freely move.
	 * Reachable means: position where the player has still space for turning while player is within the playing field
	 * or safety area around the field and not behind own goal.
	 */
	bool isInReachableField(const MRA::Geometry::Point& r_pos) const;
	bool isInReachableField(double x, double y) const;

	bool isInOpponentGoalArea(double x, double y) const;
	bool isInOwnGoalArea(double x, double y) const;
	bool isInOpponentPenaltyArea(double x, double y) const;
	bool isInOwnPenaltyArea(double x, double y) const;
	bool isInOwnPenaltyArea(double x, double y, double margin) const;

	std::string toString() const;

	double getFieldLength() const;
	double getFieldWidth()  const;
	double getFullFieldLength() const;
	double getFullFieldWidth()  const;
	double getFieldMargin()  const;
	double getFieldMarkingsWidth() const;
	double getCenterCirleDiameter() const;
	double getCenterCirleRadius() const;
	double getRobotSize() const;
	double getRobotRadius() const;

	double getMaxFullFieldX() const;
	double getMaxReachableFieldX() const;
	double getMaxFieldX() const;

	double getMaxFullFieldY() const;
	double getMaxReachableFieldY() const;
	double getMaxFieldY()  const;
	double getTopPenaltyAreaY() const;
	double getMaxPossibleFieldDistance() const;

	double getBallRadius() const;
	double getGoalAreaWidth() const;
	double getGoalAreaLength() const;
	bool isPenaltyAreaPresent() const;
	double getPenaltyAreaWidth() const;
	double getPenaltyAreaLength() const;
	double getGoalLength() const;
	double getGoalWidth() const;

	MRA::Geometry::Point getOpponentGoal() const;
	MRA::Geometry::Point getOwnGoal() const;


	double getParkingAreaWidth() const;
	double getParkingAreaLength() const;
	double getParkingDistanceBetweenPlayers() const;
	double getParkingDistanceToLine() const;

	double getPenaltySpotToBackline() const;
	double getCornerCircleDiameter() const;

public: // data is public, to avoid vision updates. In the future: data is private, vision code is using functions. TODO
	//"FieldLength: Length of the playing field [m]",
	double FIELD_LENGTH;

	//"FieldWidth: Width of the playing field [m]"
	double FIELD_WIDTH;

	//"FieldMargin: Margin [m] around the field"
	double FIELD_MARGIN;

	//"GoalWidth: Width [m] of the goal", Double.class, 2.0);
	double GOAL_WIDTH;

	//"GoalLength: Length [m] of the goal"
	double GOAL_LENGTH;

	// "CenterCircleDiameter: Diameter [m] of the center circle",
	double CENTER_CIRCLE_DIAMETER;

	// "GoalAreaWidth: Width [m] of goal area"
	double GOAL_AREA_WIDTH;

	//"GoalAreaLength: Length [m] of goal area"
	double GOAL_AREA_LENGTH;

	// "PenaltyAreaPresent: does field have a penalty area?"
	bool PENALTY_AREA_PRESENT;

	// "GoalAreaWidth: Width [m] of goal area"
	double PENALTY_AREA_WIDTH;

	//"GoalAreaLength: Length [m] of goal area"
	double PENALTY_AREA_LENGTH;

	// max size of the robot (in 1 dim) [m]
	double ROBOTSIZE;

	// radius of the ball [m]
	double BALL_RADIUS;

	double FIELD_MARKINGS_WIDTH; // [m]

	double FIELD_MARKINGS_WIDTH_INTERNAL; // [m]

	double PARKING_AREA_WIDTH; // [m] area where players will be parked

	double PARKING_AREA_LENGTH; // [m] area where players will be parked

	double PARKING_DISTANCE_BETWEEN_ROBOTS; // [m] between the parked robots (between the middle of the robots)

	double PARKING_DISTANCE_TO_LINE; // [m] to line for parking : + is outside. 0 is on the line

	double PENALTY_SPOT_TO_BACKLINE;

	double MIN_DIST_TO_GOAL_AREA;

	double CORNER_CIRCLE_DIAMETER;
private:
	bool m_is_mobile_field;
};

	FieldConfig FillDefaultFieldConfig(); // only for off-line tests (automatic tests)

} //namespace

#endif
