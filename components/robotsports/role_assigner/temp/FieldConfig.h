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
			double robot_size, double ball_radius, double field_markings_width, double corner_circle_diameter,
			double penalty_spot_to_backline);

	void setConfig(double field_length, double field_width, double field_margin, double goal_width, double goal_length,
			double center_circle_diameter, double goal_area_width, double goal_area_length, bool penalty_area_present, double penalty_area_width,
			double penalty_area_length, double parking_area_width, double parking_area_length,
			double parking_distance_between_robots, double parking_distance_to_line,
			double robot_size, double ball_radius, double field_markings_width, double corner_circle_diameter,
			double penalty_spot_to_backline);

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

private:
	//"FieldLength: Length of the playing field [m]",
	double m_field_length;

	//"FieldWidth: Width of the playing field [m]"
	double m_field_width;

	//"FieldMargin: Margin [m] around the field"
	double m_field_margin;

	//"GoalWidth: Width [m] of the goal", Double.class, 2.0);
	double m_goal_width;

	//"GoalLength: Length [m] of the goal"
	double m_goal_length;

	// "CenterCircleDiameter: Diameter [m] of the center circle",
	double m_center_circle_diameter;

	// "GoalAreaWidth: Width [m] of goal area"
	double m_goal_area_width;

	//"GoalAreaLength: Length [m] of goal area"
	double m_goal_area_length;

	// "PenaltyAreaPresent: does field have a penalty area?"
	bool m_penalty_area_present;

	// "GoalAreaWidth: Width [m] of goal area"
	double m_penalty_area_width;

	//"GoalAreaLength: Length [m] of goal area"
	double m_penalty_area_length;

	// max size of the robot (in 1 dim) [m]
	double m_robot_size;

	// radius of the ball [m]
	double m_ball_radius;

	double m_field_markings_width; // [m]

	double m_parking_area_width; // [m] area where players will be parked

	double m_parking_area_length; // [m] area where players will be parked

	double m_parking_distance_between_robots; // [m] between the parked robots (between the middle of the robots)

	double m_parking_distance_to_line; // [m] to line for parking : + is outside. 0 is on the line

	double m_penalty_spot_to_backline;

	double m_minimum_distance_to_goal_area; // TODO needed or part of parameters?

	double m_corner_circle_diameter;
};

	FieldConfig FillDefaultFieldConfig(); // only for off-line tests (automatic tests)

} //namespace

#endif
