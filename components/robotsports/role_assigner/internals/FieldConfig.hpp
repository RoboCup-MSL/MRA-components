/**
 *  @file
 *  @brief   Represents field dimension for the planner
 *  @curator Jürge van Eijck
 */

#ifndef FIELD_CONFIG_HPP
#define FIELD_CONFIG_HPP 1
#include "geometry.hpp"
#include "FieldParameters.hpp"
#include <string>

namespace MRA {

class FieldConfig {

public:
    FieldConfig() {};
    FieldConfig(const FieldParameters& fp);

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

    FieldParameters getFieldParameters() const;
private:
    FieldParameters m_fp;
    double m_robot_size = 0.52; // max size of a player
    double m_ball_radius = 0.11; // radius of the ball
    double m_penalty_area_present = true;
    double m_parking_area_width = 4.0;
    double m_parking_area_length = 0.5;
    double m_parking_distance_between_robots = 0.75;
    double m_parking_distance_to_line = 0.0;
    double m_minimum_distance_to_goal_area = 0.5;
};

} //namespace

#endif /* FIELD_CONFIG_HPP */
