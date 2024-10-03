/**
 *  @file
 *  @brief   Represents field dimension for the planner
 *  @curator Jürge van Eijck
 */

#ifndef FIELD_CONFIG_HPP
#define FIELD_CONFIG_HPP 1
#include "geometry.hpp"
#include "EnvironmentParameters.hpp"

#include <string>

namespace MRA {

class Environment {

public:
    Environment() {};
    Environment(const EnvironmentParameters& env);

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
    EnvironmentParameters m_env_params;
};

    Environment FillDefaultEnvironment(); // only for off-line tests (automatic tests)

} //namespace

#endif /* FIELD_CONFIG_HPP */
