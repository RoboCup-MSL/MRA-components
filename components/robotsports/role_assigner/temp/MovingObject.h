#ifndef MOVINGOBJECT_H
#define MOVINGOBJECT_H 1

#include "Position.h"
#include "Vector2D.h"
#include <string>
#include "planner_types.h"  // define in planner directory

using namespace std;
/**
 * Represents an object that moves with constant velocity.
 *
 * @author evg
 *
 */
namespace trs {

class MovingObject {
private:
	Position m_position;
	Vector2D m_velocity; // in m/s
	double m_rotationVelocity;
	int	m_label; // number of object, 1..10 for own player, >10 for opponents
	bool m_valid;

public:
	MovingObject();

	/**
	 * Constructs a new moving object
	 * @param x x-position Position of the object
	 * @param y y-position Position of the object
	 * velocity is set to vx, vy
	 */
	MovingObject(double x, double y, double rz, double vx, double vy, double vrz, int label, bool valid);

	MovingObject(const MovingObject& mo);

	/**
	 * Constructs a new moving object
	 * @param position Position of the object
	 * velocity is set to 0.0
	 */
	MovingObject(Position position, bool valid = true);

	/**
	 * Constructs a new moving object
	 * @param position Position of the object
	 * @param velocity Velocity of the object
	 */
	MovingObject(Position position, Vector2D linearVelocity, double rotationVelocity, bool valid = true);

	/**
	 * Position of the object
	 * @return Position
	 */
	Position getPosition()  const;

	/**
	 * Xy location of the object
	 * @return Position
	 */
	Vector2D getXYlocation() const;

	/**
	 * linear speed of the object (magnitude of velocity vector)
	 * @return linear velocity = sqrt(x^2, y^2)
	 */
	double getLinearVelocity()  const;

	/**
	 * Speed of the object
	 * @return Velocity vector
	 */
	void getVelocity(Vector2D& linearVelocity, double& rotationVelocity)  const;

	int getLabel() const;

	bool isValid() const;

	void set(double x, double y, double rz, double vx, double vy, double vrz, int label, bool valid);

	// set / update velocity
	void setVelocity(const Vector2D& linearVelocity, double rotationVelocity);

	/**
	 * Calculates the position of the object at a certain time, given the time
	 * mark of the position and assuming constant velocity.
	 *
	 * Currently it is assumed that the speed is known exactly and that
	 * calculating the new position does not influence the position confidence.
	 *
	 * @param time
	 *            Time for which to calculate the position.
	 * @return Position of the object.
	 */
	Position getPositionAt(double time) const;

	/**
	 * Moves the object at its current speed.
	 * @param timespan Elapsed time period
	 */
	void move(double timespan);

	/**
	 * Moves the object for the given offset and update time with timespan
	 */
	void move(const Vector2D& offset, double timespan);


	std::string toString(bool print_complete = true) const;

	operator moving_object_t() const;
	MovingObject& operator=(const moving_object_t& mo);

};

} //namespace
#endif
