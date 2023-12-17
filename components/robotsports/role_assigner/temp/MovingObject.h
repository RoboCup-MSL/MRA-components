#ifndef MOVINGOBJECT_H
#define MOVINGOBJECT_H 1

#include "Position.h"
#include "geometry.hpp"
#include <string>

#include "../internals/planner_types.hpp"  // define in planner directory

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
	MRA::Geometry::Point m_velocity; // in m/s
	int	m_label; // number of object, 1..10 for own player, >10 for opponents

public:
	MovingObject();

	/**
	 * Constructs a new moving object
	 * @param x x-position Position of the object
	 * @param y y-position Position of the object
	 * velocity is set to vx, vy
	 */
	MovingObject(double x, double y, double rz, double vx, double vy, int label);

	MovingObject(const MovingObject& mo);

	/**
	 * Constructs a new moving object
	 * @param position Position of the object
	 * velocity is set to 0.0
	 */
	MovingObject(Position position);

	/**
	 * Position of the object
	 * @return Position
	 */
	Position getPosition()  const;

	/**
	 * Xy location of the object
	 * @return Position
	 */
	MRA::Geometry::Point getXYlocation() const;

	/**
	 * linear speed of the object (magnitude of velocity vector)
	 * @return linear velocity = sqrt(x^2, y^2)
	 */
	double getLinearVelocity()  const;

	/**
	 * Speed of the object
	 * @return Velocity vector
	 */
	void getVelocity(MRA::Geometry::Point& linearVelocity)  const;

	int getLabel() const;

	void set(double x, double y, double rz, double vx, double vy, int label);

	// set / update velocity
	void setVelocity(const MRA::Geometry::Point& linearVelocity);

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
//

	std::string toString(bool print_complete = true) const;

};

} //namespace
#endif
