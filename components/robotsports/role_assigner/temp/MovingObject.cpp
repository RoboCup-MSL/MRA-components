#include "MovingObject.h"
#include <iomanip>
#include <ostream>
#include <sstream>
#include "geometry.hpp"

namespace trs {

	MovingObject::MovingObject() :
	    m_position(Position(MRA::Geometry::Point(0,0), 0.0, 1.0, 0)),
		m_velocity(MRA::Geometry::Point(0, 0)),
		m_label(-1) {

	}

	MovingObject::MovingObject(double x, double y, double rz, double vx, double vy, int label):
				m_position(Position(MRA::Geometry::Point(x,y), 0.0, 1.0, rz)),
				m_velocity(MRA::Geometry::Point(vx,vy)),
				m_label(label) {
		}

	MovingObject::MovingObject(const MovingObject& mo) :
		m_position(mo.m_position),
		m_velocity(mo.m_velocity),
		m_label(mo.m_label) {
	}

	/**
	 * Constructs a new moving object
	 * @param position Position of the object
	 */
	MovingObject::MovingObject(Position position) :
					m_position(position),
					m_velocity(MRA::Geometry::Point(0,0)),
					m_label(-1)  {
	}

	/**
	 * Position of the object
	 * @return Position
	 */
	Position MovingObject::getPosition() const {
		return m_position;
	}

	/**
	 * Xy location of the object
	 * @return Position
	 */
	MRA::Geometry::Point MovingObject::getXYlocation() const {
		return m_position.getPoint();
	}


	/**
	 * linear speed of the object (magnitude of velocity vector)
	 * @return linear velocity = sqrt(x^2, y^2)
	 */
	double MovingObject::getLinearVelocity() const {
		return m_velocity.size();
	}

	/**
	 * Speed of the object
	 * @return Velocity vector
	 */
	void MovingObject::getVelocity(MRA::Geometry::Point& linearVelocity) const {
		linearVelocity = m_velocity;
	}

	int MovingObject::getLabel() const {
		return m_label;
	}

	void MovingObject::set(double x, double y, double rz, double vx, double vy, int label) {
				m_position.set(x,y, rz);
				m_velocity.x = vx;
				m_velocity.y = vy;
				m_label = label;
	}

	void MovingObject::setVelocity(const MRA::Geometry::Point& linearVelocity) {
		m_velocity = linearVelocity;
	}

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
	Position MovingObject::getPositionAt(double time) const {
		double timespan = time - m_position.getTime();
		Position result(m_position);
		MRA::Geometry::Point velocity(m_velocity);
		velocity *= timespan;
		result.move(velocity, timespan);
		return result;
	}

	std::string MovingObject::toString(bool print_complete) const {
		std::stringstream buffer;
		buffer << std::fixed << std::setprecision(2)
				<< " x: " << m_position.getPoint().x
			    << " y: " << m_position.getPoint().y;
		if (print_complete) {
			buffer << " vx: " << m_velocity.x << " vy: " << m_velocity.y
				   << " label: " << m_label;
		}
		return buffer.str();
	}


}; // namespace
