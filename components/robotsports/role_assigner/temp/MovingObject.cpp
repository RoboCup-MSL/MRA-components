#include "MovingObject.h"
#include <iomanip>
#include <ostream>
#include <sstream>
#include "geometry.hpp"

namespace trs {

	MovingObject::MovingObject() :
	    m_position(Position(MRA::Geometry::Point(0,0), 0.0, 1.0, 0)),
		m_velocity(MRA::Geometry::Point(0, 0)),
		m_rotationVelocity(0.0),
		m_label(-1),
		m_valid(false) {

	}

	MovingObject::MovingObject(double x, double y, double rz, double vx, double vy, double vrz, int label, bool valid):
				m_position(Position(MRA::Geometry::Point(x,y), 0.0, 1.0, rz)),
				m_velocity(MRA::Geometry::Point(vx,vy)),
				m_rotationVelocity(vrz),
				m_label(label),
				m_valid(valid) {
		}

	MovingObject::MovingObject(const MovingObject& mo) :
		m_position(mo.m_position),
		m_velocity(mo.m_velocity),
		m_rotationVelocity(mo.m_rotationVelocity),
		m_label(mo.m_label),
		m_valid(mo.m_valid) {
	}

	/**
	 * Constructs a new moving object
	 * @param position Position of the object
	 */
	MovingObject::MovingObject(Position position, bool valid) :
					m_position(position),
					m_velocity(MRA::Geometry::Point(0,0)),
					m_rotationVelocity(0.0),
					m_label(-1),
					m_valid(valid)  {
	}

	/**
	 * Constructs a new moving object
	 * @param position Position of the object
	 * @param velocity Velocity of the object
	 */
	MovingObject::MovingObject(Position position, MRA::Geometry::Point linearVelocity, double rotationVelocity, bool valid) :
			m_position(position),
			m_velocity(linearVelocity),
			m_rotationVelocity(rotationVelocity),
			m_label(-1),
			m_valid(valid) {
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
	void MovingObject::getVelocity(MRA::Geometry::Point& linearVelocity, double& rotationVelocity) const {
		linearVelocity = m_velocity;
		rotationVelocity = m_rotationVelocity;
	}

	int MovingObject::getLabel() const {
		return m_label;
	}

	bool MovingObject::isValid() const {
		return m_valid;
	}

	void MovingObject::set(double x, double y, double rz, double vx, double vy, double vrz, int label, bool valid) {
				m_position.set(x,y, rz);
				m_velocity.x = vx;
				m_velocity.y = vy;
				m_rotationVelocity = vrz;
				m_label = label;
				m_valid = valid;
	}

	void MovingObject::setVelocity(const MRA::Geometry::Point& linearVelocity, double rotationVelocity) {
		m_velocity = linearVelocity;
		m_rotationVelocity = rotationVelocity;
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

	/**
	 * Moves the object at its current speed.
	 * @param timespan Elapsed time period
	 */
	void MovingObject::move(double timespan) {
        MRA::Geometry::Point velocity(m_velocity);
        velocity *= timespan;
		m_position.move(velocity, timespan);
	}

	/**
	 * Moves the object for the given offset and update time with timespan
	 */
	void MovingObject::move(const MRA::Geometry::Point& offset, double timespan) {
		m_position.move(offset, timespan);
	}

	std::string MovingObject::toString(bool print_complete) const {
		std::stringstream buffer;
		buffer << std::fixed << std::setprecision(2)
				<< " x: " << m_position.getPoint().x
			    << " y: " << m_position.getPoint().y;
		if (print_complete) {
			buffer << " vx: " << m_velocity.x << " vy: " << m_velocity.y
				   << " vr: " <<  m_rotationVelocity << " valid: " << (int) m_valid
				   << " label: " << m_label;
		}
		return buffer.str();
	}

	MovingObject::operator moving_object_t() const {
		moving_object_t mo;
		mo.x = this->m_position.getPoint().x;
		mo.y = this->m_position.getPoint().y;
		mo.rz = this->m_position.getRotationZ();
		mo.velx = this->m_velocity.x;
		mo.vely = this->m_velocity.y;
		mo.velrz = this->m_rotationVelocity;
        mo.valid = static_cast<long>(this->m_valid);
		return mo;
	}

	MovingObject& MovingObject::operator=(const moving_object_t& mo) {
		this->m_position.set(mo.x, mo.y, mo.rz);
		this->m_velocity.x = mo.velx;
		this->m_velocity.y = mo.vely;
		this->m_rotationVelocity = mo.velrz;
        this->m_label = mo.label;
        this->m_valid = static_cast<bool>(mo.valid);
		return *this;
	}

}; // namespace
