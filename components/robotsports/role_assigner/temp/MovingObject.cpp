#include "MovingObject.h"
#include <iomanip>
#include <ostream>
#include <sstream>

namespace trs {

	MovingObject::MovingObject() :
	    m_position(Position(Vector2D(0,0), 0.0, 1.0, 0)),
		m_velocity(Vector2D(0, 0)),
		m_rotationVelocity(0.0),
		m_label(-1),
		m_valid(false) {

	}

	MovingObject::MovingObject(double x, double y, double rz, double vx, double vy, double vrz, int label, bool valid):
				m_position(Position(Vector2D(x,y), 0.0, 1.0, rz)),
				m_velocity(Vector2D(vx,vy)),
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
					m_velocity(Vector2D(0,0)),
					m_rotationVelocity(0.0),
					m_label(-1),
					m_valid(valid)  {
	}

	/**
	 * Constructs a new moving object
	 * @param position Position of the object
	 * @param velocity Velocity of the object
	 */
	MovingObject::MovingObject(Position position, Vector2D linearVelocity, double rotationVelocity, bool valid) :
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
	Vector2D MovingObject::getXYlocation() const {
		return m_position.getVector2D();
	}


	/**
	 * linear speed of the object (magnitude of velocity vector)
	 * @return linear velocity = sqrt(x^2, y^2)
	 */
	double MovingObject::getLinearVelocity() const {
		return m_velocity.norm();
	}

	/**
	 * Speed of the object
	 * @return Velocity vector
	 */
	void MovingObject::getVelocity(Vector2D& linearVelocity, double& rotationVelocity) const {
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
				m_velocity.m_x = vx;
				m_velocity.m_y = vy;
				m_rotationVelocity = vrz;
				m_label = label;
				m_valid = valid;
	}

	void MovingObject::setVelocity(const Vector2D& linearVelocity, double rotationVelocity) {
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
		result.move(m_velocity.multiply(timespan), timespan);
		return result;
	}

	/**
	 * Moves the object at its current speed.
	 * @param timespan Elapsed time period
	 */
	void MovingObject::move(double timespan) {
		m_position.move(m_velocity.multiply(timespan), timespan);
	}

	/**
	 * Moves the object for the given offset and update time with timespan
	 */
	void MovingObject::move(const Vector2D& offset, double timespan) {
		m_position.move(offset, timespan);
	}

	std::string MovingObject::toString(bool print_complete) const {
		std::stringstream buffer;
		buffer << std::fixed << std::setprecision(2)
				<< " x: " << m_position.getVector2D().m_x
			    << " y: " << m_position.getVector2D().m_y;
		if (print_complete) {
			buffer << " rz: " << m_position.getRotationZ()
				   << " vx: " << m_velocity.m_x << " vy: " << m_velocity.m_y
				   << " vr: " <<  m_rotationVelocity << " valid: " << (int) m_valid
				   << " label: " << m_label;
		}
		return buffer.str();
	}

	MovingObject::operator moving_object_t() const {
		moving_object_t mo;
		mo.x = this->m_position.getVector2D().m_x;
		mo.y = this->m_position.getVector2D().m_y;
		mo.rz = this->m_position.getRotationZ();
		mo.velx = this->m_velocity.m_x;
		mo.vely = this->m_velocity.m_y;
		mo.velrz = this->m_rotationVelocity;
        mo.valid = static_cast<long>(this->m_valid);
		return mo;
	}

	MovingObject& MovingObject::operator=(const moving_object_t& mo) {
		this->m_position.set(mo.x, mo.y, mo.rz);
		this->m_velocity.m_x = mo.velx;
		this->m_velocity.m_y = mo.vely;
		this->m_rotationVelocity = mo.velrz;
        this->m_label = mo.label;
        this->m_valid = static_cast<bool>(mo.valid);
		return *this;
	}

}; // namespace
