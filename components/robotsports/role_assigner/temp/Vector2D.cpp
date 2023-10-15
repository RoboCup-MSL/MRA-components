/**
 * Representation of a 3D vector.
 * 
 * This class is designed to be immutable.
 * 
 * @author rbu
 * 
 */

#include "math.h"
#include "Vector2D.h"
#include <iomanip>
#include <ostream>
#include <sstream>

using namespace std;

namespace trs {
	const double Vector2D::EQUALITY_TOLERANCE = 0.00001;

	/**
	 * Constructs new 3D vector
	 * 
	 * @param aX
	 *            X coordinate
	 * @param aY
	 *            Y coordinate
	 * @param aZ
	 *            Z coordinate
	 */
	Vector2D::Vector2D( double aX,  double aY) : m_x(aX),  m_y(aY){

	}

	Vector2D::Vector2D( const Vector2D& v) {
		m_x = v.m_x;
		m_y = v.m_y;
	}

	double Vector2D::getX() const {
		return m_x;
	}

	double Vector2D::getY() const {
		return m_y;
	}

	double Vector2D::distanceTo(const Vector2D& aCoordinate) const {
		double deltaX = aCoordinate.m_x - m_x;
		double deltaY = aCoordinate.m_y - m_y;

		return hypot(deltaX, deltaY);
	}

	Vector2D Vector2D::add(const Vector2D& aVector) const {
		return Vector2D(m_x + aVector.m_x, m_y + aVector.m_y);
	}

	Vector2D Vector2D::subtract(const Vector2D& aVector) const {
		return Vector2D(m_x - aVector.m_x, m_y - aVector.m_y);
	}

	double Vector2D::inproduct( const Vector2D& aVector) const {
		return m_x * aVector.m_x + m_y * aVector.m_y;
	}

	double Vector2D::angle( const Vector2D& aVector) const {
		return atan2(m_y -aVector.m_y, m_x - aVector.m_x);
	}

	/**
	 * Provides the norm (a.k.a. length or size of a vector but these terms are
	 * sometimes used for different things).
	 *
	 * @return Norm of this vector.
	 */
	double Vector2D::norm() const {
		double norm = hypot(m_x, m_y);
		return norm;
	}

	/**
	 * Scalar multiplication
	 *
	 * @param factor
	 *            Factor for multiplication.
	 * @return Vector multiplied by the factor.
	 */
	Vector2D Vector2D::multiply(double factor) const {
		return Vector2D(factor * m_x, factor * m_y);
	}

	/**
	 * Normalizes the vector such that its norm is 1.0 and direction unchanged.
	 *
	 * @return Normalized vector. When input vector has length 0, the result is
	 *         (1,0,0)
	 */
	Vector2D Vector2D::normalize() {
		Vector2D result;
		double norm_res = norm();
		if (fabs(norm_res) < EQUALITY_TOLERANCE) {
			result = Vector2D(1.0, 0.0);
		} else {
			result = Vector2D(m_x / norm_res, m_y / norm_res);
		}
		return result;
	}

	// return the closest vector of the given vector
	Vector2D Vector2D::closestTo(const std::vector<Vector2D>& positions) const  {
		Vector2D closest = positions[0];
		double distClosest = this->distanceTo(closest);
		for (auto it = positions.begin(); it != positions.end(); ++it ) {
			if (this->distanceTo(*it) < distClosest) {
				closest = *it;
				distClosest = this->distanceTo(*it);
			}
		}
		return closest;
	}

	bool Vector2D::equals( const Vector2D& c) const {
		return fabs(c.m_x - m_x) < EQUALITY_TOLERANCE && fabs(c.m_y - m_y) < EQUALITY_TOLERANCE;
	}

	std::ostream& operator<<(std::ostream &strm, const Vector2D&a) {
	  return strm << "x: " << a.m_x << " y: " << a.m_y;
	}

	std::string Vector2D::toString() const {
		std::stringstream buffer;
		buffer << std::fixed << std::setprecision(2) << "x: " << m_x << " y: " << m_y;
		return buffer.str();
	}
}


