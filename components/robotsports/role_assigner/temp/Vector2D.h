/**
 * Representation of a 2D vector.
 * 
 * This class is designed to be immutable.
 * 
 * @author rbu
 * 
 */
#ifndef VECTOR2D_H
#define VECTOR2D_H

#include <string>
#include <vector>

// Position definition
typedef struct pos
{
  double x; // x coordinate
  double y; // y coordinate
  double z; // z coordinate
  double r; // rotation in radials
} pos_t;


namespace trs {


class Vector2D {
private:
	 static const double EQUALITY_TOLERANCE;

public:
	double m_x;
	/** do not change */
	double m_y;


	/**
	 * Constructs new 2D vector
	 * 
	 * @param aX
	 *            X coordinate
	 * @param aY
	 *            Y coordinate
	 * @param aZ
	 *            Z coordinate
	 */
	Vector2D( double aX = 0.0,  double aY = 0.0) ;
	Vector2D( const Vector2D& v);

	double getX() const;

	double getY() const;

	double distanceTo( const Vector2D& raCoordinate) const;

	Vector2D add( const Vector2D& raVector) const;

	Vector2D subtract(const Vector2D& raVector) const;

	double inproduct( const Vector2D& raVector) const;

	/* get angle between two points in radians (between -pi and +pi) */
	double angle( const Vector2D& aVector) const;


	/**
	 * Provides the norm (a.k.a. length or size of a vector but these terms are
	 * sometimes used for different things).
	 * 
	 * @return Norm of this vector.
	 */
	double norm() const;

	/**
	 * Scalar multiplication
	 * 
	 * @param factor
	 *            Factor for multiplication.
	 * @return Vector multiplied by the factor.
	 */
	Vector2D multiply(double factor) const;

	/**
	 * Normalizes the vector such that its norm is 1.0 and direction unchanged.
	 * 
	 * @return Normalized vector. When input vector has length 0, the result is
	 *         (1,0,0)
	 */
	Vector2D normalize();

	// return the closest vector of the given vector
	Vector2D closestTo(const std::vector<Vector2D>& positions) const;

	// true if vector C is equal (values within eps).
	bool equals( const Vector2D& c) const;

	std::string toString() const;
};

}
#endif
