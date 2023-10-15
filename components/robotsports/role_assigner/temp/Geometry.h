#ifndef GEOMETRY_H
#define GEOMETRY_H 1

#include "Vector2D.h"

namespace trs {

class Geometry {
public:

	/**
	 * Calculates the distance of point c to the line segment given by the points p1 and p2. Only for points on the
	 * ground (z==0)
	 *
	 * @param c
	 *            Coordinate of a point c
	 * @param p1
	 *            Coordinate of one end of the line segment
	 * @param p2
	 *            Coordinate of the other end of the line segment
	 * @return Distance of c to the line segment (p1,p2)
	 */
	static double distanceFromPointToLineSegment(const Vector2D& c,
			const Vector2D& p1, const Vector2D& p2);

	/**
	 * Calculates the square of the distance from a point to a line segment. Only for points on the ground (z==0).
	 *
	 * @param c
	 * @param p1
	 *            One end point of the line segment
	 * @param p2
	 *            Other end point of the line segment
	 * @return The square of the distance from c to line segment (p1,p2)
	 */
	static double distanceFromPointToLineSegmentSquared(const Vector2D& c,
			const Vector2D& p1, const Vector2D& p2);

	/**
	 * Calculates the area of a triangle given the lengths of the three sides.
	 *
	 * @param a
	 *            Length of side 1
	 * @param b
	 *            Length of side 2
	 * @param c
	 *            Length of side 3
	 * @return Area of the triangle
	 */
	static double triangleArea(double a, double b, double c);

	/**
	 * Calculates the area of a triangle defined by three points.
	 *
	 * @param p1
	 *            First point of the triangle
	 * @param p2
	 *            Second point
	 * @param p3
	 *            Third
	 * @return Area of the triangle defined by (p1,p2,p3).
	 */
	static double triangleArea(const Vector2D& p1, const Vector2D& p2,
			const Vector2D& p3);

	/**
	 * Integrates 1 over the square of the distance between point c and line segment p1-p2.
	 *
	 * @param c
	 *            Point
	 * @param p1
	 *            Begin point of line segment
	 * @param p2
	 *            End point of line segment
	 * @return 1 over square of the distance between c and p1-p2, integrated over p1-p2.
	 */
	static double distanceIntegral(const Vector2D& c, const Vector2D& p1, const Vector2D& p2);

private:

	static double calculateLambda(const Vector2D& c, const Vector2D& p1,
			const Vector2D& p2);

};
} // namespace

#endif
