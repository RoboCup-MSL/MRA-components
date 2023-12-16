#ifndef GEOMETRY_H
#define GEOMETRY_H 1

#include "Vector2D.h"

namespace trs {

class Geometry {
public:

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

};
} // namespace

#endif
