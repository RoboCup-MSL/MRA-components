#include <cmath>
#include <algorithm>
#include <limits>
#include "Geometry.h"

using namespace std;

namespace trs {

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
 *           infinity is returned if distanceIntegral can not be calculated.
 */
double Geometry::distanceIntegral(const Vector2D& c, const Vector2D& p1, const Vector2D& p2) {
	// Calculate square of the length of p1-p2.
	double l2 = (p2.m_x - p1.m_x) * (p2.m_x - p1.m_x) + (p2.m_y - p1.m_y) * (p2.m_y - p1.m_y);
	if (fabs(l2) < 1e-16) {
		// prevent division by zero
		return std::numeric_limits<double>::infinity();
	}
	double lambda = ((p2.m_x - p1.m_x) * (c.m_x - p1.m_x) + (p2.m_y - p1.m_y) * (c.m_y - p1.m_y)) / l2;
	double len = sqrt(l2);
	double s1 = -lambda * len;
	double s2 = (1 - lambda) * len;
	// Find the point (xx,yy) where the perpendicular line through c crosses line p1-p2
	double xx = p1.m_x + lambda * (p2.m_x - p1.m_x);
	double yy = p1.m_y + lambda * (p2.m_y - p1.m_y);
	// Calculate distance of c to line p1-p2
	double d = sqrt((c.m_x - xx) * (c.m_x - xx) + (c.m_y - yy) * (c.m_y - yy));
	if (fabs(d) < 1e-16) {
		// prevent division by zero
		return std::numeric_limits<double>::infinity();
	}
	// Formula found using WolframAlpha.com. Type: integrate 1/(x^2+d^2) dx
	return (atan2(s2, d) - atan2(s1, d)) / d;
	// return (aTan2(s2,d)-aTan2(s1,d))/d;
	// return (Math.atan2(s2, d)-Math.atan2(s1, d))/d;
}

} // namespace


