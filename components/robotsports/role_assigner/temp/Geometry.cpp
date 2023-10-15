#include <cmath>
#include <algorithm>
#include <limits>
#include "Geometry.h"

using namespace std;

namespace trs {

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
double Geometry::distanceFromPointToLineSegment(const Vector2D& c,
		const Vector2D& p1, const Vector2D& p2) {
	return sqrt(distanceFromPointToLineSegmentSquared(c, p1, p2));
}

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
double Geometry::distanceFromPointToLineSegmentSquared(const Vector2D& c,
		const Vector2D& p1, const Vector2D& p2) {
	double lambda = calculateLambda(c, p1, p2);
	if (lambda <= 0) {
		return (c.m_x - p1.m_x) * (c.m_x - p1.m_x) + (c.m_y - p1.m_y) * (c.m_y - p1.m_y);
	} else if (lambda >= 1) {
		return (c.m_x - p2.m_x) * (c.m_x - p2.m_x) + (c.m_y - p2.m_y) * (c.m_y - p2.m_y);
	} else {
		double tx = c.m_x - p1.m_x - lambda * (p2.m_x - p1.m_x);
		double ty = c.m_y - p1.m_y - lambda * (p2.m_y - p1.m_y);
		return (tx * tx + ty * ty);
	}
}

double Geometry::calculateLambda(const Vector2D& c, const Vector2D& p1,
		const Vector2D& p2) {
	double lambda =
			((p2.m_x - p1.m_x) * (c.m_x - p1.m_x) + (p2.m_y - p1.m_y) * (c.m_y - p1.m_y))
					/ ((p2.m_x - p1.m_x) * (p2.m_x - p1.m_x)
							+ (p2.m_y - p1.m_y) * (p2.m_y - p1.m_y));
	return lambda;
}

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
double Geometry::triangleArea(double a, double b, double c) {
	// using Heron's formula
	double s = (a + b + c) / 2.0;
	return sqrt(s * (s - a) * (s - b) * (s - c));
}

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
double Geometry::triangleArea(const Vector2D& p1, const Vector2D& p2,
		const Vector2D& p3) {
	double a = p1.distanceTo(p2);
	double b = p1.distanceTo(p3);
	double c = p2.distanceTo(p3);
	return triangleArea(a, b, c);
}

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

//    /**
//     * Calculates the intersection point of line given by points p1 and p2, and the line perpendicular to (p1,p2) going
//     * through point c.
//     *
//     * @param c
//     *            Point though which the perpendicular line must go
//     * @param p1
//     *            One point on the line (p1,p2)
//     * @param p2
//     *            Other point on the line (p1,p2)
//     * @return Intersection point D. Note that c.distanceTo(D) is the distance of c to the line (p1,p2).
//     */
//    public static Vector3D intersectPerpendicular(const Vector3D& c, const Vector3D& p1, const Vector3D& p2) {
//        double lambda = calculateLambda(c, p1, p2);
//        return new Vector3D(p1.x + lambda * (p2.x - p1.x), p1.y + lambda * (p2.y - p1.y), 0.0);
//    }
////    /**
////     * Fast approximation of atan2(y,x)
////     *
////     * @param y
////     * @param x
////     * @return Approximate value of atan2(y,x). Some experiments show an error of sometimes 0.07, so pretty big.
////     */
////    public static double aTan2(final double y, final double x) {
////        double coeff1 = Math.PI / (4d);
////        double coeff2 = 3d * coeff1;
////        double absY = Math.abs(y);
////        double angle;
////        if (x >= 0d) {
////            double r = (x - absY) / (x + absY);
////            angle = coeff1 - coeff1 * r;
////        } else {
////            double r = (x + absY) / (absY - x);
////            angle = coeff2 - coeff1 * r;
////        }
////        return y < 0d ? -angle : angle;
////    }
//}

