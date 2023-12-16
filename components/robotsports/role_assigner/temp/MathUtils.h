/**
 *  @file
 *  @brief   Generic mathematical functions
 *  @curator Jürge van Eijck
 */
#ifndef MATHUTILS_H
#define MATHUTILS_H 1

#ifdef __cplusplus
extern "C"
{
#endif

//#define MIN(a,b) (((a)<(b))?(a):(b))
//#define MAX(a,b) (((a)>(b))?(a):(b))
//#define COERCE(a,lower_limit,upper_limit) (((a)<(lower_limit))?(lower_limit):(((a)>(upper_limit))?(upper_limit):(a)))
//#define ABS(a) (((a)>(0))?(a):-(a))
//#define SIGN(a) (((a)>0)-((a)<0))
//
//double from0topi(double a);   // bring any angle [rad] to the range 0 .. pi
//double from0to2pi(double a);  // bring any angle to the range 0 .. 2 pi
double from_pitopi(double a); // bring any angle [rad] to the range -pi .. pi
//double rad2deg(double rad);   // convert from radials to degrees
double deg2rad(double deg);   // convert from degrees to radials
//
//double min_angle(double start_angle, double end_angle);   // get shortest angle between two angle [rad]
//

// intersection point for two lines L1 = [(x1,y1) (x2,y2)] L2 = [(x3,y3) (x4,y4)]. formula from wikipedia line-intersection
bool getIntersectionOfTwoLines(double& px, double& py, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4);
double getDistanceFromPointToLine(double Ax, double Ay, double Bx, double By, double Px, double Py);

// return true if point (x,y) is in triangle defined by the points (x1,y1), (x2,y2), (x3,y3), else return false
bool inTriangle(double x1, double y1, double x2, double y2, double x3, double y3, double x, double y);

// Find the points of intersection.
// in: center of circle <cx, cy> with given radius <radius>, line defined by two points <x1, y1> and <x2, y2>
// return value is number of solutions (0, 1 or 2).
// solution-1: <int1x, int1y> solution-2: <int2x, int2y>
int findLineCircleIntersections(double cx, double cy, double radius, double x1, double y1, double x2, double y2,
		                        double& int1x, double& int1y, double& int2x, double& int2y);
//
//  Determines the intersection point of the line segment defined by points (Ax,Ay) and (Bx, By)
//  with the line segment defined by points (Cx, Cy) and (Dx, Dy).
//
//  Returns true, if the intersection point was found, and stores that point in Px and Py.
//  Returns false, if there is no determinable intersection point, in which case Px, and Py will
//  be unmodified.
bool lineSegmentIntersection(double Ax, double Ay, double Bx, double By,
							 double Cx, double Cy, double Dx, double Dy, double& px, double& py);
//
//// Find the points where the two circles intersect.
//// return number of solutions
//int findCircleCircleIntersections(
//		double cx0, double cy0, double radius0,
//		double cx1, double cy1, double radius1,
//		double& rInt1x, double& rInt1y, double& rInt2x, double& rInt2y);
//
//// Find the tangent points for this circle and external point.
//// Return true if we find the tangents, false if the point is
//// inside the circle.
//bool findTangentsOfPointWithCircle(double cx, double cy, double radius,
//    double px, double py, double& rT1x, double& rT1y, double& rT2x, double& rT2y);
//

/**
 * Calculates the intersection point of line given by points P1 (p1x, p1y) and P2 (p2x, p2y), and the line perpendicular to (p1,p2) going
 * through point P (px, py).
 *
 * @param px
 *            X of Point though which the perpendicular line must go
 * @param py
 *            Y of Point though which the perpendicular line must go
 * @param p1x
 *            X of One point on the line (p1,p2)
 * @param p1y
 *            Y of One point on the line (p1,p2)
 * @param p2x
 *            X of Other point on the line (p1,p2)
 * @param p2y
 *            Y ofOther point on the line (p1,p2)
 * @return Intersection point D. Note that c.distanceTo(D) is the distance of c to the line (p1,p2).
 */
void intersectPerpendicular(double& intX, double& intY, double p1x, double p1y, double p2x, double p2y, double px, double py);

#ifdef __cplusplus
}
#endif

#endif
