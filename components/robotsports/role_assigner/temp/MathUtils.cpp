/**
 *  @file
 *  @brief   Generic mathematical functions
 *  @curator Jürge van Eijck
 */
#include <cmath>
#include "MathUtils.h"
#include <iostream>
using namespace std;

// -----------------------------------------------------------------------------
// intersection point for two lines L1 = [(x1,y1) (x2,y2)] L2 = [(x3,y3) (x4,y4)]. formula from wikipedia line-intersection
bool getIntersectionOfTwoLines(double& px, double& py, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
    double divider = ((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));
    if (divider != 0.0) {
        px = ( (x1*y2-y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4)) / divider;
        py = ( (x1*y2-y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4)) / divider;
        return true;
    }
    else {
        px = 0; // dummy values
        py = 0; // dummy values
        return false;
    }
}

// -----------------------------------------------------------------------------
// get distance from a given point (Px, Py) to given line segment AB (Ax,Ay), (Bx, By)
double getDistanceFromPointToLineSegment(double Ax, double Ay, double Bx, double By, double Px, double Py)
{
    // Calculate the vector along the line
    double lineX = Bx - Ax;
    double lineY = By - Ay;

    // Calculate the vector from the start of the line segment to the point
    double pointX = Px - Ax;
    double pointY = Py - Ay;

    // Calculate the projection of the point vector onto the line vector
    double projection = (pointX * lineX + pointY * lineY) / (lineX * lineX + lineY * lineY);

    double closestX, closestY;

    // if projection falls outside the line segment, calculate distance to the closer endpoint
    if (projection < 0) {
        closestX = Ax;
        closestY = Ay;
    }
    else if (projection > 1) {
        closestX = Bx;
        closestY = By;
    }
    else {
        closestX = Ax + projection * lineX;
        closestY = Ay + projection * lineY;
    }
    // Calculate the distance between the point and closest point on the line segment.
    return sqrt(pow(Px - closestX, 2) + pow(Py - closestY, 2));
}

// -----------------------------------------------------------------------------
// return true if point (x,y) is in triangle defined by the points (x1,y1), (x2,y2), (x3,y3), else return false
bool inTriangle(double x1, double y1, double x2, double y2, double x3, double y3, double x, double y) {
    double alpha = ((y2 - y3)*(x - x3) + (x3 - x2)*(y - y3)) / ((y2 - y3)*(x1- x3) + (x3 - x2)*(y1 - y3));
    double beta = ((y3 - y1)*(x - x3) + (x1- x3)*(y - y3)) / ((y2 - y3)*(x1- x3) + (x3 - x2)*(y1 - y3));
    double gamma = 1.0 - alpha - beta;

    bool in_triangle = ((alpha > 0) && (beta > 0) && (gamma > 0));
    return in_triangle;
}


// -----------------------------------------------------------------------------
// Find the points of intersection.
// in: center of circle <cx, cy> with given radius <radius>, line defined by two points <x1, y1> and <x2, y2>
// return value is number of solutions (0, 1 or 2).
// solution-1: <int1x, int1y> solution-2: <int2x, int2y>
int findLineCircleIntersections(double cx, double cy, double radius, double x1, double y1, double x2, double y2,
                                double& int1x, double& int1y, double& int2x, double& int2y)
{
    float dx, dy, A, B, C, det, t;

    dx = x2 - x1;
    dy = y2 - y1;

    A = dx * dx + dy * dy;
    B = 2 * (dx * (x1 - cx) + dy * (y1 - cy));
    C = (x1 - cx) * (x1 - cx) +
        (y1 - cy) * (y1 - cy) -
        radius * radius;

    det = B * B - 4 * A * C;
    if ((A <= 0.0000001) || (det < 0))
    {
        // No real solutions.
        int1x = NAN;
        int1y = NAN;
        int2x = NAN;
        int2y = NAN;
        return 0;
    }
    else if (det == 0)
    {
        // One solution.
        t = -B / (2 * A);
        int1x = x1 + t * dx;
        int1y = y1 + t * dy;
        int2x = NAN;
        int2y = NAN;
        return 1;
    }
    else
    {
        // Two solutions.
        t = (float)((-B + sqrt(det)) / (2 * A));
        int1x = x1 + t * dx;
        int1y = y1 + t * dy;
        t = (float)((-B - sqrt(det)) / (2 * A));
        int2x = x1 + t * dx;
        int2y = y1 + t * dy;
        return 2;
    }
}


// -----------------------------------------------------------------------------
//  public domain function by Darel Rex Finley, 2006
//
//  Source and description on http://alienryderflex.com/intersect/
//
//  Determines the intersection point of the line segment defined by points (Ax,Ay) and (Bx, By)
//  with the line segment defined by points (Cx, Cy) and (Dx, Dy).
//
//  Returns true, if the intersection point was found, and stores that point in Px and Py.
//  Returns false, if there is no determinable intersection point, in which case Px, and Py will
//  be unmodified.
bool lineSegmentIntersection(double Ax, double Ay, double Bx, double By,
                             double Cx, double Cy, double Dx, double Dy,
                             double& px, double& py) {
    double distAB, theCos, theSin, newX, ABpos;

    // Fail if either line segment is zero-length
    if ((Ax == Bx && Ay == By) || (Cx == Dx && Cy == Dy)) {
        return false;
    }

    // Fail if the segments share an end-point
    if ((Ax == Cx && Ay == Cy) || (Bx == Cx && By == Cy)
            || (Ax == Dx && Ay == Dy) || (Bx == Dx && By == Dy)) {
        return false;
    }

    // (1) Translate the system so that point A is on the origin
    Bx -= Ax;
    By -= Ay;
    Cx -= Ax;
    Cy -= Ay;
    Dx -= Ax;
    Dy -= Ay;

    // Discover the length of segment A-B
    distAB = sqrt(Bx * Bx + By * By);

    // (2) Rotate the system so that point B is on the positive X axis.
    theCos = Bx / distAB;
    theSin = By / distAB;
    newX = Cx * theCos + Cy * theSin;
    Cy = Cy * theCos - Cx * theSin;
    Cx = newX;
    newX = Dx * theCos + Dy * theSin;
    Dy = Dy * theCos - Dx * theSin;
    Dx = newX;

    // Fail if segment C-D doesn't cross line A-B.
    if ((Cy < 0. && Dy < 0.) || (Cy >= 0. && Dy >= 0.)) {
        return false;
    }

    // (3) Discover the position of the intersection point along line A-B.
    ABpos = Dx + (Cx - Dx) * Dy / (Dy - Cy);

    // Fail if segment C-D crosses line A-B outside of segment A-B.
    if (ABpos < 0. || ABpos > distAB) {
        return false;
    }

    // (4) Apply the discovered position to line A-B in the original coordinate system.
    px = Ax + ABpos * theCos;
    py = Ay + ABpos * theSin;

    //  Success.
    return true;
}


// -----------------------------------------------------------------------------
// Find the points where the two circles intersect.
int FindCircleCircleIntersections(
        double cx0, double cy0, double radius0,
        double cx1, double cy1, double radius1,
        double& rInt1x, double& rInt1y, double& rInt2x, double& rInt2y)
{
    rInt1x = NAN;
    rInt1y = NAN;
    rInt2x = NAN;
    rInt2y = NAN;

    // Find the distance between the centers.
    float dx = cx0 - cx1;
    float dy = cy0 - cy1;
    double dist = sqrt(dx * dx + dy * dy);

    // See how many solutions there are.
    if (dist > radius0 + radius1)
    {
        // No solutions, the circles are too far apart.
        return 0;
    }
    else if (dist < fabs(radius0 - radius1))
    {
        // No solutions, one circle contains the other.
        return 0;
    }
    else if ((dist == 0) && (radius0 == radius1))
    {
        // No solutions, the circles coincide.
        return 0;
    }
    else
    {
        // Find a and h.
        double a = (radius0 * radius0 -
            radius1 * radius1 + dist * dist) / (2 * dist);
        double h = sqrt(radius0 * radius0 - a * a);

        // Find P2.
        double cx2 = cx0 + a * (cx1 - cx0) / dist;
        double cy2 = cy0 + a * (cy1 - cy0) / dist;

        // Get the points P3.
        rInt1x = (cx2 + h * (cy1 - cy0) / dist);
        rInt1y = (cy2 - h * (cx1 - cx0) / dist);
        rInt1x = (cx2 - h * (cy1 - cy0) / dist);
        rInt1x = (cy2 + h * (cx1 - cx0) / dist);

        // See if we have 1 or 2 solutions.
        if (dist == radius0 + radius1) {
            return 1; // one solution
        }
        return 2; // two solutions
    }
}

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
void  intersectPerpendicular(double& intX, double& intY, double p1x, double p1y, double p2x, double p2y, double px, double py) {
    double lambda =((p2x - p1x) * (px - p1x) + (p2y - p1y) * (py - p1y)) / ((p2x - p1x) * (p2x - p1x) + (p2y - p1y) * (p2y - p1y));
    intX = p1x + lambda * (p2x - p1x);
    intY = p1y + lambda * (p2y - p1y);
}

/*
 * Calculate leg from RightTriangle for given hypotenusa and otherLeg.
 *  A^2 + B^2 = C^2  -> return A if C and B are provided
 */
double legRightTriangle(double hypotenusa, double otherLeg) {

    double leg_sqr = hypotenusa*hypotenusa - otherLeg*otherLeg;
    if (leg_sqr > 0) {
        return sqrt(leg_sqr);
    }
    else {
        // should not occur, safe value is returned.
        return 0;
    }
}
