#include "intersect.hpp"
#include <cmath>


namespace MRA::Geometry
{

// Intersection of two line segments p and q.
// Optional take full lines (infinite length) instead of only segments.
// Return codes:
// * 0: no intersection
// * 1: intersection at a single point
// * 2: overlap of two segments
// * 3: one or both segments are degenerate
// Output point is only filled in for case 1.
int intersect(const MRA::Geometry::Point& p1, const MRA::Geometry::Point& p2,
              const MRA::Geometry::Point& q1, const MRA::Geometry::Point& q2,
              bool full_lines, MRA::Geometry::Point* intersection)
{
    // check for degenerate lines
    double len_p = (p2 - p1).size();
    double len_q = (q2 - q1).size();
    if (len_p * len_q == 0)
    {
        return 3; // at least one segment is degenerate
    }

    // p1 + t * (p2 - p1) = q1 + s * (q2 - q1)
    // t = (q1 - p1) x (q2 - q1) / (p2 - p1) x (q2 - q1)
    // s = (q1 - p1) x (p2 - p1) / (p2 - p1) x (q2 - q1)
    // x = p1 + t * (p2 - p1) = q1 + s * (q2 - q1)
    // if 0 <= t <= 1 and 0 <= s <= 1, then x is the intersection point

    double p2_p1_x_q2_q1 = (p2.x - p1.x) * (q2.y - q1.y) - (p2.y - p1.y) * (q2.x - q1.x);
    double q1_p1_x_q2_q1 = (q1.x - p1.x) * (q2.y - q1.y) - (q1.y - p1.y) * (q2.x - q1.x);
    double q1_p1_x_p2_p1 = (q1.x - p1.x) * (p2.y - p1.y) - (q1.y - p1.y) * (p2.x - p1.x);

    if (full_lines)
    {
        // If considering full lines, we don't need to check if t and s are within [0, 1]
        double t = q1_p1_x_q2_q1 / p2_p1_x_q2_q1;

        if (intersection != nullptr)
        {
            intersection->x = p1.x + t * (p2.x - p1.x);
            intersection->y = p1.y + t * (p2.y - p1.y);
        }
        return 1; // intersection at a single point
    }

    if (std::abs(p2_p1_x_q2_q1) < 1e-6)
    {
        // lines are parallel
        if (std::abs(q1_p1_x_q2_q1) < 1e-6)
        {
            // lines are collinear
            // check if segments overlap
            double lengthSquared = (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y);
            double t1 = ((q1.x - p1.x) * (p2.x - p1.x) + (q1.y - p1.y) * (p2.y - p1.y)) / lengthSquared;
            double t2 = ((q2.x - p1.x) * (p2.x - p1.x) + (q2.y - p1.y) * (p2.y - p1.y)) / lengthSquared;
            double t3 = ((p1.x - q1.x) * (q2.x - q1.x) + (p1.y - q1.y) * (q2.y - q1.y)) / lengthSquared;
            double t4 = ((p2.x - q1.x) * (q2.x - q1.x) + (p2.y - q1.y) * (q2.y - q1.y)) / lengthSquared;
            if ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1) || (t3 >= 0 && t3 <= 1) || (t4 >= 0 && t4 <= 1))
            {
                return 2; // segments overlap
            }
            return 0; // no intersection
        }
        return 0; // lines are parallel but not collinear
    }

    double t = q1_p1_x_q2_q1 / p2_p1_x_q2_q1;
    double s = q1_p1_x_p2_p1 / p2_p1_x_q2_q1;

    if (t >= 0 && t <= 1 && s >= 0 && s <= 1)
    {
        if (intersection != nullptr)
        {
            intersection->x = p1.x + t * (p2.x - p1.x);
            intersection->y = p1.y + t * (p2.y - p1.y);
        }
        return 1; // intersection at a single point
    }

    return 0; // no intersection
}


} // namespace MRA::Geometry
