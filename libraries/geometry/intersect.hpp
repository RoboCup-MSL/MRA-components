#ifndef _MRA_LIBRARIES_GEOMETRY_INTERSECT_HPP
#define _MRA_LIBRARIES_GEOMETRY_INTERSECT_HPP

#include "point.hpp"


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
              bool full_lines, MRA::Geometry::Point* intersection);

} // namespace MRA::Geometry

#endif // #ifndef _MRA_LIBRARIES_GEOMETRY_INTERSECT_HPP

