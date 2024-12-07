#ifndef _MRA_LIBRARIES_GEOMETRY_POINT_HPP
#define _MRA_LIBRARIES_GEOMETRY_POINT_HPP

#include "datatypes/Point.pb.h"
#include <string>

namespace MRA::Geometry
{

class Point
{
private:
    constexpr static double DEFAULT_EQUALITY_TOLERANCE = 0.00001;
public:
    // direct read/write to the data members
    double x;
    double y;

    // constructors, destructor
    Point(double x_=0.0, double y_=0.0);
    Point(MRA::Datatypes::Point const &p);
    ~Point();

    // basic operations
    void reset();
    double size() const; // sqrt(x^2+y^2)

    // convert to protobuf object
    operator MRA::Datatypes::Point() const;

    // arithmetic operators
    virtual Point operator+(Point const&other) const;
    virtual Point& operator+=(Point const&other);
    virtual Point operator-(Point const&other) const;
    virtual Point& operator-=(Point const&other);
    virtual Point operator*(double f) const;
    virtual Point& operator*=(double f);
    virtual Point operator/(double f) const;
    virtual Point& operator/=(double f);


    // true if point is equal to point c within given tolerance
    bool equals( const Point& c, double tolerance=DEFAULT_EQUALITY_TOLERANCE) const;

    // get string representation of the point with 2 decimals
    std::string toString() const;

    // get distance to point c
    double distanceTo(const Point& c) const;

    // inner product with point p
    double inproduct( const Point& p) const;

    // get angle [rad] with to point p in the interval [-pi, +pi]
    double angle( const Point& p) const;


    /**
     * Normalizes the point such that its norm is 1.0 and direction unchanged.
     * @return Normalized point. When input point has length 0, the result is (1,0)
     */
    void normalize();

}; // class Point

} // namespace MRA::Geometry

#endif // #ifndef _MRA_LIBRARIES_GEOMETRY_POINT_HPP

