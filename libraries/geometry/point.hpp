#ifndef _MRA_LIBRARIES_GEOMETRY_POINT_HPP
#define _MRA_LIBRARIES_GEOMETRY_POINT_HPP

#include "datatypes/Point.pb.h"
#include <string>

namespace MRA::Geometry
{

class Point
{
private:
    static const double EQUALITY_TOLERANCE;
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


    bool equals( const Point& c, double tolerance=EQUALITY_TOLERANCE) const;
    std::string toString() const;
    double distanceTo(const Point& c) const;
    double inproduct( const Point& point) const;
    double angle( const Point& point) const;
    void normalize();

}; // class Point

} // namespace MRA::Geometry

#endif // #ifndef _MRA_LIBRARIES_GEOMETRY_POINT_HPP

