#include "point.hpp"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <stdexcept>

using namespace MRA::Geometry;


// constructors, destructor
Point::Point(double x_, double y_)
{
    x = x_;
    y = y_;
}

Point::Point(MRA::Datatypes::Point const &p)
{
    x = p.x();
    y = p.y();
}

Point::operator MRA::Datatypes::Point() const
{
    MRA::Datatypes::Point result;
    result.set_x(x);
    result.set_y(y);
    return result;
}

Point::~Point()
{
}

// basic operations
void Point::reset()
{
    x = 0.0;
    y = 0.0;
}

double Point::size() const
{
    return hypot(x, y);
}    

Point Point::operator+(Point const &other) const
{
    Point result(x + other.x, y + other.y);
    return result;
}

Point& Point::operator+=(Point const &other)
{
    x += other.x;
    y += other.y;
    return *this;
}

Point Point::operator-(Point const &other) const
{
    Point result(x - other.x, y - other.y);
    return result;
}

Point& Point::operator-=(Point const &other)
{
    x -= other.x;
    y -= other.y;
    return *this;
}

Point Point::operator*(double f) const
{
    Point result(x * f, y * f);
    return result;
}

Point& Point::operator*=(double f)
{
    x *= f;
    y *= f;
    return *this;
}

Point Point::operator/(double f) const
{
    return operator*(1.0 / f);
}

Point& Point::operator/=(double f)
{
    return operator*=(1.0 / f);
}

bool Point::equals( const Point& c, double tolerance) const {
    return fabs(c.x - x) < tolerance and fabs(c.y - y) < tolerance;
}


std::string Point::toString() const {
    std::stringstream buffer;
    buffer << std::fixed << std::setprecision(2) << "x: " << x << " y: " << y;
    return buffer.str();
}

double Point::distanceTo(const Point& aCoordinate) const {
    return hypot(aCoordinate.x - x, aCoordinate.y - y);
}

double Point::inproduct( const Point& point) const {
    return this->x * point.x + this->y * point.y;
}

double Point::angle( const Point& point) const {
    return atan2(this->y - point.y, this->x - point.x);
}

/**
 * Normalizes the point such that its norm is 1.0 and direction unchanged.
 *
 * @return Normalized point. When input point has length 0, the result is (1,0)
 */
void Point::normalize() {
    double norm_res = size();
    if (fabs(norm_res) < Point::DEFAULT_EQUALITY_TOLERANCE) {
        throw std::range_error("normalize of vector with magnitude zero is not possible");
    } else {
        x = x / norm_res;
        y = y / norm_res;
    }
}


