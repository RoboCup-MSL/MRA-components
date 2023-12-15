#ifndef _MRA_LIBRARIES_GEOMETRY_ANGLES_HPP
#define _MRA_LIBRARIES_GEOMETRY_ANGLES_HPP


namespace MRA::Geometry
{

// wrap rotation/angle in radians to [0, 2pi) for standardized coordinates
double wrap_2pi(double rotation_in_radians);

// wrap rotation/angle in radians to [-pi, pi) for subsequent abs()-and-compare
double wrap_pi(double rotation_in_radians);

// calculate relative angle in radians [-pi, pi) in MSL-FCS, where 0 is facing opponent goal
// (which is 90 degrees rotated w.r.t. regular polar coordinate system)
// example: when robot at x=2 takes a kickoff, the angle towards ball is 0.5*pi
double calc_facing_angle_fcs(double from_x, double from_y, double to_x, double to_y);

// convert degrees to radians
double deg_to_rad(double degrees);


// convert radians to degrees
double rad_to_deg(double radians);

// get shortest angle between two angle [rad]
double min_angle(double start_angle, double end_angle);


} // namespace MRA::Geometry


#endif // #ifndef _MRA_LIBRARIES_GEOMETRY_ANGLES_HPP

