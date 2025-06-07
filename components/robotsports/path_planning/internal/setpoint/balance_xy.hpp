#ifndef BALANCE_XY_H
#define BALANCE_XY_H

#include "setpoint.hpp"
#include "get_segments.hpp" // get_segments is called here
#include <tuple> // For std::tuple

std::tuple<std::vector<SegmentData>, std::array<double, DOF>, std::array<double, DOF>, std::array<double, DOF>>
balance_xy(std::vector<SegmentData> segment,
           const std::array<double, DOF>& p0,
           const std::array<double, DOF>& v0,
           const std::array<double, DOF>& pe,
           const std::array<double, DOF>& ve,
           const std::array<double, DOF>& vm_in,
           const std::array<double, DOF>& am_in,
           const std::array<double, DOF>& dm_in);

#endif // BALANCE_XY_H