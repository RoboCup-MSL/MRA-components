
#ifndef GET_SEGMENTS_H
#define GET_SEGMENTS_H

#include "setpoint.hpp"
#include "wrap.hpp"
#include <utility> // For std::pair, std::tie

SegmentData move_at_constant_vel(SegmentData segment,
                                 const std::array<double, DOF>& p0,
                                 const std::array<double, DOF>& v0,
                                 const std::array<double, DOF>& t0,
                                 const std::array<double, DOF>& dt);

SegmentData move_to_vel(SegmentData segment,
                        const std::array<double, DOF>& p0,
                        const std::array<double, DOF>& v0,
                        const std::array<double, DOF>& t0,
                        const std::array<double, DOF>& ve,
                        const std::array<double, DOF>& am,
                        const std::array<double, DOF>& dm);

std::pair<std::array<double, DOF>, std::array<double, DOF>> get_max_speed(SegmentData segment_in,
                                                                          const std::array<double, DOF>& p0,
                                                                          const std::array<double, DOF>& v0,
                                                                          const std::array<double, DOF>& pe,
                                                                          const std::array<double, DOF>& ve,
                                                                          const std::array<double, DOF>& vm,
                                                                          const std::array<double, DOF>& am,
                                                                          const std::array<double, DOF>& dm);

std::vector<SegmentData> get_segments(std::vector<SegmentData> segment_list,
                                      const std::array<double, DOF>& p0,
                                      const std::array<double, DOF>& v0,
                                      std::array<double, DOF> pe,
                                      const std::array<double, DOF>& ve,
                                      const std::array<double, DOF>& vm,
                                      const std::array<double, DOF>& am,
                                      const std::array<double, DOF>& dm);

#endif // GET_SEGMENTS_H