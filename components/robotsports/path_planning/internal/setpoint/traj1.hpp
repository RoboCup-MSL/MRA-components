#ifndef TRAJ1_H
#define TRAJ1_H

#include "setpoint.hpp"
#include "traject_common.hpp" // For traj_segment and combine_segment_data

Data traj1(Data d, const std::vector<SegmentData>& segment, double Ts);

#endif // TRAJ1_H