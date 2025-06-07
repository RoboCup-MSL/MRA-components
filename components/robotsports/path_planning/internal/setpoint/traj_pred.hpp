#ifndef TRAJ_PRED_HPP
#define TRAJ_PRED_HPP

#include "setpoint.hpp"
#include "traject_common.hpp" // For traj_segment and combine_segment_data

Data traj_predict(Data d, const std::vector<SegmentData>& segment);

#endif // TRAJ_PRED_HPP