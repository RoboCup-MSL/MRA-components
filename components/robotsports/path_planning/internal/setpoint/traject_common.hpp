#ifndef TRAJ_COMMON_H
#define TRAJ_COMMON_H

#include "setpoint.hpp" // For SegmentData and DOF

void combine_segment_data(const std::vector<SegmentData>& segment,
                          std::vector<double>& t_combined,
                          std::vector<double>& p_combined,
                          std::vector<double>& v_combined,
                          std::vector<double>& a_combined);

void traj_segment(const std::vector<SegmentData>& segment,
                  const std::vector<double>& time,
                  std::vector<double>& P_out,
                  std::vector<double>& V_out,
                  std::vector<double>& A_out,
                  std::vector<bool>& tseg_out);

#endif // TRAJ_COMMON_H