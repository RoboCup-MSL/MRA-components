#include "traj1.hpp"

Data traj1(Data d, const std::vector<SegmentData>& segment, double Ts) {
    int n = 1;
    std::vector<double> t_local(n);
    t_local[0] = 1 * Ts;

    int ndof = DOF;
    int nseg = segment.size();

    std::vector<double> P_all, V_all, A_all;
    std::vector<bool> tseg_all;
    traj_segment(segment, t_local, P_all, V_all, A_all, tseg_all);

    std::vector<std::array<int, DOF>> segment_id_output(n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < ndof; ++j) {
            int sum_val = 0;
            for (int k = 0; k < nseg; ++k) {
                sum_val += tseg_all[i * (ndof * nseg) + j * nseg + k];
            }
            segment_id_output[i][j] = sum_val;
        }
    }

    std::vector<int> final_ind(ndof);
    for (int j = 0; j < ndof; ++j) {
        final_ind[j] = (0) + j * n + segment_id_output[0][j] * n * ndof; // Since n=1, nvec[0]-1 = 0
    }

    d.traj.t[0] = t_local[0];

    for (int j = 0; j < ndof; ++j) {
        d.traj.p[0][j] = P_all[final_ind[j]];
        d.traj.v[0][j] = V_all[final_ind[j]];
        d.traj.a[0][j] = A_all[final_ind[j]];
        d.traj.segment_id[0][j] = segment_id_output[0][j];
    }

    return d;
}