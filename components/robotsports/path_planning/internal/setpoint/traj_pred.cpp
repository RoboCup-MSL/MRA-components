#include "traj_pred.hpp"

Data traj_predict(Data d, const std::vector<SegmentData>& segment) {
    int n = d.traj.p.size();
    std::vector<int> nvec(n);
    std::iota(nvec.begin(), nvec.end(), 1);

    std::vector<double> t_local(n);
    double Ts = d.par.Ts_predict;
    for (int i = 0; i < n; ++i) {
        t_local[i] = nvec[i] * Ts;
    }

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

    std::vector<int> final_ind(n * ndof);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < ndof; ++j) {
            final_ind[i * ndof + j] = (nvec[i] - 1) + j * n + segment_id_output[i][j] * n * ndof;
        }
    }

    for (int i = 0; i < n; ++i) {
        d.traj.t[i] = t_local[i];
        for (int j = 0; j < ndof; ++j) {
            d.traj.p[i][j] = P_all[final_ind[i * ndof + j]];
            d.traj.v[i][j] = V_all[final_ind[i * ndof + j]];
            d.traj.a[i][j] = A_all[final_ind[i * ndof + j]];
            d.traj.segment_id[i][j] = segment_id_output[i][j];
        }
    }

    if (d.input.robot.skillID == 1) { // Dribble
        for (int i = 0; i < n; ++i) {
            double v_magnitude_sq = d.traj.v[i][0] * d.traj.v[i][0] + d.traj.v[i][1] * d.traj.v[i][1];
            if (v_magnitude_sq > 1e-12) {
                d.traj.p[i][2] = std::atan2(-d.traj.v[i][0], d.traj.v[i][1]);
            }
        }
    }

    return d;
}