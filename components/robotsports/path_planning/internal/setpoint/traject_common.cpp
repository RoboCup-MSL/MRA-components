#include "traject_common.hpp"

void combine_segment_data(const std::vector<SegmentData>& segment,
                          std::vector<double>& t_combined,
                          std::vector<double>& p_combined,
                          std::vector<double>& v_combined,
                          std::vector<double>& a_combined) {
    int nseg = segment.size();
    if (nseg == 0) return;

    int ndim = DOF;

    t_combined.assign(nseg * ndim, 0.0);
    p_combined.assign(nseg * ndim, 0.0);
    v_combined.assign(nseg * ndim, 0.0);
    a_combined.assign(nseg * ndim, 0.0);

    for (int i = 0; i < nseg; ++i) {
        for (int j = 0; j < ndim; ++j) {
            int idx = i * ndim + j;
            t_combined[idx] = segment[i].t[j];
            p_combined[idx] = segment[i].p[j];
            v_combined[idx] = segment[i].v[j];
            a_combined[idx] = segment[i].a[j];
        }
    }
}

void traj_segment(const std::vector<SegmentData>& segment,
                  const std::vector<double>& time,
                  std::vector<double>& P_out,
                  std::vector<double>& V_out,
                  std::vector<double>& A_out,
                  std::vector<bool>& tseg_out) {

    std::vector<double> t_combined, p_combined, v_combined, a_combined;
    combine_segment_data(segment, t_combined, p_combined, v_combined, a_combined);

    int n_time_points = time.size();
    int n_combined_dim = t_combined.size();

    P_out.assign(n_time_points * n_combined_dim, 0.0);
    V_out.assign(n_time_points * n_combined_dim, 0.0);
    A_out.assign(n_time_points * n_combined_dim, 0.0);
    tseg_out.assign(n_time_points * n_combined_dim, false);

    for (int i = 0; i < n_time_points; ++i) {
        for (int j = 0; j < n_combined_dim; ++j) {
            double t_rel = time[i] - t_combined[j];
            double vt = v_combined[j] * t_rel;
            double at = a_combined[j] * t_rel;

            P_out[i * n_combined_dim + j] = p_combined[j] + vt + 0.5 * at * t_rel;
            V_out[i * n_combined_dim + j] = v_combined[j] + at;
            A_out[i * n_combined_dim + j] = a_combined[j];

            tseg_out[i * n_combined_dim + j] = (time[i] > t_combined[j]);
        }
    }
}