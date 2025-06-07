#include "balance_xy.hpp"

std::tuple<std::vector<SegmentData>, std::array<double, DOF>, std::array<double, DOF>, std::array<double, DOF>>
balance_xy(std::vector<SegmentData> segment,
           const std::array<double, DOF>& p0,
           const std::array<double, DOF>& v0,
           const std::array<double, DOF>& pe,
           const std::array<double, DOF>& ve,
           const std::array<double, DOF>& vm_in,
           const std::array<double, DOF>& am_in,
           const std::array<double, DOF>& dm_in) {

    double vmax_move = norm_xy(vm_in);
    double amax_move = norm_xy(am_in);
    double dmax_move = norm_xy(dm_in);

    std::array<double, DOF> amax_out = am_in;
    std::array<double, DOF> vmax_out = vm_in;
    std::array<double, DOF> dmax_out = dm_in;

    bool xy_diff_significant = false;
    if (std::abs(pe[0] - p0[0]) > 1e-8 || std::abs(pe[1] - p0[1]) > 1e-8) {
        xy_diff_significant = true;
    }

    if (xy_diff_significant) {
        double a_angle = 45.0; // in degrees
        double stepsize = a_angle / 2.0;
        int niter = 12;

        std::array<double, DOF> amax_current;
        std::array<double, DOF> vmax_current;
        std::array<double, DOF> dmax_current;

        for (int _ = 0; _ < niter; ++_) {
            double max_downscale = 0.01;
            std::array<double, 2> A;
            A[0] = std::max(max_downscale, std::cos(a_angle * M_PI / 180.0));
            A[1] = std::max(max_downscale, std::sin(a_angle * M_PI / 180.0));

            amax_current[0] = amax_move * A[0];
            amax_current[1] = amax_move * A[1];
            amax_current[2] = am_in[2];

            vmax_current[0] = vmax_move * A[0];
            vmax_current[1] = vmax_move * A[1];
            vmax_current[2] = vm_in[2];

            dmax_current[0] = dmax_move * A[0];
            dmax_current[1] = dmax_move * A[1];
            dmax_current[2] = dm_in[2];

            segment = get_segments(segment, p0, v0, pe, ve, vmax_current, amax_current, dmax_current);

            if (segment[2].t[1] > segment[2].t[0]) {
                a_angle += stepsize;
            } else {
                a_angle -= stepsize;
            }
            stepsize /= 2.0;
        }
        amax_out = amax_current;
        vmax_out = vmax_current;
        dmax_out = dmax_current;

    }

    return {segment, vmax_out, amax_out, dmax_out};
}