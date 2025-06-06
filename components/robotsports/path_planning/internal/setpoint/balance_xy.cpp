#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "setpoint.hpp" // Assuming get_segments is defined in this header

std::tuple<std::vector<Segment_t>, std::vector<double>, std::vector<double>, std::vector<double>> balance_xy(
    std::vector<Segment_t>& segments, 
    const std::vector<double>& p0, 
    const std::vector<double>& v0, 
    const std::vector<double>& pe, 
    const std::vector<double>& ve, 
    const std::vector<double>& vm, 
    const std::vector<double>& am, 
    const std::vector<double>& dm) 
{
    double vmax_move = std::sqrt(vm[0] * vm[0] + vm[1] * vm[1]);
    double amax_move = std::sqrt(am[0] * am[0] + am[1] * am[1]);
    double dmax_move = std::sqrt(dm[0] * dm[0] + dm[1] * dm[1]);

    // define and initialzie with default
    std::vector<double> amax = am;
    std::vector<double> vmax = vm;
    std::vector<double> dmax = dm;

    // Balance xy-acceleration
    if (std::abs(pe[0] - p0[0]) > 1e-8 || std::abs(pe[1] - p0[1]) > 1e-8) {
        double a = 45;
        double stepsize = a / 2;
        int niter = 12;

        for (int i = 0; i < niter; ++i) {
            double max_downscale = 0.01;
            std::vector<double> A = { std::max(max_downscale, std::cos(a * M_PI / 180)), 
                                       std::max(max_downscale, std::sin(a * M_PI / 180)) };
            std::vector<double> amax = { amax_move * A[0], amax_move * A[1], am[2] };
            std::vector<double> vmax = { vmax_move * A[0], vmax_move * A[1], vm[2] };
            std::vector<double> dmax = { dmax_move * A[0], dmax_move * A[1], dm[2] };

            get_segments(segments, p0, v0, pe, ve, vmax, amax, dmax);

            if (segments[2].t[1] > segments[2].t[0]) {
                a += stepsize;
            } else {
                a -= stepsize;
            }
            stepsize /= 2;
        }
    }

    return std::make_tuple(segments, vmax, amax, dmax);
}

