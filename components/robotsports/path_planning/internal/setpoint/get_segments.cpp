#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include "setpoint.hpp"

double wrap(double angle, double reference) {
    // keep angle between a certain range, e.g. between -pi and pi
    //
    while (angle > reference + M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle < reference - M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

Segment_t move_at_constant_vel(const Segment_t& segment_in, const std::vector<double>& p0, const std::vector<double>& v0, 
                               const std::vector<double>& t0, const std::vector<double>& dt) {
    Segment_t segment = segment_in;
    segment.dt = dt;
    std::transform(segment.dt.begin(), segment.dt.end(), segment.dt.begin(), [](double d) { return std::max(0.0, d); });
    segment.a = std::vector<double>(dt.size(), 0.0);
    for (size_t i = 0; i < segment.dt.size(); ++i) {
        segment.dt[i] += t0[i];
    }
    for (size_t i = 0; i < segment.p.size(); ++i) {
        segment.p[i] = p0[i] + v0[i] * segment.dt[i];
    }
    segment.v = v0;
    return segment;
}

Segment_t move_to_vel(const Segment_t& segment_in, const std::vector<double>& p0, const std::vector<double>& v0, const std::vector<double>& t0, 
                      const std::vector<double>& ve, const std::vector<double>& am, const std::vector<double>& dm) {
    Segment_t segment = segment_in;
    std::vector<double> acc(segment.p.size());
    for (size_t i = 0; i < ve.size(); ++i) {
        acc[i] = (ve[i] - v0[i] >= 0) ? am[i] : dm[i];
    }
    for (size_t i = 0; i < ve.size(); ++i) {
        segment.dt[i] = std::abs(ve[i] - v0[i]) / acc[i];
        segment.a[i] = std::copysign(acc[i], ve[i] - v0[i]);
        segment.t[i] = t0[i] + segment.dt[i];
        segment.p[i] = p0[i] + v0[i] * segment.dt[i] + 0.5 * segment.a[i] * segment.dt[i] * segment.dt[i];
        segment.v[i] = ve[i];
    }
    return segment;
}

std::pair<std::vector<double>, std::vector<double>> get_max_velocity(const Segment_t& segment_in, 
                                            const std::vector<double>& p0, const std::vector<double>& v0, 
                                            const std::vector<double>& pe, const std::vector<double>& ve, 
                                            const std::vector<double>& vm, 
                                            const std::vector<double>& am, const std::vector<double>& dm) {
    
    std::vector<double> t0(p0.size(), 0.0);
    auto segment = move_to_vel(segment_in, p0, v0, t0, ve, am, dm);
    std::vector<double> direction(pe.size());
    for (size_t i = 0; i < pe.size(); ++i) {
        direction[i] = (pe[i] - segment.p[i] > 0) ? 1 : -1;
    }

    std::vector<double> a(pe.size()), d(pe.size());
    for (size_t i = 0; i < pe.size(); ++i) {
        a[i] = direction[i] * am[i];
        d[i] = direction[i] * dm[i];
    }

    std::vector<double> acc_ratio(pe.size()), numerator(pe.size()), denom(pe.size()), v(pe.size()), v1(pe.size()), tmax(pe.size());
    for (size_t i = 0; i < pe.size(); ++i) {
        acc_ratio[i] = a[i] / d[i];
        numerator[i] = v0[i] * v0[i] + acc_ratio[i] * ve[i] * ve[i] + 2 * a[i] * (pe[i] - p0[i]);
        denom[i] = 1 + acc_ratio[i];
        v[i] = std::sqrt(std::max(0.0, numerator[i] / denom[i]));
        v1[i] = direction[i] * std::min(v[i], vm[i]);
        tmax[i] = std::max(0.0, (v[i] * v[i] - vm[i] * vm[i]) / (2 * dm[i] * vm[i]) + (v[i] * v[i] - vm[i] * vm[i]) / (2 * am[i] * vm[i]));
    }

    for (size_t i = 0; i < pe.size(); ++i) {
        if (std::abs(v0[i]) > vm[i]) {
            tmax[i] = (pe[i] - segment.p[i]) / vm[i] * direction[i];
        }
        if (std::abs(pe[i] - segment.p[i]) < 1e-8) {
            tmax[i] = 0;
            v1[i] = v0[i];
        }
    }

    return {v1, tmax};
}

void get_segments(std::vector<Segment_t>& segment, const std::vector<double>& p0, 
                   const std::vector<double>& v0, const std::vector<double>& pe, 
                   const std::vector<double>& ve, const std::vector<double>& vm, 
                   const std::vector<double>& am, const std::vector<double>& dm) {
    // Correction fo rotation
    auto pe_wrapped = pe;
    pe_wrapped[2] = wrap(pe_wrapped[2], p0[2]);
    
    // Determine maximum velocity
    auto [v1, tmax] = get_max_velocity(segment[0], p0, v0, pe, ve, vm, am, dm);
    
    // Move towards maximum velocity
    std::vector<double> t0(p0.size(), 0.0);
    segment[0] = move_to_vel(segment[1], p0, v0, t0, v1, am, dm);
    
    // Move at max constant velocity
    segment[1] = move_at_constant_vel(segment[1], segment[0].p, segment[0].v, segment[0].t, tmax);
    
    // Move towards end velocity/position
    segment[2] = move_to_vel(segment[2], segment[1].p, segment[1].v, segment[1].t, ve, am, dm);
    
    // Move at max constant velocity
    segment[3] = move_at_constant_vel(segment[3], segment[2].p, segment[2].v, segment[2].t, {1e10, 1e10, 1e10});
}
