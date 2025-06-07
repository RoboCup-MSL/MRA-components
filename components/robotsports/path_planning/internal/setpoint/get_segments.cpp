#include "get_segments.hpp"

SegmentData move_at_constant_vel(SegmentData segment,
                                 const std::array<double, DOF>& p0,
                                 const std::array<double, DOF>& v0,
                                 const std::array<double, DOF>& t0,
                                 const std::array<double, DOF>& dt) {
    for (int i = 0; i < DOF; ++i) {
        segment.dt[i] = std::max(0.0, dt[i]);
        segment.a[i] = 0.0;
        segment.t[i] = t0[i] + segment.dt[i];
        segment.p[i] = p0[i] + v0[i] * segment.dt[i];
        segment.v[i] = v0[i];
    }
    return segment;
}

SegmentData move_to_vel(SegmentData segment,
                        const std::array<double, DOF>& p0,
                        const std::array<double, DOF>& v0,
                        const std::array<double, DOF>& t0,
                        const std::array<double, DOF>& ve,
                        const std::array<double, DOF>& am,
                        const std::array<double, DOF>& dm) {
    for (int i = 0; i < DOF; ++i) {
        double diff_v = ve[i] - v0[i];
        double acc_val = (diff_v >= 0) ? am[i] : dm[i];
        segment.dt[i] = std::abs(diff_v) / acc_val;
        segment.a[i] = std::copysign(acc_val, diff_v);
        segment.t[i] = t0[i] + segment.dt[i];
        segment.p[i] = p0[i] + v0[i] * segment.dt[i] + 0.5 * segment.a[i] * segment.dt[i] * segment.dt[i];
        segment.v[i] = ve[i];
    }
    return segment;
}

std::pair<std::array<double, DOF>, std::array<double, DOF>> get_max_speed(SegmentData segment_in,
                                                                          const std::array<double, DOF>& p0,
                                                                          const std::array<double, DOF>& v0,
                                                                          const std::array<double, DOF>& pe,
                                                                          const std::array<double, DOF>& ve,
                                                                          const std::array<double, DOF>& vm,
                                                                          const std::array<double, DOF>& am,
                                                                          const std::array<double, DOF>& dm) {
    SegmentData segment = segment_in; // Make a mutable copy

    segment = move_to_vel(segment, p0, v0, {0.0, 0.0, 0.0}, ve, am, dm);
    std::array<double, DOF> direction;
    std::array<double, DOF> a_local;
    std::array<double, DOF> d_local;
    std::array<double, DOF> acc_ratio;
    std::array<double, DOF> numerator;
    std::array<double, DOF> denom;
    std::array<double, DOF> v;
    std::array<double, DOF> v1;
    std::array<double, DOF> tmax;

    for (int i = 0; i < DOF; ++i) {
        direction[i] = std::copysign(1.0, pe[i] - segment.p[i]);
        a_local[i] = direction[i] * am[i];
        d_local[i] = direction[i] * dm[i];
        acc_ratio[i] = a_local[i] / d_local[i];
        numerator[i] = v0[i] * v0[i] + acc_ratio[i] * ve[i] * ve[i] + 2 * a_local[i] * (pe[i] - p0[i]);
        denom[i] = 1 + acc_ratio[i];
        v[i] = std::sqrt(std::max(0.0, numerator[i] / denom[i]));
    }

    for (int i = 0; i < DOF; ++i) {
        v1[i] = direction[i] * std::min(v[i], vm[i]);
    }

    for (int i = 0; i < DOF; ++i) {
        if (vm[i] > 1e-9) { // Avoid division by zero
            tmax[i] = std::max(0.0, (v[i] * v[i] - vm[i] * vm[i]) / (2 * dm[i] * vm[i]) + (v[i] * v[i] - vm[i] * vm[i]) / (2 * am[i] * vm[i]));
        } else {
             tmax[i] = 0.0;
        }
    }

    for (int i = 0; i < DOF; ++i) {
        if (std::abs(v0[i]) > vm[i]) {
            if (vm[i] > 1e-9) {
                tmax[i] = (pe[i] - segment.p[i]) / vm[i] * direction[i];
            } else {
                tmax[i] = 0.0;
            }
        }
    }

    for (int i = 0; i < DOF; ++i) {
        if (std::abs(pe[i] - segment.p[i]) < 1e-8) {
            tmax[i] = 0.0;
            v1[i] = v0[i];
        }
    }

    return {v1, tmax};
}

std::vector<SegmentData> get_segments(std::vector<SegmentData> segment_list,
                                      const std::array<double, DOF>& p0,
                                      const std::array<double, DOF>& v0,
                                      std::array<double, DOF> pe, // Passed by value because pe[2] is modified
                                      const std::array<double, DOF>& ve,
                                      const std::array<double, DOF>& vm,
                                      const std::array<double, DOF>& am,
                                      const std::array<double, DOF>& dm) {
    if (segment_list.size() < 4) {
        segment_list.resize(4);
    }

    pe[2] = wrap(pe[2], p0[2]);

    std::array<double, DOF> v1;
    std::array<double, DOF> tmax;
    std::tie(v1, tmax) = get_max_speed(segment_list[0], p0, v0, pe, ve, vm, am, dm);

    segment_list[0] = move_to_vel(segment_list[0], p0, v0, {0.0, 0.0, 0.0}, v1, am, dm);

    segment_list[1] = move_at_constant_vel(segment_list[1], segment_list[0].p, segment_list[0].v, segment_list[0].t, tmax);

    segment_list[2] = move_to_vel(segment_list[2], segment_list[1].p, segment_list[1].v, segment_list[1].t, ve, am, dm);

    segment_list[3] = move_at_constant_vel(segment_list[3], segment_list[2].p, segment_list[2].v, segment_list[2].t, {1e10, 1e10, 1e10});

    return segment_list;
}