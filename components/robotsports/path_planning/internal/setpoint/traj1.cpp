#include <iostream>
#include <vector>
#include <array>
#include <numeric>
#include <map>
#include "setpoint.hpp"

std::tuple<std::vector<double>, std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<bool>>> traj_segment(const std::vector<std::map<std::string, std::vector<double>>>& segment, const std::vector<double>& time) {
    // Get combined segment data
    auto [t, p, v, a] = combine_segment_data(segment);

    // Auxiliary variables
    std::vector<std::vector<double>> t_rel(time.size(), std::vector<double>(t.size()));
    for (size_t i = 0; i < time.size(); ++i) {
        for (size_t j = 0; j < t.size(); ++j) {
            t_rel[i][j] = time[i] - t[j];  // Broadcasting for time difference
        }
    }

    // Velocity and acceleration contribution
    std::vector<std::vector<double>> vt(time.size(), std::vector<double>(p[0].size()));
    std::vector<std::vector<double>> at(time.size(), std::vector<double>(p[0].size()));
    for (size_t i = 0; i < time.size(); ++i) {
        for (size_t j = 0; j < p[0].size(); ++j) {
            vt[i][j] = v[j] * t_rel[i][j];
            at[i][j] = a[j] * t_rel[i][j];
        }
    }

    // Compute setpoints
    std::vector<std::vector<double>> P(time.size(), std::vector<double>(p[0].size()));
    std::vector<std::vector<double>> V(time.size(), std::vector<double>(v.size()));
    std::vector<std::vector<double>> A(time.size(), std::vector<double>(a.size()));
    for (size_t i = 0; i < time.size(); ++i) {
        for (size_t j = 0; j < p[0].size(); ++j) {
            P[i][j] = p[j] + vt[i][j] + 0.5 * at[i][j] * t_rel[i][j];  // Position setpoints
        }
    }

    for (size_t i = 0; i < time.size(); ++i) {
        for (size_t j = 0; j < v.size(); ++j) {
            V[i][j] = v[j] + at[i][j];  // Velocity setpoints
        }
    }

    A = std::vector<std::vector<double>>(time.size(), a);  // Acceleration setpoints

    // Determine segment timing
    std::vector<std::vector<bool>> tseg(time.size(), std::vector<bool>(t.size()));
    for (size_t i = 0; i < time.size(); ++i) {
        for (size_t j = 0; j < t.size(); ++j) {
            tseg[i][j] = time[i] > t[j];  // Active segment indicator
        }
    }

    return {P, V, A, tseg};
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>> combine_segment_data(const std::vector<std::map<std::string, std::vector<double>>>& segment) {
    size_t nseg = segment.size();
    size_t ndim = segment[0]["p"].size();

    std::vector<double> t(nseg * ndim);
    std::vector<double> p(nseg * ndim);
    std::vector<double> v(nseg * ndim);
    std::vector<double> a(nseg * ndim);

    size_t ind = 0;
    for (size_t i = 0; i < nseg; ++i) {
        for (size_t j = 0; j < ndim; ++j) {
            t[ind] = segment[i]["t"][j];
            p[ind] = segment[i]["p"][j];
            v[ind] = segment[i]["v"][j];
            a[ind] = segment[i]["a"][j];
            ind++;
        }
    }

    return {t, p, v, a};
}

Traject traj1(Traject traject, const std::vector<std::map<std::string, std::vector<double>>>& segment, double Ts) {
    // Equidistant time vector
    std::vector<double> nvec = {1};
    std::vector<double> t = {nvec[0] * Ts};

    // Preparations
    size_t ndof = segment[0]["p"].size();  // Number of degrees of freedom
    size_t nseg = segment.size();  // Number of segments

    // Get time response for all segments
    auto [P, V, A, tseg] = traj_segment(segment, t);

    // Determine active segment at each time instance
    std::vector<int> segment_id(nseg);
    for (size_t i = 0; i < nseg; ++i) {
        segment_id[i] = std::accumulate(tseg[i].begin(), tseg[i].end(), 0);
    }

    // Determine correct segment sample selection
    std::vector<size_t> ind(ndof);
    for (size_t i = 0; i < ndof; ++i) {
        ind[i] = nvec[0] - 1 + i;  // Initialize first segment
        ind[i] += segment_id[i] * ndof;  // Update if in other segments
    }

    // Set output
    traject.t[nvec[0] - 1] = t[0];
    for (size_t i = 0; i < ndof; ++i) {
        traject.p[nvec[0] - 1][i] = P[ind[i]];
        traject.v[nvec[0] - 1][i] = V[ind[i]];
        traject.a[nvec[0] - 1][i] = A[ind[i]];
        traject.segment_id[nvec[0] - 1][i] = segment_id[i];
    }

    return traject;
}

