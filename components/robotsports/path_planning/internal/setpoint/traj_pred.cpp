#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <map>

using namespace std;

struct Trajectory {
    vector<double> t;
    vector<vector<double>> p;
    vector<vector<double>> v;
    vector<vector<double>> a;
    vector<vector<int>> segment_id;
};

struct Input {
    struct Robot {
        int skillID;
    } robot;
};

struct Data {
    Trajectory traj;
    Input input;
    struct Parameters {
        double Ts_predict;
    } par;
};

vector<double> arange(int start, int end) {
    vector<double> result(end - start);
    iota(result.begin(), result.end(), start);
    return result;
}

Data traj_predict(Data d, const std::vector<std::map<string, std::vector<double>>>& segment) {
    // Equidistant time vector
    int n = d.traj.p.size();
    vector<double> nvec = arange(1, n + 1);
    double Ts = d.par.Ts_predict;
    vector<double> t(n);
    for (int i = 0; i < n; ++i) {
        t[i] = nvec[i] * Ts;
    }

    // Preparations
    int ndof = segment[0].at("p").size();
    int nseg = segment.size();

    // Get time response for all segments
    vector<vector<double>> P, V, A;
    vector<vector<bool>> tseg;
    tie(P, V, A, tseg) = traj_segment(segment, t);

    // Determine active segment at each time instance
    vector<int> segment_id(n);
    for (int i = 0; i < n; ++i) {
        segment_id[i] = accumulate(tseg[i].begin(), tseg[i].end(), 0);
    }

    // Determine correct segment sample selection
    vector<int> ind(n * ndof);
    iota(ind.begin(), ind.end(), 0);  // Initialize first segment
    for (int i = 0; i < n; ++i) {
        ind[i] += segment_id[i] * n * ndof;  // Update indices for different segments
    }

    // Set output
    d.traj.t = t;  // Adjusting index as Python is 0-based
    for (int i = 0; i < n; ++i) {
        d.traj.p[i] = P[ind[i]];
        d.traj.v[i] = V[ind[i]];
        d.traj.a[i] = A[ind[i]];
        d.traj.segment_id[i] = vector<int>(1, segment_id[i]);
    }

    // Adjust orientation for dribble skill
    if (d.input.robot.skillID == 1) {  // Dribble
        for (int i = 0; i < n; ++i) {
            if (d.traj.v[i][0] * d.traj.v[i][0] + d.traj.v[i][1] * d.traj.v[i][1] > 1e-12) {
                d.traj.p[i][2] = atan2(-d.traj.v[i][0], d.traj.v[i][1]);
            }
        }
    }

    return d;
}

tuple<vector<vector<double>>, vector<vector<double>>, vector<vector<double>>, vector<vector<bool>>>
traj_segment(const vector<map<string, vector<double>>>& segment, const vector<double>& time) {
    // Get combined segment data
    auto [t, p, v, a] = combine_segment_data(segment);

    // Auxiliary variables
    vector<vector<double>> t_rel(time.size(), vector<double>(t.size()));
    for (size_t i = 0; i < time.size(); ++i) {
        for (size_t j = 0; j < t.size(); ++j) {
            t_rel[i][j] = time[i] - t[j];  // Broadcasting for time difference
        }
    }
    vector<vector<double>> vt(t_rel.size(), vector<double>(t_rel[0].size()));
    vector<vector<double>> at(t_rel.size(), vector<double>(t_rel[0].size()));
    for (size_t i = 0; i < t_rel.size(); ++i) {
        for (size_t j = 0; j < t_rel[0].size(); ++j) {
            vt[i][j] = v[j] * t_rel[i][j];  // Velocity contribution
            at[i][j] = a[j] * t_rel[i][j];  // Acceleration contribution
        }
    }

    // Compute setpoints
    vector<vector<double>> P(t.size(), vector<double>(p[0].size()));
    vector<vector<double>> V(t.size(), vector<double>(v.size()));
    vector<vector<double>> A(t.size(), vector<double>(a.size()));
    for (size_t i = 0; i < t.size(); ++i) {
        for (size_t j = 0; j < p[0].size(); ++j) {
            P[i][j] = p[j] + vt[i][j] + 0.5 * at[i][j] * t_rel[i][j];  // Position setpoints
        }
        for (size_t j = 0; j < v.size(); ++j) {
            V[i][j] = v[j] + at[i][j];  // Velocity setpoints
        }
    }
    for (size_t i = 0; i < t.size(); ++i) {
        A[i] = a;  // Acceleration setpoints
    }

    // Determine segment timing
    vector<vector<bool>> tseg(t.size(), vector<bool>(t.size()));
    for (size_t i = 0; i < t.size(); ++i) {
        for (size_t j = 0; j < t.size(); ++j) {
            tseg[i][j] = time[i] > t[j];  // Active segment indicator
        }
    }

    return make_tuple(P, V, A, tseg);
}

tuple<vector<double>, vector<vector<double>>, vector<vector<double>>, vector<vector<double>>>
combine_segment_data(const vector<map<string, vector<double>>>& segment) {
    int nseg = segment.size();
    int ndim = segment[0].at("p").size();

    vector<double> t(nseg * ndim);
    vector<vector<double>> p(nseg * ndim, vector<double>(ndim));
    vector<vector<double>> v(nseg * ndim, vector<double>(ndim));
    vector<vector<double>> a(nseg * ndim, vector<double>(ndim));

    int ind = 0;
    for (int i = 0; i < nseg; ++i) {
        for (int j = 0; j < ndim; ++j) {
            t[ind] = segment[i].at("t")[j];
            p[ind] = segment[i].at("p")[j];
            v[ind] = segment[i].at("v")[j];
            a[ind] = segment[i].at("a")[j];
            ind++;
        }
    }

    return make_tuple(t, p, v, a);
}

