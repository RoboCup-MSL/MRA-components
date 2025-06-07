#ifndef SETPOINT_HPP
#define SETPOINT_HPP 1

#include <vector>
#include <array>
#include <cmath>
#include <numeric>   // For std::iota
#include <algorithm> // For std::max, std::min, std::clamp

// Define degrees of freedom (e.g., x, y, rotation)
const int DOF = 3;

// Represents a single segment in the trajectory
struct SegmentData {
    std::array<double, DOF> t;
    std::array<double, DOF> p;
    std::array<double, DOF> v;
    std::array<double, DOF> a;
    std::array<double, DOF> dt;
};

// Simplified representation of the 'd' dictionary for relevant fields
struct Data {
    struct Parameters {
        std::array<double, 2> field_size; // [width, length]
        double field_border_margin;
        double dmax_move;
        double dmax_rotate;
        double Ts;
        double Ts_predict;
        double technical_area_width;
    } par;

    struct Input {
        struct Robot {
            std::array<double, DOF> p; // position
            std::array<double, DOF> v; // velocity
            std::array<double, DOF> IMU_orientation; // [pitch, roll, yaw] or similar
            int skillID;
        } robot;
    } input;

    struct Subtarget {
        std::array<double, DOF> p;
        std::array<double, DOF> v;
        std::array<double, DOF> vmax;
        std::array<double, DOF> amax;
        bool automatic_substitution_flag;
    } subtarget;

    struct Setpoint {
        std::array<double, DOF> p;
        std::array<double, DOF> v;
        std::array<double, DOF> a;
    } setpoint;

    struct Auxiliary {
        std::vector<SegmentData> segment; // List of SegmentData
    } aux;

    struct Trajectory {
        std::vector<double> t; // This should be a 1D vector of times
        std::vector<std::array<double, DOF>> p; // N x DOF matrix
        std::vector<std::array<double, DOF>> v; // N x DOF matrix
        std::vector<std::array<double, DOF>> a; // N x DOF matrix
        std::vector<std::array<int, DOF>> segment_id; // N x DOF matrix
    } traj;
};

// Helper for numpy.linalg.norm for a DOF-element array
inline double norm(const std::array<double, DOF>& vec) {
    double sum_sq = 0.0;
    for (double val : vec) {
        sum_sq += val * val;
    }
    return std::sqrt(sum_sq);
}

// Helper for numpy.linalg.norm for a 2-element array (specifically for XY plane)
inline double norm_xy(const std::array<double, DOF>& vec) {
    return std::sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
}

Data set(Data d);

#endif