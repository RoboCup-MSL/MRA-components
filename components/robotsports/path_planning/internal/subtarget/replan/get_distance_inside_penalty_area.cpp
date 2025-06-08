#include "get_distance_inside_penalty_area.hpp"
#include <cmath>

// get_distances_inside_penalty_area.py
double get_distance_inside_penalty_area(const Data& d, const std::vector<double>& pos) {
    // Calculate x and y distances
    double x = -std::abs(pos[0]) + (d.par["field_penalty_area"])[0] * 0.5;
    double y = std::abs(pos[1]) - (d.par["field_size"][1] * 0.5 - d.par["field_penalty_area"][1]);

    // Calculate distance
    double distance = std::fmax(0, std::fmin(x, y));

    return distance;
}

