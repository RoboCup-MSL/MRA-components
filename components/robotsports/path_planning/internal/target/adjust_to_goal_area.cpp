// adjust_to_goal_area.cpp
#include "adjust_to_goal_area.h"
#include <cmath>
#include <algorithm> // For std::abs

double sign(double x) {
    return (x > 0) - (x < 0);
}

DType::TargetType adjust_to_goal_area(const DType& d, DType::TargetType target) {
    double xpos = 0.5 * d.par.field_goal_area[0];
    double ypos = d.par.field_size[1] * 0.5 - d.par.field_goal_area[1];

    bool is_in_goal_area = std::abs(target.p[0]) < xpos && std::abs(target.p[1]) > ypos;

    if (is_in_goal_area) {
        std::vector<double> distance = {xpos - std::abs(target.p[0]), std::abs(target.p[1]) - ypos};

        if (distance[0] < distance[1]) { // Clip to side
            target.p[0] = xpos * sign(target.p[0]);
            if (std::abs(target.p[1]) > d.par.field_size[1] * 0.5) {
                target.p[1] = d.par.field_size[1] * 0.5 * sign(target.p[1]);
            }
        } else { // Clip to front
            target.p[1] = ypos * sign(target.p[1]);
        }
    }

    return target;
}