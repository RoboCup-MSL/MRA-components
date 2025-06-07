// adjust_to_penalty_area.cpp
#include "adjust_to_penalty_area.h"
#include <cmath>
#include <algorithm> // For std::abs

DType::TargetType adjust_to_penalty_area(const DType& d, DType::TargetType target) {
    double xpos = 0.5 * d.par.field_penalty_area[0];
    double ypos = d.par.field_size[1] * 0.5 - d.par.field_penalty_area[1];

    bool is_in_penalty_area = (std::abs(target.p[0]) < xpos) && (std::abs(target.p[1]) > ypos);

    if (is_in_penalty_area) {
        // Assuming d.input.robot.CPPA is a boolean member
        // You'll need to add CPPA to the RobotInput struct if it's not there
        // For now, I'll assume it's part of DType.input_robot.CPPA
        // If CPPA is 'can be in penalty area', then `not CPPA` means robot cannot be in penalty area
        if (!d.input_robot.CPPA) { // Assuming CPPA is a bool in RobotInput
            std::vector<double> distance = {xpos - std::abs(target.p[0]), std::abs(target.p[1]) - ypos};

            if (distance[0] < distance[1]) { // Clip to side
                target.p[0] = xpos * sign(target.p[0]);
                if (std::abs(target.p[1]) > (d.par.field_size[1] * 0.5)) {
                    target.p[1] = (d.par.field_size[1] * 0.5) * sign(target.p[1]);
                }
            } else { // Clip to front
                target.p[1] = ypos * sign(target.p[1]);
            }
        }
    }

    return target;
}
