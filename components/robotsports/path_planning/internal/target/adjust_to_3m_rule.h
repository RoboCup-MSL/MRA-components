// adjust_to_3m_rule.h
#ifndef ADJUST_TO_3M_RULE_H
#define ADJUST_TO_3M_RULE_H

#include <vector>
#include <cmath>
#include <numeric> // For std::iota

// Assuming a structure for 'd' and 'target' that mimics the Python dictionary
// You might need to adjust these structures based on your actual data representation in C++
struct RobotInput {
    std::vector<int> skillID;
    std::vector<double> cpb_poi_xy;
    int human_dribble_flag;
    bool CPPA; // can not be in penaly area ?
    std::vector<double> target;
    std::vector<double> target_vel;
};

struct ObstacleInput {
    std::vector<bool> active;
    std::vector<double> p; // Flattened 2D array [x1, y1, x2, y2, ...]
    std::vector<double> r;
};

struct DParameters {
    double robot_radius;
    double ball_radius;
    std::vector<double> field_size; // [width, height]
    std::vector<double> field_goal_area; // [width, height]
    std::vector<double> field_penalty_area; // [width, height]
    double field_border_margin;
    double technical_area_width;
};

struct Subtarget {
    int automatic_substitution_flag;
};

struct DType {
    RobotInput input_robot;
    ObstacleInput input_obstacles;
    DParameters par;
    Subtarget subtarget;
    struct TargetType {
        std::vector<double> p; // Target position [x, y, orientation]
        std::vector<double> v; // Target velocity
        int eta;
    } target;
};

DType::TargetType adjust_to_3m_rule(const DType& d, DType::TargetType target);

#endif // ADJUST_TO_3M_RULE_H