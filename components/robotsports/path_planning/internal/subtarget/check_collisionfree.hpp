#ifndef CHECK_COLLISIONFREE_H
#define CHECK_COLLISIONFREE_H

#include <vector>
#include <map>
#include <string>
#include <cmath> // For std::sqrt, std::abs, std::max, std::sin, std::cos, std::atan2, std::degrees, M_PI

// Assuming these structures are defined elsewhere or will be defined
struct SetpointData {
    // Define members based on your 'd' and 'subtarget' dictionary structure
    // For example:
    std::vector<double> p; // position
    std::vector<double> v; // velocity
    double vmax;
    double amax;
    std::vector<double> dmax_move;
    std::vector<double> dmax_rotate;
    int segment_id; // Added to match usage in the Python code
    std::map<std::string, std::vector<std::vector<double>>> segment; // Placeholder for segment data structure
    double eta; // Added to match usage in the Python code
    bool collisionfree; // Added to match usage in the Python code
    int violation_count; // Added to match usage in the Python code
    std::vector<double> target_p; // target position
    std::vector<double> target_v; // target velocity
};

struct TrajectoryData {
    // Define members for 'd['traj']'
    std::vector<std::vector<double>> p; // trajectory positions
    std::vector<std::vector<double>> v; // trajectory velocities
    std::vector<int> segment_id; // Added to match usage in the Python code
};

struct ObstacleData {
    // Define members for 'd['input']['obstacles']'
    std::vector<std::vector<double>> p; // obstacle positions
    std::vector<std::vector<double>> v; // obstacle velocities
    std::vector<double> r; // obstacle radii
    std::vector<bool> active; // Added to match usage in the Python code
};

struct InputData {
    // Define members for 'd['input']'
    ObstacleData obstacles;
    double robot_radius;
    std::vector<double> ball_p; // Added to match usage in the Python code
    std::vector<double> ball_v; // Added to match usage in the Python code
    struct RobotData {
        int skillID;
        double dist2ball_vs_opp;
        bool human_dribble_flag; // Added to match usage in the Python code
        std::vector<double> p; // Added to match usage in the Python code
        std::vector<double> v; // Added to match usage in the Python code
    } robot; // Added to match usage in the Python code
};

struct ParamsData {
    // Define members for 'd['par']'
    double robot_radius;
    double obstacle_vel_gain;
    double Ts_predict;
    // Add other parameters from d['par'] as needed
    std::vector<double> dmax_move; // Added to match usage in the Python code
    std::vector<double> dmax_rotate; // Added to match usage in the Python code
};

struct MainData {
    // This will hold the equivalent of the 'd' dictionary
    SetpointData setpoint;
    InputData input;
    ParamsData par;
    TrajectoryData traj; // This will be updated by traj_predict
    SetpointData target; // Added to match usage in the Python code
    SetpointData subtarget; // Added to match usage in the Python code
};

struct Violation {
    bool collisionfree;
    int count;
    double SubtargetAvoidPolygon;
    double obstacle;
    double field;
};

// Helper function declaration
Violation update_violation(Violation violation, const std::string& field, double violation_value);

// Main function declaration
std::tuple<SetpointData, std::vector<std::vector<double>>, std::vector<std::vector<std::vector<double>>>>
check_collisionfree(MainData& d, SetpointData subtarget, double obstacle_margin);

#endif // CHECK_COLLISIONFREE_H