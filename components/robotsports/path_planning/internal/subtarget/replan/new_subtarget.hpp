#ifndef NEW_SUBTARGET_HPP
#define NEW_SUBTARGET_HPP

#include <map>
#include <vector>

// --- Structure Definitions (Simplified) ---
// In a real application, you would define more specific structs for 'par', 'setpoint', 'target', 'input', etc.
// For this translation, we'll use a nested map structure to mimic Python dictionaries.
// This is not the most efficient or type-safe C++ approach but serves for direct translation.

typedef struct Subtarget {
    std::vector<double> p; // position
    std::vector<double> v; // velocity
    std::vector<double> vmax; // max velocity
    std::vector<double> amax; // max acceleration
    std::vector<double> dmax; // max deceleration
    double eta; // time to reach subtarget
    int age;
    bool collisionfree;
    int violation_count;
    std::string action; // Assuming 'action' is a string
    std::vector<int> segment_id; // Assuming segment_id is a vector of ints
    std::map<int, std::map<char, std::vector<double>>> segment; // Mimics segment[2]['t'][0:2]
    std::vector<double> target; // New field for subtarget_target['target']
} Subtarget_t;

typedef struct RobotInput {
    int skillID;
    double CPBteam;
    bool quickstop_trigger;
    bool human_dribble_flag;
} RobotInput_t;

typedef struct Data {
    std::map<std::string, std::map<std::string, std::vector<double>>> par; // Parameters
    std::map<std::string, std::vector<double>> setpoint; // Current state
    std::map<std::string, std::vector<double>> target; // Target state
    RobotInput input_robot; // input['robot']
    Subtarget subtarget; // Current subtarget info
} Data_t;

#endif