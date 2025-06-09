#ifndef SUBTARGET_HPP
#define SUBTARGET_HPP

#include <vector>
#include <map>
#include <cmath> // For std::sqrt, std::pow, std::cos, std::deg2rad, M_PI
#include <eigen3/Eigen/Dense>

// Structure to mimic the relevant parts of the Python 'd' dictionary
typedef struct AimData {
    struct Target {
        Eigen::Vector3d p; // Position vector (x, y, angle)
    } target;
    struct Setpoint {
        Eigen::Vector3d p; // Position vector (x, y, angle)
    } setpoint;
    struct Subtarget {
        Eigen::Vector3d p; // Position vector (x, y, angle)
    } subtarget;
} AimData_t;



// Structure to mimic the relevant parts of the Python 'd' dictionary
typedef struct ShieldData {
    struct Input {
        struct Obstacles {
            Eigen::MatrixX2d p; // N x 2 matrix for obstacle positions (x, y)
            Eigen::VectorXi active; // Boolean array as an integer vector (0 or 1)
        } obstacles;
        struct Robot {
            int skillID; // Assuming skillID is an integer
        } robot;
    } input;
    struct Setpoint {
        Eigen::Vector3d p; // Position vector (x, y, angle)
        Eigen::Vector3d v; // Velocity vector (vx, vy, vz)
    } setpoint;
    struct Subtarget {
        Eigen::Vector3d p; // Position vector (x, y, angle)
    } subtarget;
} ShieldData_t;


// Structure to mimic the relevant parts of the Python 'd' dictionary
typedef struct DribbleData {
    struct Setpoint {
        Eigen::Vector3d v; // Velocity vector (x, y, z)
        Eigen::Vector3d p; // Position vector (x, y, angle) - in Python, p[2] was the angle
    } setpoint;
} DribbleData_t;

// Define necessary structs based on the Python 'd' dictionary and other objects
typedef struct Obstacles {
    std::vector<bool> active;
    std::vector<Eigen::Vector3d> p; // Position [x, y, z]
    std::vector<double> r;           // Radius
} Obstacles_t;

typedef struct Input {
    Obstacles obstacles;
    // Add other input fields as necessary, e.g., robot, ball
} Input_t;

typedef struct Target {
    Eigen::Vector3d p; // Position [x, y, z]
    // Add other target fields as necessary
} Target_t;

typedef struct Setpoint {
    Eigen::Vector3d p; // Position [x, y, z]
    Eigen::Vector3d v; // Velocity [vx, vy, vz]
    // Add other setpoint fields as necessary
} Setpoint_t;

typedef struct Parameters {
    double vmax_move;
    double robot_radius;
    double margin_replan;
    // Add other parameters as necessary, e.g., field_size, goalwidth, nattempts_replan, replan_uphill_distance
} Parameters_t;

typedef struct D_Struct { // Represents the 'd' dictionary in Python
    Input input;
    Target target;
    Setpoint setpoint;
    Parameters par;
} D_Struct_t;

typedef struct SubtargetCandidate {
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    // Add other relevant fields like 'score' if used for comparison
    double score = 0.0; // Example score, adjust as per update_best logic
} SubtargetCandidate_t;

typedef struct Best_Struct {
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    double score = -std::numeric_limits<double>::infinity(); // Initialize with a very low score
} Best_Struct_t;



// Consolidated Data structure for 'd' that can accommodate all skill IDs.
// In a real application, you might use polymorphism or more specialized structs.
// For this translation, we'll include all potentially needed fields.
typedef struct SetData {
    struct Input {
        struct Robot {
            int skillID;
        } robot;
        // Include other input fields if set_angle was to call aim, shield, etc.
    } input;
    struct Target {
        Eigen::Vector3d p; // Position vector (x, y, angle)
    } target;
    struct Setpoint {
        Eigen::Vector3d p; // Position vector (x, y, angle)
    } setpoint;
    // If other functions (aim, shield) were to be called, their required data would be here
    // For dribble, the Setpoint.v is needed, which is part of DribbleData.
    // For shield, Input.obstacles and Setpoint.v, Subtarget.p are needed.
    // To simplify, we can either pass specific structs or a large combined struct.
    // Let's make it a general data structure that can be passed to dribble, aim, shield etc.
    // It's more practical to have the specific functions take their own data types.
    // So, set_angle will take a generic struct and then cast/construct specific data structs.
    // However, since we are only calling dribble and using other parts of 'd' directly,
    // let's define a general structure that contains all necessary fields for 'set_angle'.
    // For example, if aim_at_target and shield were called by set_angle directly,
    // we would need their input fields too.
    struct Subtarget {
        Eigen::Vector3d p; // Position vector (x, y, angle)
    } subtarget;

    // To handle the `d` object as passed to `dribble`, `aim_at_target`, `shield`,
    // we can either have a very large struct or pass a generic map-like object.
    // Given the Python dictionary structure, it's often best to pass the 'd' object directly
    // and then extract what's needed by each function.
    // Let's create a comprehensive struct for SetData that can feed into the other functions.
    // This isn't ideal for large projects, but works for direct translation.
    // Alternatively, if `set_angle` were to directly call `dribble`, `aim_at_target`, `shield`,
    // we would need to pass in data structures that match those functions' inputs.
    // For this translation, we'll combine the fields necessary for `set_angle` and `dribble`.
    // If `aim` or `shield` were called by `set_angle`, their respective `Data` structs
    // would be populated and passed.
} SetData_t;

// A more generalized data structure for the main 'd' object
// This is a common pattern in robotics or game development where a global state 'd' is passed around.
typedef struct GlobalData {
    struct Input {
        struct Robot {
            int skillID;
        } robot;
        struct Obstacles {
            Eigen::MatrixX2d p;
            Eigen::VectorXi active;
        } obstacles;
    } input;
    struct Target {
        Eigen::Vector3d p;
    } target;
    struct Setpoint {
        Eigen::Vector3d p;
        Eigen::Vector3d v;
    } setpoint;
    struct Subtarget {
        Eigen::Vector3d p;
    } subtarget;
} GlobalData_t;


// Assuming these structures are defined elsewhere or will be defined
typedef struct SetpointData {
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
} SetpointData_t;

typedef struct TrajectoryData {
    // Define members for 'd['traj']'
    std::vector<std::vector<double>> p; // trajectory positions
    std::vector<std::vector<double>> v; // trajectory velocities
    std::vector<int> segment_id; // Added to match usage in the Python code
} TrajectoryData_t;

typedef struct ObstacleData {
    // Define members for 'd['input']['obstacles']'
    std::vector<std::vector<double>> p; // obstacle positions
    std::vector<std::vector<double>> v; // obstacle velocities
    std::vector<double> r; // obstacle radii
    std::vector<bool> active; // Added to match usage in the Python code
} ObstacleData_t;

typedef struct InputData {
    // Define members for 'd['input']'
    ObstacleData obstacles;
    double robot_radius;
    struct BallData {
        std::vector<double> p; // Added to match usage in the Python code
        std::vector<double> v; // Added to match usage in the Python code
    } ball;
    struct RobotData {
        int skillID;
        double dist2ball_vs_opp;
        bool human_dribble_flag; // Added to match usage in the Python code
        std::vector<double> p; // Added to match usage in the Python code
        std::vector<double> v; // Added to match usage in the Python code
    } robot; // Added to match usage in the Python code
} InputData_t;

typedef struct ParamsData {
    // Define members for 'd['par']'
    double robot_radius;
    double obstacle_vel_gain;
    double Ts_predict;
    // Add other parameters from d['par'] as needed
    std::vector<double> dmax_move; // Added to match usage in the Python code
    std::vector<double> dmax_rotate; // Added to match usage in the Python code
} ParamsData_t;

typedef struct MainData {
    // This will hold the equivalent of the 'd' dictionary
    SetpointData_t setpoint;
    InputData_t input;
    ParamsData_t par;
    TrajectoryData_t traj; // This will be updated by traj_predict
    SetpointData_t target; // Added to match usage in the Python code
    SetpointData_t subtarget; // Added to match usage in the Python code
} MainData_t;

typedef struct Violation {
    bool collisionfree;
    int count;
    double SubtargetAvoidPolygon;
    double obstacle;
    double field;
} Violation_t;

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




// #include "subtarget/replan/determine_setpoint_limits.hpp" // Assuming these are custom headers
// #include "subtarget/replan/quickstop_desired.hpp"
// #include "subtarget/replan/quickstop.hpp"
// #include "subtarget/replan/to_target.hpp"
// #include "subtarget/replan/new_subtarget.hpp"
// #include "subtarget/replan/new_subtarget_desired.hpp"
// #include "subtarget/angle/set.h" // Assuming this is set.h inside subtarget/angle

// Assuming MainData and SetpointData are defined in check_collisionfree.h or a common header

MainData set(MainData d);

#endif // SUBTARGET_HPP
