#ifndef ANGLE_HPP
#define ANGLE_HPP

#include "dribble_module.hpp" // For dribble function and DribbleData struct
#include "aim_at_target.hpp"  // Although not directly used for skillID, it's a related module
#include "shield.hpp"         // Although not directly used for skillID, it's a related module
#include <cmath>            // For fmod and other math operations

// Define a wrap function similar to setpoint.wrap
// This function needs to be defined or included from a C++ equivalent of setpoint.wrap
// For now, let's define a simple wrap function assuming it wraps angles to -PI to PI
double wrap_angle(double angle, double current_setpoint_angle);

// Consolidated Data structure for 'd' that can accommodate all skill IDs.
// In a real application, you might use polymorphism or more specialized structs.
// For this translation, we'll include all potentially needed fields.
struct SetData {
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
};

// A more generalized data structure for the main 'd' object
// This is a common pattern in robotics or game development where a global state 'd' is passed around.
struct GlobalData {
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
};

double set_angle(const GlobalData& d);

#endif // ANGLE_HPP