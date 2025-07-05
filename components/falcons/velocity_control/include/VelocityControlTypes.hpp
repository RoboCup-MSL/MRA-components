#ifndef VELOCITY_CONTROL_TYPES_HPP
#define VELOCITY_CONTROL_TYPES_HPP

#include <cstdint>
#include <vector>
#include <string>
#include <cmath>

namespace VelocityControlTypes {

// Basic geometric types
struct Position2D {
    double x = 0.0;
    double y = 0.0;
    double rz = 0.0;  // rotation around z-axis

    Position2D() = default;
    Position2D(double x_, double y_, double rz_) : x(x_), y(y_), rz(rz_) {}

    // Transform operations
    Position2D transformFcsToRcs(const Position2D& robot_position) const;
    Position2D transformRcsToFcs(const Position2D& robot_position) const;

    // Operators
    Position2D operator+(const Position2D& other) const {
        return Position2D(x + other.x, y + other.y, rz + other.rz);
    }
    Position2D operator-(const Position2D& other) const {
        return Position2D(x - other.x, y - other.y, rz - other.rz);
    }
    Position2D operator*(double scalar) const {
        return Position2D(x * scalar, y * scalar, rz * scalar);
    }
};

struct Velocity2D {
    double x = 0.0;   // velocity in x direction
    double y = 0.0;   // velocity in y direction
    double rz = 0.0;  // angular velocity around z-axis

    Velocity2D() = default;
    Velocity2D(double x_, double y_, double rz_) : x(x_), y(y_), rz(rz_) {}

    // Transform operations
    Velocity2D transformFcsToRcs(const Position2D& robot_position) const;
    Velocity2D transformRcsToFcs(const Position2D& robot_position) const;

    // Operators
    Velocity2D operator+(const Velocity2D& other) const {
        return Velocity2D(x + other.x, y + other.y, rz + other.rz);
    }
    Velocity2D operator-(const Velocity2D& other) const {
        return Velocity2D(x - other.x, y - other.y, rz - other.rz);
    }
    Velocity2D operator*(double scalar) const {
        return Velocity2D(x * scalar, y * scalar, rz * scalar);
    }
};

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double rz = 0.0;

    Pose2D() = default;
    Pose2D(double x_, double y_, double rz_) : x(x_), y(y_), rz(rz_) {}
};

// Velocity Control specific types
struct PosVel {
    Position2D position;
    Velocity2D velocity;
    bool has_position = false;
    bool has_velocity = false;

    PosVel() = default;
    PosVel(const Position2D& pos, const Velocity2D& vel, bool has_pos = true, bool has_vel = true)
        : position(pos), velocity(vel), has_position(has_pos), has_velocity(has_vel) {}
};

struct XYRzLimits {
    double x = 0.0;
    double y = 0.0;
    double rz = 0.0;
    double y_forward = 0.0;   // Optional Y split for forward movement
    double y_backward = 0.0;  // Optional Y split for backward movement

    XYRzLimits() = default;
    XYRzLimits(double x_, double y_, double rz_) : x(x_), y(y_), rz(rz_), y_forward(y_), y_backward(y_) {}
    XYRzLimits(double x_, double y_fwd, double y_bwd, double rz_)
        : x(x_), y(0.0), rz(rz_), y_forward(y_fwd), y_backward(y_bwd) {}

    double getY(bool forward = true) const {
        if (y_forward > 0.0 || y_backward > 0.0) {
            return forward ? y_forward : y_backward;
        }
        return y;
    }
};

struct Limits {
    std::string name = "default";
    XYRzLimits max_velocity;
    XYRzLimits max_acceleration;
    XYRzLimits max_deceleration;
    XYRzLimits acceleration_threshold;

    Limits() = default;
    Limits(const XYRzLimits& vel, const XYRzLimits& acc)
        : max_velocity(vel), max_acceleration(acc), max_deceleration(acc) {}
};

// Configuration structures
struct SpgConfig {
    bool synchronize_rotation = false;
    double weight_factor_closed_loop_vel = 0.0;
    double weight_factor_closed_loop_pos = 0.7;
    double latency_offset = 0.0;
    bool convergence_workaround = false;
};

struct DribbleConfig {
    bool apply_limits_to_ball = true;
    double radius_robot_to_ball = 0.26;
};

struct DeadzoneConfig {
    bool enabled = true;
    double tolerance_xy = 0.03;  // [m]
    double tolerance_rz = 0.005; // [rad]
};

struct MotionProfile {
    std::string name = "default";
    Limits limits;
    SpgConfig spg_config;
    DribbleConfig dribble_config;
    DeadzoneConfig deadzone_config;
};

// Control modes
enum class ControlMode {
    INVALID,   // Invalid or uninitialized
    POSVEL,    // Position and velocity control
    POS_ONLY,  // Position only control
    VEL_ONLY,  // Velocity only control
    STOP       // Stop the robot
};

// Input structure - represents robot's current state and target
struct RobotState {
    Position2D position;    // Current position in FCS
    Velocity2D velocity;    // Current velocity in FCS
    bool active = true;     // Whether robot is active
    bool has_ball = false;  // Whether robot has the ball
};

struct WorldState {
    RobotState robot;
    double timestamp = 0.0;
};

struct Setpoint {
    Position2D position;
    Velocity2D velocity;
    bool has_position = false;
    bool has_velocity = false;
};

struct VelocityControlInput {
    WorldState world_state;          // Current world state
    Setpoint setpoint;               // Target setpoint in FCS
    int32_t motion_profile_id = 0;   // Motion profile to use
    double timestamp = 0.0;          // Timestamp in seconds
};

// Output structure
struct VelocityControlOutput {
    Velocity2D velocity_rcs;     // Output velocity in RCS (Robot Coordinate System)
    bool success = false;        // Whether the calculation was successful
    std::string error_message;   // Error message if not successful
};

// State structure for internal use
struct VelocityControlState {
    Position2D previous_position_setpoint_fcs;
    Velocity2D previous_velocity_setpoint_fcs;
    double last_calculation_time = 0.0;
    bool initialized = false;
    int num_algorithms_executed = 0;
    ControlMode control_mode = ControlMode::INVALID;
};

// SPG (SetPointGenerator) internal structures
struct SpgLimits {
    double vx = 2.0;
    double vy = 2.0;
    double vRz = 6.0;
    double ax = 3.0;
    double ay = 3.0;
    double aRz = 10.0;
};

// Parameters structure
struct VelocityControlParams {
    double dt = 0.025;                       // [seconds] timestep
    double timeout = 0.1;                    // [seconds] watchdog timeout
    SpgConfig spg;                           // SPG configuration
    DribbleConfig dribble;                   // Dribbling configuration
    DeadzoneConfig deadzone;                 // Deadzone configuration
    std::vector<Limits> limits;              // Motion profiles limits
    double calculation_frequency = 40.0;     // [Hz]

    VelocityControlParams() {
        // Add default motion profile
        Limits default_limits;
        default_limits.name = "default";
        default_limits.max_velocity = XYRzLimits(1.6, 1.6, 1.6, 2.0);
        default_limits.max_acceleration = XYRzLimits(1.0, 1.0, 1.0, 1.5);
        default_limits.max_deceleration = XYRzLimits(2.5, 2.5, 5.0);
        default_limits.acceleration_threshold = XYRzLimits(0.0, 0.0, 0.0);
        limits.push_back(default_limits);

        // Add "withBall" motion profile
        Limits with_ball_limits;
        with_ball_limits.name = "withBall";
        with_ball_limits.max_velocity = XYRzLimits(0.5, 1.4, 0.5, 2.5);
        with_ball_limits.max_acceleration = XYRzLimits(0.5, 1.0, 0.5, 3.0);
        with_ball_limits.max_deceleration = XYRzLimits(2.5, 2.5, 5.0);
        with_ball_limits.acceleration_threshold = XYRzLimits(0.0, 0.0, 0.0);
        limits.push_back(with_ball_limits);
    }
};

// Utility functions
double wrap_pi(double angle);
double project_angle_0_2pi(double angle);

} // namespace VelocityControlTypes

#endif // VELOCITY_CONTROL_TYPES_HPP
