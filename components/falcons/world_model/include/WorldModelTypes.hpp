#ifndef WORLD_MODEL_TYPES_HPP
#define WORLD_MODEL_TYPES_HPP

#include <vector>
#include <chrono>

namespace falcons
{

struct Pose2D
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    Pose2D() = default;
    Pose2D(double x, double y, double theta) : x(x), y(y), theta(theta) {}
};

struct Velocity2D
{
    double vx = 0.0;
    double vy = 0.0;
    double vtheta = 0.0;

    Velocity2D() = default;
    Velocity2D(double vx, double vy, double vtheta) : vx(vx), vy(vy), vtheta(vtheta) {}
};

struct VisionLandmark
{
    enum Type
    {
        GOALPOST,
        LINE_POINT,
        CORNER
    };

    Type type;
    double azimuth = 0.0;      // angle from robot forward direction (rad)
    double distance = 0.0;     // distance from robot (m)
    double confidence = 0.0;   // confidence [0.0, 1.0]

    VisionLandmark() = default;
    VisionLandmark(Type type, double azimuth, double distance, double confidence)
        : type(type), azimuth(azimuth), distance(distance), confidence(confidence) {}
};

struct OdometryData
{
    std::chrono::system_clock::time_point timestamp;
    Velocity2D velocity;

    OdometryData() = default;
    OdometryData(const std::chrono::system_clock::time_point& timestamp, const Velocity2D& velocity)
        : timestamp(timestamp), velocity(velocity) {}
};

struct RobotPose
{
    std::chrono::system_clock::time_point timestamp;
    Pose2D pose;
    Velocity2D velocity;
    double confidence = 0.0;

    RobotPose() = default;
    RobotPose(const std::chrono::system_clock::time_point& timestamp, const Pose2D& pose,
              const Velocity2D& velocity, double confidence)
        : timestamp(timestamp), pose(pose), velocity(velocity), confidence(confidence) {}
};

struct LocalizationInput
{
    std::chrono::system_clock::time_point timestamp;
    std::vector<VisionLandmark> landmarks;
    OdometryData odometry;

    LocalizationInput() = default;
    LocalizationInput(const std::chrono::system_clock::time_point& timestamp,
                     const std::vector<VisionLandmark>& landmarks,
                     const OdometryData& odometry)
        : timestamp(timestamp), landmarks(landmarks), odometry(odometry) {}
};

struct WorldModelState
{
    std::chrono::system_clock::time_point timestamp;
    RobotPose robot_pose;
    // TODO: Add ball, teammates, opponents when implementing those components

    WorldModelState() = default;
    WorldModelState(const std::chrono::system_clock::time_point& timestamp, const RobotPose& robot_pose)
        : timestamp(timestamp), robot_pose(robot_pose) {}
};

} // namespace falcons

#endif // WORLD_MODEL_TYPES_HPP
