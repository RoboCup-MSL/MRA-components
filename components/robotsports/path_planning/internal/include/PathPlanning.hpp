/*
 * PathPlanning.hpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#ifndef PATHPLANNING_HPP_
#define PATHPLANNING_HPP_

#include "../../RobotsportsPathPlanning_datatypes.hpp"

#include <vector>
#include "geometry.hpp"

enum class BoundaryOptionEnum
{
    ALLOW,
    STOP_AND_PASS,
    STOP_AND_FAIL,
    CLIP
};


enum class robotStatusEnum
{
    UNKNOWN,
    INITIALIZING, // before having a valid loc
    INPLAY,       // valid loc + inplay button = participating in team
    OUTOFPLAY     // button toggled off
};

struct ConfigExecution
{
    double frequency = 40.0;                 // [Hz]     The frequency of the heartbeat (tick), e.g., 40Hz -> 40 ticks per second.
    double simulationSpeedupFactor = 1.0;    // [double] The speedup factor for simulation, e.g., 2.0 -> simulation time advances at 200%: 1 real-world second == 2 seconds in simulation time
    std::string tickFinishRtdbKey = "ROBOT_VELOCITY_SETPOINT";   // [string] The RtDB key that is written when a tick / heartbeat finishes. Execution will subscribe to this RtDB key.
};

struct wayPoint
{
    MRA::Geometry::Pose pos;
    MRA::Geometry::Pose vel;
};

struct ObstacleAvoidanceConfig
{
    bool enabled = true;
    double robotRadius = 0.26;
    double obstacleRadius = 0.26;
    double distanceScalingFactor = 0.0;
    double speedScalingFactor = 1.0;
    double speedLowerThreshold = 0.3;
    double speedUpperThreshold = 4.0;
    double generatedObstacleSpacing = 0.5;
    double ballClearance = 0.5;
    double groupGapDistance = 0.5;
    double subTargetDistance = 0.5;
    double subTargetExtensionFactor = 0.0;
};

struct BoundaryConfig
{
    BoundaryOptionEnum targetInsideForbiddenArea;
    BoundaryOptionEnum targetOutsideField;
    double fieldMarginX = 0.0;
    double fieldMarginY = 0.0;
    BoundaryOptionEnum targetOnOwnHalf;
    BoundaryOptionEnum targetOnOpponentHalf;
};

struct ForwardDrivingConfig
{
    bool enabled = true;
    double minimumDistance = 2.0;
};

struct ForwardDrivingConfigs
{
    ForwardDrivingConfig withoutBall;
    ForwardDrivingConfig withBall;
    double radiusRobotToBall = 0.25;
    bool applyLimitsToBall = true;
};

struct DeadzoneConfig
{
    bool enabled;
    double toleranceXY    = 0.01;
    double toleranceRz    = 0.005;
};

struct TokyoDriftConfig
{
    double toleranceRz    = 1.0; // if deltaPos.Rz > tokyoDrift.toleranceRz -> do tokyo drift; otherwise do normal rotation
};

struct ConfigPathPlanning
{
    int                                numExtraSettlingTicks = 0;
    ObstacleAvoidanceConfig            obstacleAvoidance;
    BoundaryConfig                     boundaries;
    double                              slowFactor = 0.5;
    ForwardDrivingConfigs              forwardDriving;
    DeadzoneConfig                     deadzone;
    TokyoDriftConfig                   tokyoDrift;
};

struct robotState
{
    robotStatusEnum    status;
    // rtime              timestamp;
    MRA::Geometry::Pose position;
    MRA::Geometry::Pose velocity;
    bool                hasBall;
    // vec2d              ballAcquired; // only filled in when having ball, for dribble rule
    // int                robotId;
    // teamIdType         teamId;
};


struct vec2d
{
    double      x = 0.0;
    double      y = 0.0;
    
    vec2d(double xx = 0.0, double yy = 0.0)
    {
        x = xx;
        y = yy;
    }
};

struct polygon
{
    std::vector<vec2d>   points;

    // this magic was found in geometry, polygon2D.cpp
    // (original from internet somewhere??)
    bool isPointInside(vec2d const &p)
    {
        bool retVal = false;
        size_t i, j, nvert = points.size();
        if (nvert > 2)
        {
            for (i = 0, j = nvert - 1; i < nvert; j = i++)
            {
                if (((points[i].y >= p.y ) != (points[j].y >= p.y) ) &&
                    (p.x <= (points[j].x - points[i].x) * (p.y - points[i].y) / (points[j].y - points[i].y) + points[i].x))
                {
                    retVal = !retVal;
                }
            }
        }
        return retVal;
    }
};

struct forbiddenArea: public polygon
{
    int id; // extra w.r.t. polygon
};

typedef double rtime;

struct vec3d
{
    double      x;
    double      y;
    double      z;
};

enum class ballPossessionTypeEnum
{
    UNKNOWN,
    FIELD,
    TEAM,
    OPPONENT
};

struct ballPossession
{
    ballPossessionTypeEnum type;
    int                    robotId; // can be friendly or even opponent robot id (roadmap)
};

struct ballResult
{
    vec3d          position;
    vec3d          velocity;
    double          confidence;
    ballPossession owner;
};

struct obstacleResult
{
    vec2d          position;
    vec2d          velocity;
    double         confidence = 0.0;
    int            id = 0; // optional, if we ever want to track/identify opponents
};

enum class motionTypeEnum
{
    INVALID,
    NORMAL,         // Default movement, as fast as possible without taking risks / bumping into obstacles
    WITH_BALL,      // Default movement with ball, minimizing risk to lose the ball
    ACCURATE,       // Sacrificing speed for accuracy, typically used in a setpiece
    INTERCEPT,      // Moving sideways, limit rotation for highest chance of catching ball
    SLOW,           // Slow movement for safety. Used for park. At the end of a match, when robots are parking, people often walk over the field between the robots.
    SPRINT          // Higher speed, higher risk, sacrificing accuracy. May have overshoot and bump into obstacles. Do not use when near obstacles.
};


typedef struct PathPlanningData
{
    // inputs
    ConfigPathPlanning          configPP;
    ConfigExecution             configEx;
    wayPoint                    target;  
    double                      timestamp;   // was type: rtime
    std::vector<forbiddenArea>  forbiddenAreas;
    robotState                  robot;
    std::vector<ballResult>     balls;
    std::vector<robotState>     teamMembers;
    std::vector<obstacleResult> obstacles; // only the ones from worldModel, see calculatedObstacles below

    // calculation results
    std::vector<wayPoint>         path;
    MRA::Datatypes::ActionResult resultStatus;
    std::vector<forbiddenArea>   calculatedForbiddenAreas; // = input + obstacle paths
    std::vector<obstacleResult>  calculatedObstacles;
    MRA::Geometry::Position      targetPositionFcs; // might be corrected with ball possession offset
    MRA::Geometry::Position      currentPositionFcs; // might be corrected with ball possession offset
    MRA::Geometry::Position      deltaPositionFcs; // delta of current position w.r.t. subtarget (=first waypoint)
    MRA::Geometry::Position      deltaPositionRcs;

    // internal data
    double                      dt;
    bool                        done = false;
    motionTypeEnum              motionType = motionTypeEnum::INVALID;
    bool                        stop = false;
    rtime                       previousTimestamp;

    // functions
    void reset();
    void traceInputs();
    void traceOutputs();
    MRA::Geometry::Position getSubTarget() const;
    void insertSubTarget(MRA::Geometry::Position const &pos, MRA::Geometry::Velocity const &vel = MRA::Geometry::Velocity(0,0,0));
    void addForbiddenAreas(std::vector<forbiddenArea> const &newForbiddenAreas);
    void addForbiddenArea(forbiddenArea const &newForbiddenArea);
} PathPlanningData_t;


struct diagPathPlanning
{
    std::vector<wayPoint> path; // can contain a single target, or no target, or even an extra intermediate (sub-)target
    std::vector<forbiddenArea> forbiddenAreas;
    MRA::Geometry::Pose   distanceToSubTargetRCS; // for kstplot_motion
    int                   numCalculatedObstacles;
};

#include <cmath>
inline double project_angle_mpi_pi(double angle)
{
    // sanity checks
    if ((angle > 100) || (angle < -100))
    {
        throw std::runtime_error("angle out of bounds");
    }
    while (angle < -M_PI) angle += 2*M_PI;
    while (angle > M_PI) angle -= 2*M_PI;
    return angle;
}

inline void Rotate(MRA::Geometry::Pose& p, double angle) {
    double s = sin(angle);
    double c = cos(angle);
    double nx = c * p.x - s * p.y;
    double ny = s * p.x + c * p.y;
    p.x = nx;
    p.y = ny;
}

// TODO add pose + point to pose in MRA

inline void Normalize(MRA::Geometry::Pose& p, double factor = 1.0)
{
    p /= hypot(p.x, p.y);
    p *= factor;
}


// // common Falcons headers
// #include "ConfigInterface.hpp"

// // PathPlanning interfaces
// #include "int/OutputInterface.hpp"

struct motionSetpoint
{
    MRA::Datatypes::ActionResult  action;
    MRA::Geometry::Position       position; // could be interpreted as a pose (in case of move) or vec3d (when shooting)
    motionTypeEnum                motionType; // different move types (e.g., normal, accurate (setpiece), intercept)
};


class InputInterface
{
public:
    InputInterface() {};
    virtual ~InputInterface() {};

    virtual void                        fetch() = 0;

    virtual motionSetpoint              getMotionSetpoint() = 0;
    virtual std::vector<forbiddenArea>  getForbiddenAreas() = 0;
    virtual robotState                  getRobotState() = 0;
    virtual std::vector<robotState>     getTeamMembers() = 0;
    virtual std::vector<ballResult>     getBalls() = 0;
    virtual std::vector<obstacleResult> getObstacles() = 0;

};


class OutputInterface
{
public:
    OutputInterface() {};
    virtual ~OutputInterface() {};

    // required
    virtual void setSubtarget(MRA::Datatypes::ActionResult const &status, 
                              bool positionSetpointValid,
                              MRA::Geometry::Position const &robotPositionSetpoint,
                              bool velocitySetpointValid,
                              MRA::Geometry::Velocity const &robotVelocitySetpoint) = 0;

    // optional
    virtual void setDiagnostics(diagPathPlanning const &diagnostics) {};
};


// data struct

// // use a short alias
// using ppCFI = ConfigInterface<ConfigPathPlanning>;
// using exCFI = ConfigInterface<ConfigExecution>;


class PathPlanning
{
public:
    // PathPlanning(ppCFI *configInterfacePP = NULL, exCFI *configInterfaceEx = NULL, InputInterface *inputInterface = NULL, OutputInterface *outputInterface = NULL);
    PathPlanning();
    ~PathPlanning();

    // full iteration:
    // * get RTDB inputs
    // * calculate
    // * set RTDB outputs
    MRA::Datatypes::ActionResult iterate();

    // raw calculation based on inputs, useful for unit testing
    MRA::Datatypes::ActionResult calculate();

    // TODO: add an interface to provide (part of) the configuration
    // example use case: interceptBall calculation needs robot speed capability (maxVelXY)


public:
    // having these public is convenient for test suite
    PathPlanningData data;
    void prepare();
    void setOutputs();

private:
    // helper functions
    void getInputs();
    diagPathPlanning makeDiagnostics();

    // ppCFI                     *_configInterfacePP;
    // exCFI                     *_configInterfaceEx;
    // InputInterface            *_inputInterface;
    // OutputInterface           *_outputInterface;

};

#endif

