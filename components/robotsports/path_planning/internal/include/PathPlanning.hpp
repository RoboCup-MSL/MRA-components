/*
 * PathPlanning.hpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#ifndef PATHPLANNING_HPP_
#define PATHPLANNING_HPP_

#include "../../RobotsportsPathPlanning_datatypes.hpp"
#include "vector2d.hpp"

#include <vector>
// map RobotSports Position2D, Velocity2D pose to MRA classes and operations (at MRA-libraries)
// #include "geometry.hpp"
// typedef MRA::Geometry::Position Position2D;
// typedef MRA::Geometry::Velocity Velocity2D;
// typedef MRA::Geometry::Pose Pose2D;
#include "position2d.hpp"

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
    float frequency = 40.0;                 // [Hz]     The frequency of the heartbeat (tick), e.g., 40Hz -> 40 ticks per second.
    float simulationSpeedupFactor = 1.0;    // [double] The speedup factor for simulation, e.g., 2.0 -> simulation time advances at 200%: 1 real-world second == 2 seconds in simulation time
    std::string tickFinishRtdbKey = "ROBOT_VELOCITY_SETPOINT";   // [string] The RtDB key that is written when a tick / heartbeat finishes. Execution will subscribe to this RtDB key.
};

struct pose
{
    float x = 0;
    float y = 0;
    float Rz = 0;
    
    pose(float xx = 0.0, float yy = 0.0, float Rzz = 0.0) : x(xx), y(yy), Rz(Rzz) {}
};

struct wayPoint
{
    pose pos;
    pose vel;
};

struct ObstacleAvoidanceConfig
{
    bool enabled = true;
    float robotRadius = 0.26;
    float obstacleRadius = 0.26;
    float distanceScalingFactor = 0.0;
    float speedScalingFactor = 1.0;
    float speedLowerThreshold = 0.3;
    float speedUpperThreshold = 4.0;
    float generatedObstacleSpacing = 0.5;
    float ballClearance = 0.5;
    float groupGapDistance = 0.5;
    float subTargetDistance = 0.5;
    float subTargetExtensionFactor = 0.0;
};

struct BoundaryConfig
{
    BoundaryOptionEnum targetInsideForbiddenArea;
    BoundaryOptionEnum targetOutsideField;
    float fieldMarginX = 0.0;
    float fieldMarginY = 0.0;
    BoundaryOptionEnum targetOnOwnHalf;
    BoundaryOptionEnum targetOnOpponentHalf;
};

struct ForwardDrivingConfig
{
    bool enabled = true;
    float minimumDistance = 2.0;
};

struct ForwardDrivingConfigs
{
    ForwardDrivingConfig withoutBall;
    ForwardDrivingConfig withBall;
    float radiusRobotToBall = 0.25;
    bool applyLimitsToBall = true;
};

struct DeadzoneConfig
{
    bool enabled;
    float toleranceXY    = 0.01;
    float toleranceRz    = 0.005;
};

struct TokyoDriftConfig
{
    float toleranceRz    = 1.0; // if deltaPos.Rz > tokyoDrift.toleranceRz -> do tokyo drift; otherwise do normal rotation
};

struct ConfigPathPlanning
{
    int                                numExtraSettlingTicks = 0;
    ObstacleAvoidanceConfig            obstacleAvoidance;
    BoundaryConfig                     boundaries;
    float                              slowFactor = 0.5;
    ForwardDrivingConfigs              forwardDriving;
    DeadzoneConfig                     deadzone;
    TokyoDriftConfig                   tokyoDrift;
};

struct robotState
{
    robotStatusEnum    status;
    // rtime              timestamp;
    pose               position;
    pose               velocity;
    bool               hasBall;
    // vec2d              ballAcquired; // only filled in when having ball, for dribble rule
    // int                robotId;
    // teamIdType         teamId;
};


struct vec2d
{
    float      x = 0.0;
    float      y = 0.0;
    
    vec2d(float xx = 0.0, float yy = 0.0)
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
    float      x;
    float      y;
    float      z;
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
    float          confidence;
    ballPossession owner;
};

struct obstacleResult
{
    vec2d          position;
    vec2d          velocity;
    float          confidence = 0.0;
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
    std::vector<wayPoint>     path;
    MRA::Datatypes::ActionResult  resultStatus;
    std::vector<forbiddenArea>  calculatedForbiddenAreas; // = input + obstacle paths
    std::vector<obstacleResult> calculatedObstacles;
    Position2D                  targetPositionFcs; // might be corrected with ball possession offset
    Position2D                  currentPositionFcs; // might be corrected with ball possession offset
    Position2D                  deltaPositionFcs; // delta of current position w.r.t. subtarget (=first waypoint)
    Position2D                  deltaPositionRcs;

    // internal data
    float                       dt;
    bool                        done = false;
    motionTypeEnum              motionType = motionTypeEnum::INVALID;
    bool                        stop = false;
    rtime                       previousTimestamp;

    // functions
    void reset();
    void traceInputs();
    void traceOutputs();
    Position2D getSubTarget() const;
    void insertSubTarget(Position2D const &pos, Velocity2D const &vel = Velocity2D(0,0,0));
    void addForbiddenAreas(std::vector<forbiddenArea> const &newForbiddenAreas);
    void addForbiddenArea(forbiddenArea const &newForbiddenArea);
} PathPlanningData_t;


struct diagPathPlanning
{
    std::vector<wayPoint> path; // can contain a single target, or no target, or even an extra intermediate (sub-)target
    std::vector<forbiddenArea> forbiddenAreas;
    pose                  distanceToSubTargetRCS; // for kstplot_motion
    int                   numCalculatedObstacles;
};


// // common Falcons headers
// #include "ConfigInterface.hpp"

// // PathPlanning interfaces
// #include "int/InputInterface.hpp"
// #include "int/OutputInterface.hpp"

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

