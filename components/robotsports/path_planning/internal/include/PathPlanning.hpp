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

struct wayPoint
{
    MRA::Geometry::Pose pos;
    MRA::Geometry::Pose vel;
};

typedef struct ObstacleAvoidanceParameters_s
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
} ObstacleAvoidanceParameters_t;

typedef struct BoundaryParameters_s
{
    BoundaryOptionEnum targetInsideForbiddenArea;
    BoundaryOptionEnum targetOutsideField;
    double fieldMarginX = 0.0;
    double fieldMarginY = 0.0;
    BoundaryOptionEnum targetOnOwnHalf;
    BoundaryOptionEnum targetOnOpponentHalf;
} BoundaryParameters_t;

typedef struct ForwardDrivingSituationParameters_s
{
    bool enabled = true;
    double minimumDistance = 2.0;
} ForwardDrivingSituationParameters_t;

typedef struct ForwardDrivingParameters_s
{
    ForwardDrivingSituationParameters_t withoutBall;
    ForwardDrivingSituationParameters_t withBall;
    double radiusRobotToBall = 0.25;
    bool applyLimitsToBall = true;
} ForwardDrivingParameters_t; 

typedef struct DeadzoneParameters_s
{
    bool enabled;
    double toleranceXY    = 0.01;
    double toleranceRz    = 0.005;
} DeadzoneParameters_t;

typedef struct TokyoDriftParameters_s
{
    double toleranceRz    = 1.0; // if deltaPos.Rz > tokyoDrift.toleranceRz -> do tokyo drift; otherwise do normal rotation
} TokyoDriftParameters_t;

typedef struct path_planner_parameters_s {
    int                           numExtraSettlingTicks = 0;
    ObstacleAvoidanceParameters_t obstacleAvoidance;
    BoundaryParameters_t          boundaries;
    double                        slowFactor = 0.5;
    ForwardDrivingParameters_t    forwardDriving;
    DeadzoneParameters_t          deadzone;
    TokyoDriftParameters_t        tokyoDrift;
    double                        frequency = 40.0; // [Hz]     The frequency of the heartbeat (tick), e.g., 40Hz -> 40 ticks per second.
} path_planner_parameters_t;

typedef struct robotState_s
{
    bool                active = false;
    // rtime              timestamp;
    MRA::Geometry::Pose position = {};
    MRA::Geometry::Pose velocity = {};
    bool                hasBall = false;
    // vec2d              ballAcquired; // only filled in when having ball, for dribble rule
    // int                robotId;
    // teamIdType         teamId;
} robotState_t;



class polygon
{
    public:
        std::vector<MRA::Geometry::Point>   points;

        // this magic was found in geometry, polygon2D.cpp
        // (original from internet somewhere??)
        bool isPointInside(MRA::Geometry::Point const &p)
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
    bool                    valid;
    MRA::Geometry::Position position;
    MRA::Geometry::Velocity velocity;
    double                  confidence;
    ballPossession          owner;
};

struct obstacleResult
{
    MRA::Geometry::Point position;
    MRA::Geometry::Point velocity;
    double               confidence = 0.0;
    int                  id = 0; // optional, if we ever want to track/identify opponents
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
    path_planner_parameters_t   parameters;
    wayPoint                    target;  
    double                      timestamp;   // was type: rtime
    std::vector<forbiddenArea>  forbiddenAreas;
    robotState_t                robot;
    ballResult                  ball;
    std::vector<robotState_t>     teamMembers;
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


typedef struct path_planner_diagnostics_s
{
    std::vector<wayPoint> path; // can contain a single target, or no target, or even an extra intermediate (sub-)target
    std::vector<forbiddenArea> forbiddenAreas;
    MRA::Geometry::Pose   distanceToSubTargetRCS; // for kstplot_motion
    int                   numCalculatedObstacles;
    bool                  stop; 
} path_planner_diagnostics_t;

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

typedef struct motionSetpoint_s
{
    MRA::Datatypes::ActionResult  action;
    MRA::Geometry::Position       position; // could be interpreted as a position (in case of move) or pose (when shooting)
    motionTypeEnum                motionType; // different move types (e.g., normal, accurate (setpiece), intercept)
} motionSetpoint_t;


typedef struct path_planner_input_s
{
    motionSetpoint_t            motionSetpoint;
    robotState_t                myRobotState;
    std::vector<robotState_t>   teamRobotState;
    ballResult                  ball;
    std::vector<obstacleResult> obbstacles;

    std::vector<forbiddenArea>  forbiddenAreas; // parameters ?
} path_planner_input_t;


typedef struct path_planner_output_s
{
    MRA::Datatypes::ActionResult status;
    bool positionSetpointValid;
    MRA::Geometry::Position robotPositionSetpoint;
    bool velocitySetpointValid;
    MRA::Geometry::Velocity robotVelocitySetpoint;
    motionTypeEnum motionType;
} path_planner_output_t;



typedef struct path_planner_state_s {

} path_planner_state_t;


class PathPlanning
{
public:
    PathPlanning();
    ~PathPlanning();


    // raw calculation based on inputs
    void calculate(double ts, 
                    const path_planner_input_t& r_input, 
                    const path_planner_parameters_t& r_params,
                    path_planner_state_t& r_state,
                    path_planner_output_t& r_output,
                    path_planner_diagnostics_t& r_diagnostics);

    // TODO: add an interface to provide (part of) the configuration
    // example use case: interceptBall calculation needs robot speed capability (maxVelXY)

    // having these public is convenient for test suite
    PathPlanningData data;
private:

};

#endif

