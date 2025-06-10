/*
 * MiniSimulation.hpp
 *
 *  Created on: November 2019
 *      Author: Jan Feitsma
 */

#ifndef PATHPLANNING_MINISIMULATION_HPP_
#define PATHPLANNING_MINISIMULATION_HPP_


#include "PathPlanningSimulationScene.hpp" // sharedTypes
#include "cLogger.hpp"
#include "int/PathPlanning.hpp"


#define MINISIMULATION_DEFAULT_ROBOT_ID 2

struct MiniSimulationResult
{
    bool success;
    actionResultTypeEnum status;
    double duration;
    double distance;
    double maxSpeedRobot = 0.0;
    double minDistanceToObstacles = 0.0;
    // to verify limiter behavior when robot has the ball, both in RCS
    vec2d maxVelBall;
    vec2d maxAccBall;
};


class MiniSimulation
{
public:
    MiniSimulation(std::string const &yamlfile, double timeout=10.0, bool writeRDL=false);
    MiniSimulation(ConfigPathPlanning const &config, double timeout=10.0, bool writeRDL=false);
    void setScene(PathPlanningSimulationScene &scene);
    void setScene(std::string const &scenefile);
    void setRdlFilename(std::string const &rdlFilename) { _rdlFilename = rdlFilename; }
    void overruleVelocityController(VelocitySetpointControllerTypeEnum const &vct, CoordinateSystemEnum const &vccs);
    PathPlanningSimulationScene loadScene(std::string const &scenefile); // not yet implemented
    void initialize();
    MiniSimulationResult run();
    PathPlanningData const &getData() const { return _pp.data; }

private:
    PathPlanning                _pp;
    PathPlanningSimulationScene _scene;
    ConfigPathPlanning          _config;
    double                       _timeout = 10.0;
    int                         _myRobotId = MINISIMULATION_DEFAULT_ROBOT_ID;
    std::map<int, RtDB2*>       _rtdb;
    bool                        _initialized = false;
    bool                        _writeRDL = false;
    std::string                 _rdlFilename = "auto";
    OutputInterface            *_rtdbOutputAdapter = NULL;
    MRA::Geometry::Position     _simulatedPositionFcs;
    MRA::Geometry::Velocity     _simulatedVelocityFcs;
    cLogger                    *_logger = NULL;
    MiniSimulationResult        _result;

    // helper functions
    void applySceneToData();
    void processIterationResult();
    double calcDistanceToObstacles();
    void initRDL();
    void tickRDL(rtime const &t);
    void finishRDL();

    // data adapters on PathPlanningData, to put extra things in RDL, for visualization / analysis
    void rtdbPut();
};

#endif
