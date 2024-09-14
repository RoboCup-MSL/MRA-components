/**
 *  @file
 *  @brief   class for robot planner
 *  @curator Jürge van Eijck
 */
#ifndef GlobalPathPlanner_HPP
#define GlobalPathPlanner_HPP 1

#include "FieldConfig.hpp"
#include "TeamPlannerData.hpp"
#include "TeamPlannerParameters.hpp"
#include "TeamPlannerResult.hpp"
#include "TeamPlannerOpponent.hpp"

#include "Vertex.hpp"
#include "planner_types.hpp"

#include <vector>
#include <list>

/**
 * Represents an undirected graph with nodes the important point in the field.
 * Nodes are connected when there is an free direct line between them (they can
 * "see" each other)
 *
 */

namespace MRA {

class GlobalPathPlanner {

private:
    FieldConfig m_fieldConfig;
    Vertex * m_start;
    MRA::Geometry::Position m_startVelocity;
    std::vector<Vertex *> m_target;
    std::vector<Vertex *> m_vertices;
    std::vector<MRA::Geometry::Position> m_teammates;
    std::vector<MRA::Geometry::Position> m_opponents;
    std::vector<Vertex* > m_approachVertices;
    std::vector<Vertex* > m_addPoints;
    TeamPlannerParameters m_options;
    planner_target_e m_targetFunction;
    double m_maxFieldX;
    double m_maxFieldY;

public:

    /**
     * Constructs a visibility graph
     *
     * @param start
     *            Starting position
     * @param target
     *            Target position
     * @param objects
     *            Position of objects in the field to avoid. All objects are
     *            assumed to be 60x60 cm.
     */
    explicit GlobalPathPlanner(FieldConfig fieldConfig);
    virtual ~GlobalPathPlanner();

    /* set the options for the planner */
    void setOptions(const TeamPlannerParameters& options);

    /* create graph for the provided input */
    void createGraph(const MRA::Geometry::Position& start_pose, const MRA::Geometry::Position& start_vel,
                     const TeamPlannerBall& ball,
                     const std::vector<TeamPlannerRobot>& teammates, /* filtered based on robot to calculate the graph for */
                     const std::vector<TeamPlannerOpponent>& opponents,
                     const std::vector<MRA::Vertex>& targetPos,
                     planner_target_e targetFunction,
                     bool ballIsObstacleAndValid,
                     bool avoidBallPath,
                     bool stayInPlayingField,
                     const MRA::Geometry::Point& rBallTargetPos);

    /**
     *
     * Plans a path in the given graph. This is an implementation of the A*
     * algorithm with the straight line distance to target as the heuristic
     * function.
     *
     * @return List of coordinates on the shortest path
     */
    std::vector<planner_piece_t> getShortestPath(const TeamPlannerData& teamplanner_data);

    /* Save the current status of the graph planner to svg file (use name from options) */
    void save_graph_as_svg(const TeamPlannerData& teamplanner_data, const std::vector<planner_piece_t>& path);
private:
    GlobalPathPlanner(); // prevent creating class via default constructor

    std::vector<Vertex> getVertices();
    void clearApproachVertices();

    bool equalToTarget(const Vertex* v);

    void addObstacle(const MRA::Geometry::Position& opponent, bool skipFirstRadius, bool stayInPlayingField); // TODO rename to moving obstacle

    bool nearPath(const MRA::Geometry::Position& v);

    void addEdges(bool avoidBallPath, const MRA::Geometry::Point& rBallTargetPos, const TeamPlannerBall& ball);

    double ballApproachPenalty(Vertex* v);
    double ownVelocityPenalty(Vertex* v);

    double barrierCosts(Vertex* v1, Vertex* v2);

    void addUniformVertices(bool stayInPlayingField);

    void addBallApproachVertices();

    void addEnemyGoalApproachVertices();

    void addPoint( const MRA::Geometry::Position& point, bool stayInPlayingField);

};

} // namespace
#endif // GlobalPathPlanner_HPP
