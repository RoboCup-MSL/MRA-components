/**
 *  @file
 *  @brief   class for robot planner
 *  @curator Jürge van Eijck
 */
#ifndef GlobalPathPlanner_H
#define GlobalPathPlanner_H 1

#include <vector>
#include <list>
#include "Vertex.h"
#include "PlannerOptions.h"
#include "Position.h"
#include "MovingObject.h"
#include "FieldConfig.h"
#include "TeamPlannerData.h"
#include "TeamPlannerResult.h"

/**
 * Represents an undirected graph with nodes the important point in the field.
 * Nodes are connected when there is an free direct line between them (they can
 * "see" each other)
 *
 */

namespace trs {

class GlobalPathPlanner {

private:
	FieldConfig m_fieldConfig;
	MovingObject m_Me;
	MovingObject m_Ball;
	Vertex * m_start;
	Vector2D m_startVelocity;
	std::vector<Vertex *> m_target;
	std::vector<Vertex *> m_vertices;
	std::vector<Vector2D> m_teammates;
	std::vector<Vector2D> m_opponents;
	std::vector<Vertex* > m_approachVertices;
	std::vector<Vertex* > m_addPoints;
	PlannerOptions m_options;
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
	void setOptions(const PlannerOptions& options);

	/* create graph for the provided input */
	void createGraph(MovingObject me, MovingObject ball, const std::vector<MovingObject>& teammates,
			const std::vector<MovingObject>& opponents, const std::vector<trs::Vertex>& targetPos, planner_target_e targetFunction, bool ballIsObstacle,
			bool avoidBallPath, const Vector2D& rBallTargePos);

	/**
	 *
	 * Plans a path in the given graph. This is an implementation of the A*
	 * algorithm with the straight line distance to target as the heuristic
	 * function.
	 *
	 * @return List of coordinates on the shortest path
	 */
	std::vector<planner_piece_t> getShortestPath();

	/* Save the current status of the graph planner to svg file (use name from options) */
	void save_graph_as_svg(const vector<planner_piece_t>& path);
private:
	GlobalPathPlanner(); // prevent creating class via default constructor

	std::vector<Vertex> getVertices();
	void clearApproachVertices();

	bool equalToTarget(const Vertex* v);

	void addOpponents(const std::vector<MovingObject>& opponents, bool skipFirstRadius);
	void addTeammates(const std::vector<MovingObject>& teamMates);

	bool nearPath(const Vector2D& v);

	void addEdges(bool avoidBallPath, const Vector2D& rBallTargePos);

	double ballApproachPenalty(Vertex* v);
	double ownVelocityPenalty(Vertex* v);

	double barrierCosts(Vertex* v1, Vertex* v2);

	void addUniformVertices();

	void addBallApproachVertices();

	void addEnemyGoalApproachVertices();

	void addPoint( const Vector2D& point);

};

} // namespace
#endif
