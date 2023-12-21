/**
 *  @file
 *  @brief   class for robot planner
 *  @curator Jürge van Eijck
 */
#include "GlobalPathPlanner.hpp"

#include <vector>
#include <cmath>
#include <string>
#include <list>
#include <iostream>
#include <ostream>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <limits>

#include "FieldConfig.h"
#include "MathUtils.h"
#include "SvgUtils.hpp"
#include "Vertex.hpp"

using namespace std;

namespace trs {

/**
 * Integrates 1 over the square of the distance between point c and line segment p1-p2.
 *
 * @param c
 *            Point
 * @param p1
 *            Begin point of line segment
 * @param p2
 *            End point of line segment
 * @return 1 over square of the distance between c and p1-p2, integrated over p1-p2.
 *           infinity is returned if distanceIntegral can not be calculated.
 */
double distanceIntegral(const MRA::Geometry::Point& c, const MRA::Geometry::Point& p1, const MRA::Geometry::Point& p2) {
    // Calculate square of the length of p1-p2.
    double l2 = (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y);
    if (fabs(l2) < 1e-16) {
        // prevent division by zero
        return std::numeric_limits<double>::infinity();
    }
    double lambda = ((p2.x - p1.x) * (c.x - p1.x) + (p2.y - p1.y) * (c.y - p1.y)) / l2;
    double len = sqrt(l2);
    double s1 = -lambda * len;
    double s2 = (1 - lambda) * len;
    // Find the point (xx,yy) where the perpendicular line through c crosses line p1-p2
    double xx = p1.x + lambda * (p2.x - p1.x);
    double yy = p1.y + lambda * (p2.y - p1.y);
    // Calculate distance of c to line p1-p2
    double d = sqrt((c.x - xx) * (c.x - xx) + (c.y - yy) * (c.y - yy));
    if (fabs(d) < 1e-16) {
        // prevent division by zero
        return std::numeric_limits<double>::infinity();
    }
    // Formula found using WolframAlpha.com. Type: integrate 1/(x^2+d^2) dx
    return (atan2(s2, d) - atan2(s1, d)) / d;
    // return (aTan2(s2,d)-aTan2(s1,d))/d;
    // return (Math.atan2(s2, d)-Math.atan2(s1, d))/d;
}


static int svgId = -1;

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
GlobalPathPlanner::GlobalPathPlanner(FieldConfig fieldConfig)  :
	m_fieldConfig(fieldConfig),
	m_Me(MovingObject()),
	m_Ball(MovingObject()),
	m_start(0),
	m_startVelocity(0),
	m_target(std::vector<Vertex *>()),
	m_vertices(std::vector<Vertex *>()),
	m_teammates(std::vector<MRA::Geometry::Point>()),
	m_opponents(std::vector<MRA::Geometry::Point>()),
	m_approachVertices(std::vector<Vertex*>()),
	m_addPoints(std::vector<Vertex*>()),
	m_options(TeamPlannerParameters()),
	m_targetFunction(planner_target_e::GOTO_BALL),
	m_maxFieldX(0),
	m_maxFieldY(0)
{
	m_Me.getVelocity(m_startVelocity);
}

GlobalPathPlanner::~GlobalPathPlanner() {
	delete m_start;
	clearApproachVertices();
	for(std::vector<Vertex *>::size_type idx = 0; idx != m_target.size(); idx++) {
		delete m_target[idx];
	}
	m_target.clear();
	for(std::vector<Vertex *>::size_type idx = 0; idx != m_addPoints.size(); idx++) {
		delete m_addPoints[idx];
	}
	m_addPoints.clear();


}

/**
 *  clear (clean-up) all approach vertices
 */
void GlobalPathPlanner::clearApproachVertices() {
	for(std::vector<Vertex *>::size_type idx = 0; idx != m_approachVertices.size(); idx++) {
		delete m_approachVertices[idx];
	}
	m_approachVertices.clear();
}

void GlobalPathPlanner::setOptions(const TeamPlannerParameters& options) {
	m_options = options;
}

void GlobalPathPlanner::createGraph(const MovingObject& start, const TeamPlannerData& teamplanner_data,
		const std::vector<trs::Vertex>& targetPos,
		planner_target_e targetFunction,
		bool ballIsObstacle,
		bool avoidBallPath,
		const MRA::Geometry::Point& rBallTargePos) {

    auto ball = teamplanner_data.ball;

	// TODO check on dist me to target, if large than max field. stop calculations !!!!!
	m_Ball = ball;
	Position startPos = start.getPosition();
	if (m_start != 0) {
		delete m_start;
	}
	m_start = new Vertex(startPos.getPoint(), 0);
	start.getVelocity(m_startVelocity);
	m_vertices.push_back(m_start);
	m_targetFunction = targetFunction;
	for(std::vector<Vertex>::size_type pos_idx = 0; pos_idx != targetPos.size(); pos_idx++) {
		Vertex* pTarget = new Vertex(targetPos[pos_idx]);
		m_target.push_back(pTarget);
		m_vertices.push_back(pTarget);
	}

	m_maxFieldX = m_fieldConfig.getMaxFieldX();
	m_maxFieldY = m_fieldConfig.getMaxFieldY();
	//
	// Create nodes and opponents (barriers) node
	//

	//

	if (m_options.addBallApproachVertices && m_targetFunction == planner_target_e::GOTO_BALL) {
			addBallApproachVertices();
	}

	if (m_targetFunction == planner_target_e::DRIBBLE) {
		addEnemyGoalApproachVertices();
	}


	if (m_options.addBarierVertices) {
		if (ballIsObstacle) {
			// handle ball as obstacle
			if (teamplanner_data.ball_present) {
				addOpponent(teamplanner_data.ball, true);
			}
		}
		for (auto opponent: teamplanner_data.opponents) {
		    addOpponent(opponent.position, false);
		}
        for (auto teammate: teamplanner_data.team) {
            addOpponent(teammate.position, false); // handle team mates as opponent (drive around them)
        }
	}

	//
	// Add extra vertices
	//
	if (m_options.addUniformVertices) {
		addUniformVertices();
	}

	// Add edges
	addEdges(avoidBallPath, rBallTargePos);
}

/**
 * Plans a path in the given graph. This is an implementation of the A*
 * algorithm with the straight line distance to target as the heuristic
 * function.
 *
 * @return List of coordinates on the shortest path
 */
vector<planner_piece_t> GlobalPathPlanner::getShortestPath(const TeamPlannerData& teamplanner_data) {
	m_start->m_minDistance = 0.0;

	std::list<trs::Vertex*> sortedList = std::list<trs::Vertex*>();
	sortedList.push_back(m_start);
	// check if start is not on target location
	for(std::vector<Vertex*>::size_type idx = 0; idx != m_target.size(); idx++) {
		if (m_start->equals(*(m_target[idx]))) {
			vector < planner_piece_t> p = vector<planner_piece_t>();
			planner_piece_t piece;
			piece.x = m_target[idx]->m_coordinate.x;
			piece.y = m_target[idx]->m_coordinate.y;
			piece.cost = 0; // no costs
			piece.target = static_cast<long>(m_targetFunction);
			p.push_back(piece);
			return p; // shortest path found
		}
	}

	// check if start is less than minimum threshold from any target
	bool nearTarget = false;
	double nearTargetThreshold = 0.01; // 1 [mm]: if start is near any target: cost is only distance to target
	double nearTargetCost = std::numeric_limits<double>::infinity();
	MRA::Geometry::Point nearestTarget;
	for(std::vector<Vertex*>::size_type idx = 0; idx != m_target.size(); idx++) {
		double distStartToTarget = m_start->m_coordinate.distanceTo(m_target[idx]->m_coordinate);
		if  (distStartToTarget <  nearTargetThreshold) {
			nearTarget = true;
			if (distStartToTarget < nearTargetCost) {
				nearestTarget = m_target[idx]->m_coordinate;
			}
		}
	}
	if (nearTarget) {
		// start is near a target position.
		vector < planner_piece_t> p = vector<planner_piece_t>();
		planner_piece_t piece;
		piece.x = nearestTarget.x;
		piece.y = nearestTarget.y;
		piece.cost = 0; // no costs
		piece.target = static_cast<long>(m_targetFunction);
		p.push_back(piece);
		return p; // shortest path found
	}


	std::vector<Vertex *> unreached_target = std::vector<Vertex *>();
	for(std::vector<Vertex*>::size_type idx = 0; idx != m_target.size(); idx++) {
		unreached_target.push_back(m_target[idx]);
	}

	while (!sortedList.empty()) {
		Vertex* u = sortedList.front();
		sortedList.pop_front();

		if (!unreached_target.empty()) {
			for(std::vector<Vertex>::size_type idx = unreached_target.size()-1; idx > 0 ; idx--) {
				if (u != 0 && unreached_target[idx]->equals(*u)) {
					if (unreached_target.size() > idx) {  // check if vector has enough element to erase element: indexing starts with 0.
						unreached_target.erase(unreached_target.begin() + idx);
					}
				}
			}
		}
		for (std::vector<Edge>::iterator it = u->m_neighbours.begin(); it != u->m_neighbours.end(); ++it) {
			Edge e = *it;
			Vertex* v = e.m_pTarget;
			double d = e.m_dCost;
			double dd = u->m_minDistance + d;
			if (dd < v->m_minDistance) {
				sortedList.remove(v);
				v->m_minDistance = dd;
				v->m_pPrevious = u;
				sortedList.push_back(v);
				sortedList.sort(VertexCompareFunction);
			}
		}
	}

	bool bestTargetFound = false;
	std::vector<Vertex>::size_type bestTarget_idx = 0;
	double bestTargetMinDist = std::numeric_limits<double>::infinity();
	for(std::vector<Vertex>::size_type idx = 0; idx != m_target.size(); idx++) {
		if (m_target[idx]->totalCosts() <= bestTargetMinDist) {
			bestTargetMinDist = m_target[idx]->totalCosts();
			bestTarget_idx = idx;
			bestTargetFound = true;
		}
	}
	// only 1 target has a path.
	std::vector<Vertex>::size_type idx  = bestTarget_idx;
	vector < planner_piece_t> path2 = vector<planner_piece_t>();

	if (bestTargetFound) {
		int piece_nr = 0;
		for (Vertex* vertex = m_target[idx]; vertex != 0 && piece_nr < 20; vertex = vertex->m_pPrevious) {
			piece_nr++;
			planner_piece_t piece;
			piece.x = vertex->m_coordinate.x;
			piece.y = vertex->m_coordinate.y;
			piece.cost = vertex->m_minDistance;
			piece.target = static_cast<long>(m_targetFunction);

			path2.push_back(piece);
		}
	}
	std::reverse(path2.begin(), path2.end());

	if (m_options.svgRobotPlanner) {
		// Save svgRobotData
		svgId++;
		string org_svgOutputFileName = m_options.svgOutputFileName;
		char buffer[250];
		sprintf(buffer, "_robotplanner_%02d.svg", svgId);
		string filename = m_options.svgOutputFileName;
		if (filename.size() > 4) {
			filename.replace(filename.end()-4,filename.end(), buffer);
		}
		m_options.svgOutputFileName = filename;
		save_graph_as_svg(teamplanner_data, path2);
		m_options.svgOutputFileName = org_svgOutputFileName; // restore svg name
	}
	return path2;
}

void GlobalPathPlanner::addOpponent(const MovingObject& opponent, bool skipFirstRadius) {
    MRA::Geometry::Point v(opponent.getPosition().getPoint());
    double x = v.x;
    double y = v.y;

    // Opponents that are far from any reasonable path from start to
    // target are not added.
    if (nearPath(v)) {
        if (!skipFirstRadius) {
            for (int i = 0; i < m_options.nrVerticesFirstCircle; i++) {
                double angle = i * 2.0 * M_PI
                        / m_options.nrVerticesFirstCircle;
                double xx = x + m_options.firstCircleRadius * cos(angle);
                double yy = y + m_options.firstCircleRadius * sin(angle);
                addPoint(MRA::Geometry::Point(xx, yy));
            }
        }

        for (int i = 0; i < m_options.nrVerticesSecondCircle; i++) {
            double angle = i * 2.0 * M_PI
                    / m_options.nrVerticesSecondCircle;
            double xx = x + m_options.secondCircleRadius * cos(angle);
            double yy = y + m_options.secondCircleRadius * sin(angle);
            addPoint(MRA::Geometry::Point(xx, yy));
        }
    }
    // TODO: prevent a path from going through the goal, or have
    // another process guard such a route?
    m_opponents.push_back(v);
}

void GlobalPathPlanner::addTeammate(const MovingObject& teamMate) {
    MRA::Geometry::Point v(teamMate.getPosition().getPoint());
    double x = v.x;
    double y = v.y;

    // Opponents that are far from any reasonable path from start to
    // target are not added.
    if (nearPath(v)) {
        for (int i = 0; i < m_options.nrVerticesFirstCircle; i++) {
            double angle = i * 2.0 * M_PI
                    / m_options.nrVerticesFirstCircle;
            double xx = x + m_options.firstCircleRadius * cos(angle);
            double yy = y + m_options.firstCircleRadius * sin(angle);
            addPoint(MRA::Geometry::Point(xx, yy));
        }

        for (int i = 0; i < m_options.nrVerticesSecondCircle; i++) {
            double angle = i * 2.0 * M_PI
                    / m_options.nrVerticesSecondCircle;
            double xx = x + m_options.secondCircleRadius * cos(angle);
            double yy = y + m_options.secondCircleRadius * sin(angle);
            addPoint(MRA::Geometry::Point(xx, yy));
        }
    }
    // TODO: prevent a path from going through the goal, or have
    // another process guard such a route?
    m_teammates.push_back(v);
}

/**
 * Calculates whether a point is near a reasonable path from start to
 * target.
 *
 * @param v
 *            Point to check
 * @return true iff v is near the path from start S to any target T. v is near
 *         if it is either within minimumDistanceToEndPoint from either S or
 *         T, or, if the angle <S,v,T> is greater than 90 degrees.
 */
bool GlobalPathPlanner::nearPath(const MRA::Geometry::Point& v) {
	bool res = false;

	if (!res)
		for (std::vector<Vertex *>::iterator it = m_target.begin();
				it != m_target.end() && !res; ++it) {
			res = (((*it)->m_coordinate.x - v.x) * (m_start->m_coordinate.x - v.x)
					+ ((*it)->m_coordinate.y - v.y) * (m_start->m_coordinate.y - v.y) <= 0
					|| v.distanceTo((*it)->m_coordinate) < m_options.minimumDistanceToEndPoint ||
					v.distanceTo(m_start->m_coordinate) < m_options.minimumDistanceToEndPoint);
		}
	return res;
}

/**
 * Inserts edges in the graph between vertices. To optimize, edges that are
 * very long are not included.
 */
void GlobalPathPlanner::addEdges(bool avoidBallPath, const MRA::Geometry::Point& rBallTargePos) {
	for (std::vector<Vertex>::size_type i = 0; i < m_vertices.size() - 1; i++) {
		for (std::vector<Vertex>::size_type j = i + 1; j < m_vertices.size(); j++) {
			Vertex* v1 = m_vertices[i];
			Vertex* v2 = m_vertices[j];
			if (v1 == nullptr || v2 == nullptr) {
//				logAlways("a vertex is 0. (in %s on line: %d)", __FILE__, __LINE__);
				exit(1);
			}
			double distance = v1->m_coordinate.distanceTo(v2->m_coordinate);

			// no edge if both are a target
			if (equalToTarget(v1) && equalToTarget(v2)) {
				continue;
			}

			if (distance < m_options.maximumEdgeLength) {
				if (avoidBallPath) {
					// not allowed to make edge which crosses the pass line.
					double px =0;
					double py = 0;
					bool intersect = lineSegmentIntersection(v1->m_coordinate.x, v1->m_coordinate.y,
							                                 v2->m_coordinate.x, v2->m_coordinate.y,
															 m_Ball.getXYlocation().x, m_Ball.getXYlocation().y,
															 rBallTargePos.x, rBallTargePos.y,
															 px, py);
					if (intersect) {
						// line v1->v2 intersects the passing line (ball - ball targetpos)
						continue;
					}
				}

				double safetyCost = barrierCosts(v1, v2);
				double rugbyCost = m_options.safetyFactor * safetyCost;
				double totalCost = distance + rugbyCost;
				if (v1->equals(*m_start)) {
					totalCost += ownVelocityPenalty(v2);
				} else if (v2->equals(*m_start)) {
					totalCost += ownVelocityPenalty(v1);
				}

				if (((!equalToTarget(v1)) && (!equalToTarget(v2))) || (m_approachVertices.empty())){
					// v1 or v2 is not target OR no approach vertices present
					v1->m_neighbours.push_back(Edge(v2, totalCost));
					v2->m_neighbours.push_back(Edge(v1, totalCost));
				}
				else {
					bool approachVertices_has_v1 = false;
					bool approachVertices_has_v2 = false;
					// one vertice is the target
					for (std::vector<Vertex*>::iterator it = m_approachVertices.begin(); it != m_approachVertices.end(); ++it) {
						if ((*it)->equals(*v2)) {
							approachVertices_has_v2 = true;
						}
						if ((*it)->equals(*v1)) {
							approachVertices_has_v1 = true;
						}
						if (approachVertices_has_v1 && approachVertices_has_v2) {
							break;
						}
					}

					if ((equalToTarget(v1) && approachVertices_has_v2) ||
							(equalToTarget(v2) && approachVertices_has_v1)) {
						if (equalToTarget(v1)) {
							if (m_targetFunction == planner_target_e::GOTO_BALL) {
								totalCost = distance + ballApproachPenalty(v2);
							}
							else {
								/* goto goal : TODO add correct penalty */
								totalCost = distance;
							}
						} else if (equalToTarget(v2)) {
							if (m_targetFunction == planner_target_e::GOTO_BALL) {
								totalCost = distance + ballApproachPenalty(v1);
							}
							else {
								/* goto goal : TODO add correct penalty */
								totalCost = distance;
							}
						}
						v1->m_neighbours.push_back(Edge(v2, totalCost));
						v2->m_neighbours.push_back(Edge(v1, totalCost));
					}
					//ELSE no edge added, only edges from approach to target, no other edges to target allowed
				}
			}
		}
	}
}

/* is give vertex any target vertex ? */
bool GlobalPathPlanner::equalToTarget(const Vertex* pV) {
	bool res = false;
	for(std::vector<Vertex>::size_type idx = 0; idx != m_target.size(); idx++) {
		res = pV->equals(*(m_target[idx]));
		if (res == true) {
			break;
		}
	}
	return res;
}
/**
 * calculate approach ball cost, close to border, less costs. (approach ball from outside)
 * @param v
 * @return
 */
double GlobalPathPlanner::ballApproachPenalty(Vertex* v) {
	double x = v->m_coordinate.x;
	double y = v->m_coordinate.y;

	/* calculate distance to middle of penalty area */
	double penaltyAreaY = m_fieldConfig.getMaxFieldY() - m_fieldConfig.getPenaltyAreaLength();

	double distToMiddlePenaltyArea = hypot(x, (penaltyAreaY-y));
	const double maxLength = hypot(m_fieldConfig.getFullFieldLength(), m_fieldConfig.getFullFieldWidth());
	double approachPenalty = (maxLength-distToMiddlePenaltyArea)*3.0;
	return approachPenalty;
}

double GlobalPathPlanner::ownVelocityPenalty(Vertex* v) {
	MRA::Geometry::Point startCoordinate(m_start->m_coordinate);
	MRA::Geometry::Point initialDirection(v->m_coordinate);
	initialDirection -= startCoordinate;
	MRA::Geometry::Point normalizedInitialDirection(initialDirection);
	normalizedInitialDirection.normalize();
	normalizedInitialDirection -= m_startVelocity;
	double norm = normalizedInitialDirection.size();
	double penalty = norm*norm*m_options.startingVelocityPenaltyFactor;
	return penalty;
}

/**
 * Calculates how much influence a barrier has on a line segment. The closer
 * the barrier, the more costs it adds to traversing the line segment.
 *
 * @param v1
 *            First point of line segment
 * @param v2
 *            Second point of line segment
 * @param distance
 *            Distance between v1 and v2 (necessary for performance reasons:
 *            this distance has been calculated before).
 * @return The cost inflicted by enemies.
 */
double GlobalPathPlanner::barrierCosts(Vertex* v1, Vertex* v2) {
	double safetyCost = 0.0;
	for(std::vector<MRA::Geometry::Point>::size_type bar_idx = 0; bar_idx != m_opponents.size(); bar_idx++) {
		MRA::Geometry::Point barrier = m_opponents[bar_idx];

		double cost = distanceIntegral(barrier, v1->m_coordinate, v2->m_coordinate);
		safetyCost += cost;
	}

	// add barrier for each team mate in own penalty area (keeper and an possible defender).
	// This to prevent that the player will hit them. Only checking on role "goalie" is not possible
	// due to dynamic goalie assignment when the normal goalie is absent.
	for(std::vector<MRA::Geometry::Point>::size_type teammate_idx = 0; teammate_idx != m_teammates.size(); teammate_idx++) {
		MRA::Geometry::Point teammate_position = m_teammates[teammate_idx];
		if (m_fieldConfig.isInOwnPenaltyArea(teammate_position.x, teammate_position.y)) {
			double cost = distanceIntegral(teammate_position, v1->m_coordinate, v2->m_coordinate);
			safetyCost += cost;
		}
	}
	return safetyCost;
}

/**
 * Adds vertices to the graph, that is, possible points along which a path
 * can go. For performance reasons, only vertices that are near a possible
 * path between start and target are added.
 */
void GlobalPathPlanner::addUniformVertices() {
	double border = 0.25;
	if (m_targetFunction == planner_target_e::DRIBBLE) {
		border = -m_fieldConfig.getBallRadius(); // keep balls in the field
	}

	for (double x = -(m_maxFieldX+border); x <= (m_maxFieldX+border); x += m_options.uniform_x_interval) {
		for (double y = -(m_maxFieldY+border); y <= (m_maxFieldY+border); y += m_options.uniform_y_interval) {
			if (!m_fieldConfig.isInOwnGoalArea(x,y) && !m_fieldConfig.isInOpponentGoalArea(x,y)) {
				MRA::Geometry::Point vv = MRA::Geometry::Point(x, y);
				if (nearPath(vv)) {
					addPoint(vv);
				}
			}
		}
	}
}


/**
 * Adds vertices to the graph when the target (ball) is close to the end of the field.
 */
void GlobalPathPlanner::addBallApproachVertices() {
	clearApproachVertices();

	for (std::vector<Vertex*>::iterator it = m_target.begin();
			it != m_target.end(); ++it) {

		double x = (*it)->m_coordinate.x;
		double y = (*it)->m_coordinate.y;
		double distToTarget = (*it)->m_coordinate.distanceTo(m_start->m_coordinate);
		double approachXLimit = fabs(m_maxFieldX - m_options.distToapplyBallApproachVertices);
		double approachYLimit = fabs(m_maxFieldY - m_options.distToapplyBallApproachVertices);

		if ((distToTarget > m_options.ballApproachVerticesRadius)
				&& ((fabs(x) > approachXLimit) || (fabs(y) > approachYLimit))) {
			double angleOffset = (2 * M_PI) / m_options.ballApproachNumberOfVertices;
			for (int i = 0; i < m_options.ballApproachNumberOfVertices; i++) {
				double x1 = x + m_options.ballApproachVerticesRadius * cos(i * angleOffset);
				double y1 = y + m_options.ballApproachVerticesRadius * sin(i * angleOffset);
				if (m_fieldConfig.isInReachableField(x1, y1)) {
					MRA::Geometry::Point point = MRA::Geometry::Point(x1, y1);
					Vertex* vertex = new Vertex(point, point.distanceTo((*it)->m_coordinate));
					m_vertices.push_back(vertex);
					m_approachVertices.push_back(vertex);
				}
			}
		}
	}
}

/**
 * add approach vertices for the goal, this allow multiple path to the goal
 */
void GlobalPathPlanner::addEnemyGoalApproachVertices() {
	clearApproachVertices();

	for (std::vector<Vertex*>::iterator it = m_target.begin();
			it != m_target.end(); ++it) {

		double x = (*it)->m_coordinate.x;
		double y = (*it)->m_coordinate.y;
		double distToTarget = (*it)->m_coordinate.distanceTo(m_start->m_coordinate);
		double approachXLimit = fabs(m_maxFieldX - m_options.distToapplyBallApproachVertices);
		double approachYLimit = fabs(m_maxFieldY - m_options.distToapplyBallApproachVertices);

		if ((distToTarget > m_options.ballApproachVerticesRadius)
				&& ((fabs(x) > approachXLimit) || (fabs(y) > approachYLimit))) {
			double angleOffset = (2 * M_PI) / m_options.ballApproachNumberOfVertices;
			for (int i = 0; i < m_options.ballApproachNumberOfVertices; i++) {
				double x1 = x + m_options.ballApproachVerticesRadius * cos(i * angleOffset);
				double y1 = y + m_options.ballApproachVerticesRadius * sin(i * angleOffset);
				if (m_fieldConfig.isInReachableField(x1, y1)) {
					MRA::Geometry::Point point = MRA::Geometry::Point(x1, y1);

					//logger.info("Add point "+ i + " pos: " + point);
					Vertex* vertex = new Vertex(point, point.distanceTo((*it)->m_coordinate));
					m_vertices.push_back(vertex);
					m_approachVertices.push_back(vertex);
				}
			}
		}
	}
}

/**
 * Adds a vertex to this graph, given by its coordinates but only when
 * appropriate: a vertex that is very close to another vertex is not
 * added as that would not change the best very much. This is done for
 * performance reasons.
 *
 * @param point
 *            Coordinates of the vertex to add to the graph
 * @return Vertex created, null if the vertex was not added to the graph
 */
void GlobalPathPlanner::addPoint( const MRA::Geometry::Point& point) {
	//vertices,
	if (m_fieldConfig.isInReachableField(point.x, point.y) == false) {
		return; // only point inside the (complete) field should be added
	}
	// See if it is too close to another vertex
	for (std::vector<Vertex*>::iterator it = m_vertices.begin(); it != m_vertices.end(); ++it) {
		if (((*it) == 0) || (point.distanceTo((*it)->m_coordinate) < m_options.minimumEdgeLength)) {
			return; // too close to other vertex
		}
	}

	double shortestDistanceToAnyTarget = std::numeric_limits<double>::infinity();

	for (std::vector<Vertex *>::iterator it = m_target.begin(); it != m_target.end(); ++it) {
		double dist = point.distanceTo((*it)->m_coordinate);
		if (dist <= shortestDistanceToAnyTarget) {
			dist = shortestDistanceToAnyTarget;
		}
	}
	Vertex* vertex = new Vertex(point, shortestDistanceToAnyTarget);
	m_addPoints.push_back(vertex);
	m_vertices.push_back(vertex);
}

void GlobalPathPlanner::save_graph_as_svg(const TeamPlannerData& teamplanner_data, const vector<planner_piece_t>& path) {
	std::vector<MovingObject> myTeam = std::vector<MovingObject>();
	//myTeam.push_back(MovingObject(m_Me));
	for(std::vector<MRA::Geometry::Point>::size_type pos_idx = 0; pos_idx != m_teammates.size(); pos_idx++) {
		myTeam.push_back(MovingObject(m_teammates[pos_idx].x, m_teammates[pos_idx].y, 0.0, 0.0, 0.0, -1)); //TODO add id
	}

	std::vector<MovingObject> opponents = std::vector<MovingObject>();
	for(std::vector<MRA::Geometry::Point>::size_type pos_idx = 0; pos_idx != m_opponents.size(); pos_idx++) {
		opponents.push_back(MovingObject(m_opponents[pos_idx].x, m_opponents[pos_idx].y, 0.0, 0.0, 0.0, -1)); //TODO add id
	}

	team_planner_result_t player_paths = team_planner_result_t();
	PlayerPlannerResult playerResult = PlayerPlannerResult();
	playerResult.path = path;
	player_paths.push_back(playerResult);
	// TODO handle localball correctly towards svg
	SvgUtils::save_graph_as_svg(teamplanner_data, player_paths,
			m_options, m_vertices, game_state_e::NONE, -1, vector<player_type_e>(), vector<long>(), "red");
}


} // namespace
