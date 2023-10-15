/**
 *  @file
 *  @brief   Represents a vertex in the graph.
 *  @curator Jürge van Eijck
 */
#ifndef VERTEX_H
#define VERTEX_H 1

#include <vector>
#include "Edge.h"
#include "Vector2D.h"

namespace trs {

class Vertex {

public:
	double m_minDistance;
	double m_extraCost;
	Vector2D m_coordinate;
	double m_straightLineToTarget;
	std::vector<Edge> m_neighbours;
	Vertex * m_pPrevious;

	Vertex(const Vector2D& aCoordinate, double distanceToTarget);
	Vertex(const Vector2D& aCoordinate, double distanceToTarget, double dCost);
	Vertex(const Vertex& rVertex);
	virtual ~Vertex();

	std::string toString();

	bool equals(const Vertex& v) const;

	double totalCosts() const;
private:
	Vertex() {
			/* empty*/
	};

};

// compare function for sorting
bool VertexCompareFunction (Vertex* t1, Vertex* t2);

};


#endif

