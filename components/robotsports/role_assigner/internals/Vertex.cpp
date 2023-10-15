/**
 *  @file
 *  @brief   Represents a vertex in the graph.
 *  @curator Jürge van Eijck
 */
#include <vector>
#include "Vertex.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <ostream>
#include <sstream>

using namespace std;

namespace trs {

Vertex::Vertex(const Vector2D& aCoordinate, double distanceToTarget) :
		m_minDistance(std::numeric_limits<double>::infinity()),
		m_extraCost(0),
		m_coordinate(aCoordinate),
		m_straightLineToTarget(distanceToTarget),
		m_neighbours(std::vector<Edge>()),
		m_pPrevious(0)
		{
		}

Vertex::Vertex(const Vector2D& aCoordinate, double distanceToTarget, double dCost) :
		m_minDistance(std::numeric_limits<double>::infinity()),
		m_extraCost(dCost),
		m_coordinate(aCoordinate),
		m_straightLineToTarget(distanceToTarget),
		m_neighbours(std::vector<Edge>()),
		m_pPrevious(0)
		{
		}

Vertex::Vertex(const Vertex& rVertex) :
		m_minDistance(rVertex.m_minDistance),
		m_extraCost(rVertex.m_extraCost),
		m_coordinate(rVertex.m_coordinate),
		m_straightLineToTarget(rVertex.m_straightLineToTarget),
		m_neighbours(rVertex.m_neighbours),
		m_pPrevious(rVertex.m_pPrevious) {
}


Vertex::~Vertex() {
	m_neighbours.clear();
}

std::string Vertex::toString() {
	std::stringstream buffer;
	buffer << m_coordinate.toString() << " min dist: " << m_minDistance << " extra cost: " << m_extraCost
			<< "nr neighbours: "<< m_neighbours.size();
	return buffer.str();
}

bool Vertex::equals(const Vertex& v) const {
	bool c = m_coordinate.equals(v.m_coordinate);
//	bool d = fabs(straightLineToTarget - v.straightLineToTarget) < eps;
//	if ( (fabs(minDistance - v.minDistance) < eps) && (coordinate.equals(v.coordinate))
//			&& (fabs(straightLineToTarget - v.straightLineToTarget) < eps) ){
//		    //&& (v.previous == 0 && previous == 0) && (v.previous != 0 && v.previous.equals(previous))
//		return true;
//	}
	return c;
}

double Vertex::totalCosts() const {
	return m_minDistance + m_extraCost;
}

//    public Vertex previous;
//

bool VertexCompareFunction (Vertex* t1, Vertex* t2)
{
    	return (t1->m_minDistance + t1->m_straightLineToTarget) < (t2->m_minDistance + t2->m_straightLineToTarget);
}



} // namespace

