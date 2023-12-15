/**
 *  @file
 *  @brief   Represents a vertex in the graph..
 *  @curator Jürge van Eijck
 */
#include "Edge.hpp"

#include "Vertex.hpp"
namespace trs {

Edge::Edge(Vertex* pTarget, double dCost) :
		m_pTarget(pTarget),
		m_dCost(dCost) {

}

Edge::Edge(const Edge& e) :
		m_pTarget(e.m_pTarget),
		m_dCost(e.m_dCost) {

}

}

