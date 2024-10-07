/**
 *  @file
 *  @brief   Represents a vertex in the graph..
 *  @curator Jürge van Eijck
 */
#ifndef EDGE_HPP
#define EDGE_HPP 1

namespace MRA {

class Vertex;

/**
 * Represents an edge in the graph.
 * The members are the target Vertex and the cost of the edge
 */
class Edge {
    public:
        MRA::Vertex* m_pTarget;
        double m_dCost;

        Edge(Vertex *pTarget, double dCost);
        Edge(const Edge& e);
};

}
#endif /* EDGE_HPP */
