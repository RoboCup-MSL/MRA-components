/**
 *  @file
 *  @brief   Represents a vertex in the graph.
 *  @curator Jürge van Eijck
 */
#ifndef VERTEX_HPP
#define VERTEX_HPP 1

#include "Edge.hpp"
#include "geometry.hpp"

#include <vector>

namespace MRA {

class Vertex {

public:
    double m_minDistance;
    double m_extraCost;
    MRA::Geometry::Point m_coordinate;
    double m_straightLineToTarget;
    std::vector<Edge> m_neighbours;
    Vertex * m_pPrevious;

    Vertex(const MRA::Geometry::Point& aCoordinate, double distanceToTarget);
    Vertex(const MRA::Geometry::Point& aCoordinate, double distanceToTarget, double dCost);
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


#endif // VERTEX_HPP

