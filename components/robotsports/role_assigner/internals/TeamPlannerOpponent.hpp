/**
 *  @file
 *  @brief   Class for team planning 2017
 *  @curator Jürge van Eijck
 */
#ifndef TeamPlannerOpponent_H
#define TeamPlannerOpponent_H 1

namespace MRA {

class TeamPlannerOpponent {
public:
    MRA::Geometry::Position position;
    MRA::Geometry::Position velocity;
    bool assigned;
    int label;
};

} // namespace

#endif /* TeamPlannerOpponent_H */
