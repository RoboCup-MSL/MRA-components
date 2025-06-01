/*
 * RoleAssignerGrid.h
 *
 *  Created on: Jan 3, 2016
 *      Author: jurge
 */

#ifndef ROLEASSIGNERGRID_HPP
#define ROLEASSIGNERGRID_HPP

#include "geometry.hpp"
#include <string>

namespace MRA {

class RoleAssignerGridInfoData;
class GridHeuristic;
class RoleAssignerData;
class RoleAssignerParameters;

class RoleAssignerGrid {
public:
    typedef struct t_GridData
    {
        double x;
        double y;
        double z;
    } griddata_t;

    static MRA::Geometry::Position findBallPlayerPosition(const RoleAssignerData& r_role_assigner_data, int gridFileNumber);


    static MRA::Geometry::Position findManToManDefensivePosition(MRA::Datatypes::DynamicRole role, const MRA::Geometry::Point& oppentToDefend, const RoleAssignerData& r_role_assigner_data,
            int gridFileNumber, bool setPlayActive);

    static MRA::Geometry::Position findDefensivePosition(const RoleAssignerData& r_role_assigner_data, int gridFileNumber);

    static MRA::Geometry::Position findSweeperPosition(const RoleAssignerData& r_role_assigner_data, int gridFileNumber);

    static MRA::Geometry::Position findInterceptorPositionDuringRestart(const RoleAssignerData& r_role_assigner_data, int gridFileNumber);

    static bool  findAttackSupportPosition(MRA::Geometry::Point& rSupportPosition, const RoleAssignerData& r_role_assigner_data, const Geometry::Point& r_balPositionToUse, int gridFileNumber, bool position_close_to_ball);

    static MRA::Geometry::Position findDefensivePositionDuringPenaltyShootOut(const RoleAssignerData& r_role_assigner_data, int gridFileNumber);

    static MRA::Geometry::Position findSetPlayPosition(MRA::Datatypes::DynamicRole dynamic_role, const RoleAssignerData& r_role_assigner_data,
                                                   const MRA::Geometry::Point& preferred_position, int gridFileNumber,
                                                   bool strongDesiredX, bool strongDesiredY, bool beAvailableForPass);


private:
    static void writeGridDataToFile(RoleAssignerGridInfoData& pgid, const RoleAssignerData& r_role_assigner_data, const Geometry::Point& r_balPositionToUse, const std::string& strSituation, int gridFileNumber);

    static double CalcInterceptionThreat(const RoleAssignerData& r_role_assigner_data, int grid_x, int grid_y, double interceptionInfluenceDistance, double interceptionDistancePenaltyFactor);
    static double calculate_a_penaly_factor_for_teammate(double ball_ax_sqr, double ball_c);
    static double calc_a_penalty_factor(double radius, double c);
    static MRA::Geometry::Position calculateGridValues(const std::list<MRA::Geometry::Position>& allowedTargetPositions,
            std::vector<GridHeuristic*> heuristics, const RoleAssignerParameters& parameters, RoleAssignerGridInfoData& pgid) ;

    static void handle_penalty_heuristics(const RoleAssignerData& r_role_assigner_data, const Geometry::Point& r_balPositionToUse, std::vector<GridHeuristic*> &heuristics, RoleAssignerGridInfoData &pgid);
};

} /* namespace trs */

#endif // ROLEASSIGNERGRID_HPP
