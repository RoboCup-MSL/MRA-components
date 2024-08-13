/*
 * TeamPlannerGrid.h
 *
 *  Created on: Jan 3, 2016
 *      Author: jurge
 */

#ifndef TEAMPLANNERGRID_H_
#define TEAMPLANNERGRID_H_

#include "geometry.hpp"
#include "TeamPlay.hpp"
#include <string>

namespace MRA {

class PlannerGridInfoData;
class GridHeuristic;

class TeamPlanner_Grid {
public:
	typedef struct t_GridData
	{
		double x;
		double y;
		double z;
	} griddata_t;

	static MRA::Geometry::Position findBallPlayerPosition(const TeamPlannerData& r_teamplannerData, int gridFileNumber);


	static MRA::Geometry::Position findManToManDefensivePosition(dynamic_role_e dynamic_role, const MRA::Geometry::Point& oppentToDefend, const TeamPlannerData& r_teamplannerData,
			int gridFileNumber, bool setPlayActive);

	static MRA::Geometry::Position findDefensivePosition(const TeamPlannerData& r_teamplannerData, int gridFileNumber);

	static MRA::Geometry::Position findSweeperPosition(const TeamPlannerData& r_teamplannerData, int gridFileNumber);

	static MRA::Geometry::Position findInterceptorPositionDuringRestart(const TeamPlannerData& r_teamplannerData, int gridFileNumber);

	static bool  findAttackSupportPosition(MRA::Geometry::Point& rSupportPosition, const TeamPlannerData& r_teamplannerData, const Geometry::Point& r_balPositionToUse, int gridFileNumber, bool position_close_to_ball);

	static MRA::Geometry::Position findDefensivePositionDuringPenaltyShootOut(const TeamPlannerData& r_teamplannerData, int gridFileNumber);

	static MRA::Geometry::Position findSetPlayPosition(dynamic_role_e dynamic_role, const TeamPlannerData& r_teamplannerData,
	                                               const MRA::Geometry::Point& preferred_position, int gridFileNumber,
	                                               bool strongDesiredX, bool strongDesiredY, bool beAvailableForPass);


private:
	static void writeGridDataToFile(PlannerGridInfoData& pgid, const TeamPlannerData& r_teamplannerData, const Geometry::Point& r_balPositionToUse, const std::string& strSituation, int gridFileNumber);

	static double CalcInterceptionThreat(const TeamPlannerData& r_teamplannerData, int grid_x, int grid_y, double interceptionInfluenceDistance, double interceptionDistancePenaltyFactor);
	static double calculate_a_penaly_factor_for_teammate(double ball_ax_sqr, double ball_c);
	static double calc_a_penalty_factor(double radius, double c);
	static MRA::Geometry::Position calculateGridValues(const std::list<MRA::Geometry::Position>& allowedTargetPositions,
			std::vector<GridHeuristic*> heuristics, const TeamPlannerParameters& parameters, PlannerGridInfoData& pgid) ;

	static void handle_penalty_heuristics(const TeamPlannerData& r_teamplannerData, const Geometry::Point& r_balPositionToUse, std::vector<GridHeuristic*> &heuristics, PlannerGridInfoData &pgid);
};

} /* namespace trs */

#endif /* TEAMPLANNERGRID_H_ */
