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

	static MRA::Geometry::Point findBallPlayerPosition(const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate, const TeamPlannerParameters& plannerOptions,
			const MRA::Geometry::Pose& ball, const std::vector<TeamPlannerOpponent>& Opponents, int gridFileNumber, const FieldConfig& fieldConfig,
			const ball_pickup_position_t& m_ball_pickup_position, bool passIsRequired);


	static MRA::Geometry::Point findManToManDefensivePosition(dynamic_role_e dynamic_role, const MRA::Geometry::Point& oppentToDefend, const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate,
			const TeamPlannerParameters& plannerOptions, 	const MRA::Geometry::Pose& ball, const std::vector<TeamPlannerOpponent>& Opponents,
			int gridFileNumber, const FieldConfig& fieldConfig, bool setPlayActive, bool teamControlBall);

	static MRA::Geometry::Point findDefensivePosition(const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate, const TeamPlannerParameters& plannerOptions,
			const MRA::Geometry::Pose& ball, const std::vector<TeamPlannerOpponent>& Opponents, int gridFileNumber, const FieldConfig& fieldConfig);

	static MRA::Geometry::Point findSweeperPosition(const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate,
			const TeamPlannerParameters& plannerOptions, const MRA::Geometry::Pose& ball, const std::vector<TeamPlannerOpponent>& Opponents,
		    int gridFileNumber, const FieldConfig& fieldConfig);

	static MRA::Geometry::Point findInterceptorPositionDuringRestart(const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate,
			const TeamPlannerParameters& plannerOptions, 	const MRA::Geometry::Pose& ball, const std::vector<TeamPlannerOpponent>& Opponents,
		    int gridFileNumber, const FieldConfig& fieldConfig);

	static bool  findAttackSupportPosition(MRA::Geometry::Point& rSupportPosition, const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate,
			const TeamPlannerParameters& plannerOptions, const MRA::Geometry::Pose& ball, const std::vector<TeamPlannerOpponent>& Opponents,
			int gridFileNumber, const FieldConfig& fieldConfig, bool position_close_to_ball, bool teamControlBall);

	static MRA::Geometry::Point findDefensivePositionDuringPenaltyShootOut(const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate,
			const TeamPlannerParameters& plannerOptions, 	const MRA::Geometry::Pose& ball, const std::vector<TeamPlannerOpponent>& Opponents,
			int gridFileNumber, const FieldConfig& fieldConfig);

private:
	static void writeGridDataToFile(PlannerGridInfoData& pgid, const std::vector<TeamPlannerRobot>& Team, const std::vector<TeamPlannerOpponent>& Opponents,
			const MRA::Geometry::Pose& ball, const TeamPlannerParameters& plannerOptions, const std::string& sMRAituation, int gridFileNumber);
	static double CalcInterceptionThreat(const std::vector<TeamPlannerRobot>& m_Team, int grid_x, int grid_y,
			const std::vector<TeamPlannerOpponent>& opponents, double interceptionInfluenceDistance, double interceptionDistancePenaltyFactor);
	static double calculate_a_penaly_factor_for_teammate(double ball_ax_sqr, double ball_c);
	static double calc_a_penalty_factor(double radius, double c);
	static MRA::Geometry::Point calculateGridValues(const std::list<MRA::Geometry::Point>& allowedTargetPositions,
			std::vector<GridHeuristic*> heuristics, const TeamPlannerParameters& plannerOptions, PlannerGridInfoData& pgid) ;

	static void handle_penalty_heuristics(game_state_e gamestate,
			const TeamPlannerParameters &plannerOptions,
			const std::vector<TeamPlannerRobot>& Team,
			const MRA::Geometry::Point& r_ballPos,
			const FieldConfig &fieldConfig,
			std::vector<GridHeuristic*> &heuristics,
			PlannerGridInfoData &pgid);
};

} /* namespace MRA */

#endif /* TEAMPLANNERGRID_H_ */
