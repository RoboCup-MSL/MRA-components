/*
 * TeamPlannerGrid.h
 *
 *  Created on: Jan 3, 2016
 *      Author: jurge
 */

#ifndef TEAMPLANNERGRID_H_
#define TEAMPLANNERGRID_H_

#include "Vector2D.h"
#include "MovingObject.h"
#include  "TeamPlay.h"

namespace trs {

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

	static Vector2D findBallPlayerPosition(const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate, const PlannerOptions& plannerOptions,
			const MovingObject& ball, const std::vector<TeamPlannerOpponent>& Opponents, int gridFileNumber, const FieldConfig& fieldConfig,
			const ball_pickup_position_t& m_ball_pickup_position, bool passIsRequired);


	static Vector2D findManToManDefensivePosition(dynamic_role_e dynamic_role, const Vector2D& oppentToDefend, const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate,
			const PlannerOptions& plannerOptions, 	const MovingObject& ball, const std::vector<TeamPlannerOpponent>& Opponents,
			int gridFileNumber, const FieldConfig& fieldConfig, bool setPlayActive, bool teamControlBall);

	static Vector2D findDefensivePosition(const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate, const PlannerOptions& plannerOptions,
			const MovingObject& ball, const std::vector<TeamPlannerOpponent>& Opponents, int gridFileNumber, const FieldConfig& fieldConfig);

	static Vector2D findSweeperPosition(const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate,
			const PlannerOptions& plannerOptions, 	const MovingObject& ball, const std::vector<TeamPlannerOpponent>& Opponents,
		    int gridFileNumber, const FieldConfig& fieldConfig);

	static Vector2D findInterceptorPositionDuringRestart(const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate,
			const PlannerOptions& plannerOptions, 	const MovingObject& ball, const std::vector<TeamPlannerOpponent>& Opponents,
		    int gridFileNumber, const FieldConfig& fieldConfig);

	static bool  findAttackSupportPosition(Vector2D& rSupportPosition, const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate,
			const PlannerOptions& plannerOptions, const MovingObject& ball, const std::vector<TeamPlannerOpponent>& Opponents,
			int gridFileNumber, const FieldConfig& fieldConfig, bool position_close_to_ball, bool teamControlBall);

	static Vector2D findDefensivePositionDuringPenaltyShootOut(const std::vector<TeamPlannerRobot>& Team, game_state_e gamestate,
			const PlannerOptions& plannerOptions, 	const MovingObject& ball, const std::vector<TeamPlannerOpponent>& Opponents,
			int gridFileNumber, const FieldConfig& fieldConfig);

private:
	static void writeGridDataToFile(PlannerGridInfoData& pgid, const std::vector<TeamPlannerRobot>& Team, const std::vector<TeamPlannerOpponent>& Opponents,
			const MovingObject& ball, const PlannerOptions& plannerOptions, const string& strSituation, int gridFileNumber);
	static double CalcInterceptionThreat(const std::vector<TeamPlannerRobot>& m_Team, int grid_x, int grid_y,
			const std::vector<TeamPlannerOpponent>& opponents, double interceptionInfluenceDistance, double interceptionDistancePenaltyFactor);
	static double calculate_a_penaly_factor_for_teammate(double ball_ax_sqr, double ball_c);
	static double calc_a_penalty_factor(double radius, double c);
	static Vector2D calculateGridValues(const std::list<Vector2D>& allowedTargetPositions,
			std::vector<GridHeuristic*> heuristics, const PlannerOptions& plannerOptions, PlannerGridInfoData& pgid) ;

	static void handle_penalty_heuristics(game_state_e gamestate,
			const PlannerOptions &plannerOptions,
			const std::vector<TeamPlannerRobot>& Team,
			const Vector2D& r_ballPos,
			const FieldConfig &fieldConfig,
			vector<GridHeuristic*> &heuristics,
			PlannerGridInfoData &pgid);
};

} /* namespace trs */

#endif /* TEAMPLANNERGRID_H_ */
