/**
 *  @file
 *  @brief   Utility class for plotting planner to svg file
 *  @curator Jürge van Eijck
 */
#include <vector>
#include "MovingObject.h"
#include "WmTypes.h"
#include "FieldConfig.h"
#include "GlobalPathPlanner.hpp"
#include "PlannerOptions.hpp"
#include "TeamPlannerData.hpp"

namespace trs {

class SvgUtils {
public:

	static void save_graph_as_svg(const MovingObject& globalBall, const std::vector<MovingObject>& myTeam,
			const std::vector<MovingObject>& opponents, const team_planner_result_t& player_paths,
			const PlannerOptions& options, const std::vector<Vertex* >& vertices,
			game_state_e gamestate, long controlBallByPlayer,
			const std::vector<player_type_e>& teamTypes,
			const std::vector<long>& robotIds,
			const std::string& meColor, const FieldConfig& fieldConfig,
			bool hasTeamPlannerInputInfo, const TeamPlannerInputInfo&  inputInfo);

	static void save_graph_as_svg(const MovingObject& globalBall, const std::vector<MovingObject>& myTeam,
			const std::vector<MovingObject>& opponents, const team_planner_result_t& player_paths,
			const team_planner_result_t&  comparing_player_paths,
			const PlannerOptions& options, const std::vector<Vertex* >& vertices,
			game_state_e gamestate, long controlBallByPlayer,
			const std::vector<player_type_e>& teamTypes,
			const std::vector<long>& robotIds,
			const std::string& meColor, const FieldConfig& fieldConfig,
			bool hasTeamPlannerInputInfo, const TeamPlannerInputInfo&  inputInfo);

private:
	static FieldConfig m_fieldConfig;
/**
 * Get svg x coordinate for given field X
 */
		static double svgX(double fieldX);

/**
 * Get svg x coordinate for given field Y
 */
		static double svgY(double fieldY);

		static bool doesDirectoryExists(const std::string& filename);
};

} // namespace
