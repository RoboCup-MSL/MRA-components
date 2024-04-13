/**
 *  @file
 *  @brief   Utility class for plotting planner to svg file
 *  @curator Jürge van Eijck
 */
#include <vector>
#include "WmTypes.h"
#include "FieldConfig.h"
#include "GlobalPathPlanner.hpp"
#include "TeamPlannerData.hpp"
#include "TeamPlannerParameters.hpp"

namespace MRA {

class SvgUtils {
public:

	static void save_graph_as_svg(const TeamPlannerData & teamplanner_data,
	        const team_planner_result_t& player_paths,
	        const std::vector<Vertex* >& vertices,
			const std::string& meColor);

	static void save_graph_as_svg(const TeamPlannerData & teamplanner_data,
	        const team_planner_result_t& player_paths,
	        const team_planner_result_t&  comparing_player_paths,
			const std::vector<Vertex* >& vertices,
			const std::string& meColor);

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