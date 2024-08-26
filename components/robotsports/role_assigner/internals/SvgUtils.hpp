/**
 *  @file
 *  @brief   Utility class for plotting planner to svg file
 *  @curator Jürge van Eijck
 */
#ifndef SVGUTILS_H
#define SVGUTILS_H 1

#include <vector>
#include "WmTypes.h"
#include "FieldConfig.h"
#include "GlobalPathPlanner.hpp"
#include "TeamPlannerData.hpp"
#include "TeamPlannerParameters.hpp"
#include <string>

namespace MRA {

class SvgUtils {
public:
    static void plannerdata_to_svg(const std::vector<PlayerPlannerResult>& player_paths, const TeamPlannerData& data, const FieldConfig&  fieldConfig,
                                    const std::string& save_name);
    static void plannerdata_to_svg(const std::vector<PlayerPlannerResult>& player_paths, const TeamPlannerData& data, const FieldConfig&  fieldConfig,
                                    const std::string& save_name, const std::vector<PlayerPlannerResult>&  comparing_player_paths);

private:


	static FieldConfig m_fieldConfig;

	/*  Get svg x coordinate for given field X */
	static double svgX(double fieldX);

	/* Get svg x coordinate for given field Y */
	static double svgY(double fieldY);

	static bool doesDirectoryExists(const std::string& filename);
	static std::string boolToString(bool b);
	static void toLower(std::string& r_string);
};

} // namespace

#endif
