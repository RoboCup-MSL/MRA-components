/**
 *  @file
 *  @brief   Utility class for plotting planner to svg file
 *  @curator Jürge van Eijck
 */
#ifndef SVGUTILS_HPP
#define SVGUTILS_HPP 1

#include "GlobalPathPlanner.hpp"
#include "FieldConfig.hpp"

#include <string>
#include <vector>

#include "RoleAssignerData.hpp"
#include "RoleAssignerParameters.hpp"

namespace MRA {

class RoleAssignerSvg {
public:
    static void role_assigner_data_to_svg(const std::vector<RoleAssignerResult>& player_paths, const RoleAssignerData& data, const FieldConfig&  fieldConfig,
                                    const std::string& save_name);
    static void role_assigner_data_to_svg(const std::vector<RoleAssignerResult>& player_paths, const RoleAssignerData& data, const FieldConfig&  fieldConfig,
                                    const std::string& save_name, const std::vector<RoleAssignerResult>&  comparing_player_paths);

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

#endif // SVGUTILS_HPP
