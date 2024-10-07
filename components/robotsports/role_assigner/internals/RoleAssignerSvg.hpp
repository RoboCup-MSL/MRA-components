/**
 *  @file
 *  @brief   Utility class for plotting role assigner data to svg file
 *  @curator Jürge van Eijck
 */
#ifndef ROLE_ASSIGNER_SVG_HPP
#define ROLE_ASSIGNER_SVG_HPP 1

#include <string>
#include <vector>

#include "Environment.hpp"

namespace MRA {

class RoleAssignerResult;
class RoleAssignerData;

class RoleAssignerSvg {
public:
    static void role_assigner_data_to_svg(const std::vector<RoleAssignerResult>& player_paths, const RoleAssignerData& data, const Environment&  rEnvironment,
                                    const std::string& save_name);
    static void role_assigner_data_to_svg(const std::vector<RoleAssignerResult>& player_paths, const RoleAssignerData& data, const Environment&  Environment,
                                    const std::string& save_name, const std::vector<RoleAssignerResult>&  comparing_player_paths);

private:


    static Environment m_environment;

    /*  Get svg x coordinate for given field X */
    static double svgX(double fieldX);

    /* Get svg x coordinate for given field Y */
    static double svgY(double fieldY);

    static bool doesDirectoryExists(const std::string& filename);
    static std::string boolToString(bool b);
    static void toLower(std::string& r_string);
};

} // namespace

#endif // ROLE_ASSIGNER_SVG_HPP
