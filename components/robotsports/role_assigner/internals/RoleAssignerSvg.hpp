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

class RoleAssignerState;
class RoleAssignerInput;
class RoleAssignerOutput;
class RoleAssignerParameters;

class RoleAssignerSvg {
public:
    static void role_assigner_data_to_svg(const RoleAssignerInput& input,
                const RoleAssignerState& r_state,
                const RoleAssignerOutput& r_output,
                const RoleAssignerParameters& parameters,
                const std::string& save_name);

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
