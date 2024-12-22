/**
 *  @file
 *  @brief   Class for exporting role assigner data
 *  @curator Jürge van Eijck
 */
#ifndef ROLE_ASSIGNER_EXPORT_HPP
#define ROLE_ASSIGNER_EXPORT_HPP 1

#include <string>
#include "RoleAssigner_types.hpp"

namespace MRA {
    std::string GetRoleAssignerSVGname(game_state_e gamestate, std::string suffix = "");
} // namespace

#endif // ROLE_ASSIGNER_EXPORT_HPP
