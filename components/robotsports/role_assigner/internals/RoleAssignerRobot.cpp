/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */

#include "RoleAssignerRobot.hpp"

#include "TeamPlannerData.hpp"

#include <iomanip>
#include <sstream>
#include <iostream>
#include <cmath>

using namespace std;
using namespace MRA;

std::string RoleAssignerRobot::toString() const {
    std::stringstream buffer;
    buffer << "ID: " << robotId   << " type: " << PlayerTypeAsString(player_type) << endl
           << "Pos: " << position.toString() << " Vel: " << velocity.toString() << " control ball: " <<  controlBall << endl
           << "pass is on its way: " << passBall << endl;
            //<< "assigned: " << assigned << endl;
// TODO
//    buffer << " dynamic role: " << DynamicRoleAsString(result.dynamic_role) << " (rank: " << result.role_rank << " )"
//                " gamestate: " << GameStateAsString(result.gamestate) << endl;
//    if (result.defend_info.valid) {
//        buffer << " Defend info: valid: true id: "<< result.defend_info.defending_id;
//        buffer << " dist to id: " << result.defend_info.dist_from_defending_id;
//        buffer << "    between ball and id: " << result.defend_info.between_ball_and_defending_pos << endl;
//    }
//    else {
//        buffer << " Defend info: valid: false" << endl;
//    }
//
//    buffer << "time_in_own_penalty_area:" << time_in_own_penalty_area
//           << " time_in_opponent_penalty_area: " << time_in_opponent_penalty_area << endl;
//    // TODO    buffer <<  " End position: " << result.target.toString() << endl;
//
//    if (previous_result.present) {
//        buffer << "previous dynamic role: " << DynamicRoleAsString(static_cast<dynamic_role_e>(previous_result.dynamic_role)) << endl;
//        buffer << "previous end pos (x, y, ts) : " << previous_result.end_position.x
//               << ",  " << previous_result.end_position.y << " on : " << previous_result.ts << endl;
//    }
//    else {
//        buffer << "NO previous_result" << endl;
//    }
//
//    if (!result.path.empty()) {
//        buffer <<  " target: " << PlannerTargetAsString(static_cast<planner_target_e>(result.path[result.path.size()-1].target))
//            << " (x: " <<  std::setprecision(3) <<  result.path[result.path.size()-1].x
//            << " y: " << result.path[result.path.size()-1].y << ")" << endl;
//    }
//    buffer << endl;
//            player_planner_result_t result;
//            bool have_previous_result;
//            previous_planner_result_t previous_result;;
    return buffer.str();
}


// compare function to sort vector of the class on the member robotId
bool RoleAssignerRobot::CompareRobotId(const RoleAssignerRobot& r1, const RoleAssignerRobot& r2)
{
    return (r1.robotId < r2.robotId);
}


