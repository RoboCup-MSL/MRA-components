/**
 *  @file
 *  @brief  tool for testing teamplanner with xml files
 *  @curator Jürge van Eijck
 */
#ifndef XML_PLANNER_H
#define XML_PLANNER_H 1
#include <string>

#include "../RobotsportsRoleAssigner.hpp"

using namespace MRA;

class RoleAssignTestData {
public:
    MRA::RobotsportsRoleAssigner::InputType  input;
    MRA::RobotsportsRoleAssigner::ParamsType params;
};
//public:
//
//
//      TeamPlannerInput tp_input = {};
//        tp_input.gamestate = gameState;
//        tp_input.fieldConfig = fieldConfig;
//        tp_input.ball_pickup_position = pickup_pos;
//        tp_input.passIsRequired = passIsRequired;
//        tp_input.ball_present = ball_present;
//        tp_input.ball = ball.position;
//        tp_input.pass_data = pass_data;
//        tp_input.playerPassedBall = playerPassedBall;
//        tp_input.teamControlBall = team_has_ball;
//        for (auto idx = 0; idx < strategy_output.dynamic_roles_size(); idx++) {
//            tp_input.teamFormation.push_back((dynamic_role_e) strategy_output.dynamic_roles(idx));
//        }
//        tp_input.parking_positions = parking_postions;
//        tp_input.team = myTeam;
//        tp_input.opponents = opponents;
//
//        std::cout << "gameState: " << gameState << " (" << GameStateAsString(gameState) << " )"<< endl << flush;
//        std::cout << "tp_input.game_state_e: " << tp_input.gamestate << " (" << GameStateAsString(tp_input.gamestate) << " )"<< endl << flush;
//        printInputs(tp_input);
//
//        TeamPlannerState tp_state;
//        TeamPlannerOutput tp_output;
//        TeamPlannerParameters tp_parameters;
//        TeamPlay teamplay = TeamPlay();
//
//        teamplay.assign(tp_input, tp_state, tp_output, tp_parameter

void xmlplanner(std::string input_filename, RoleAssignTestData& r_data);

#endif

