#ifndef WMTYPES_H
#define WMTYPES_H
#include <string>
#include "planner_types.hpp"

namespace MRA {


inline std::string GameStateAsString(game_state_e gamestate)
{
    std::string gamestate_remark = "";
    switch (gamestate) {

    case NONE:                     gamestate_remark = "NONE"; break;
    case NORMAL:                   gamestate_remark = "Normal"; break;
    case NORMAL_ATTACK:            gamestate_remark = "Normal attack"; break;
    case NORMAL_DEFEND:            gamestate_remark = "Normal defend"; break;
    case PARKING:                  gamestate_remark = "Parking"; break;
    case BEGIN_POSITION:           gamestate_remark = "Begin Position"; break;
    case KICKOFF:                  gamestate_remark = "Kickoff"; break;
    case KICKOFF_AGAINST:          gamestate_remark = "Kickoff Against"; break;
    case FREEKICK:                 gamestate_remark = "Freekick"; break;
    case FREEKICK_AGAINST:         gamestate_remark = "Freekick Against"; break;
    case GOALKICK:                 gamestate_remark = "Goalkick"; break;
    case GOALKICK_AGAINST:         gamestate_remark = "Goalkick Against"; break;
    case THROWIN:                  gamestate_remark = "Throwin"; break;
    case THROWIN_AGAINST:          gamestate_remark = "Throwin Against"; break;
    case CORNER:                   gamestate_remark = "Corner"; break;
    case CORNER_AGAINST:           gamestate_remark = "Corner Against"; break;
    case PENALTY:                  gamestate_remark = "Penalty"; break;
    case PENALTY_AGAINST:          gamestate_remark = "Penalty Against"; break;
    case PENALTY_SHOOTOUT:         gamestate_remark = "Penalty Shootout"; break;
    case PENALTY_SHOOTOUT_AGAINST: gamestate_remark = "Penalty Shootout Against"; break;
    case DROPPED_BALL:             gamestate_remark = "Dropped Ball"; break;
    case YELLOW_CARD_AGAINST:      gamestate_remark = "Yellow Card Against"; break;
    case RED_CARD_AGAINST:         gamestate_remark = "Red Card Against"; break;
    case GOAL:                     gamestate_remark = "Goal"; break;
    case GOAL_AGAINST:             gamestate_remark = "Goal Against"; break;
    default:
        gamestate_remark = "unknown game-state (ERROR situation)";
    }
    return gamestate_remark;
}


inline std::string PlannerTargetAsString(planner_target_e target) {
    std::string targetString = "";
    switch (target) {
    case planner_target_e::GOTO_BALL:               targetString = "Goto ball"; break;
    case planner_target_e::DRIBBLE:                 targetString = "Dribble"; break;
    case planner_target_e::SUPPORT_DEFENSE:         targetString = "Defense support"; break;
    case planner_target_e::SUPPORT_ATTACK:          targetString = "Attack support"; break;
    case planner_target_e::PREPARE_RESTART_AGAINST: targetString = "Prepare restart against"; break;
    case planner_target_e::PREPARE_RESTART:         targetString = "Prepare restart"; break;
    case planner_target_e::PREPARE_DROPBALL:        targetString = "Prepare dropball"; break;
    case planner_target_e::GOTO_TARGET_POSITION:    targetString = "goto target position"; break;
    case planner_target_e::SWEEPER:                 targetString = "Sweeper"; break;
    case planner_target_e::GOALIE_POSITION:         targetString = "Goalie"; break;
    case planner_target_e::GOTO_TARGET_POSITION_SLOW:  targetString = "goto target position slow"; break;
    case planner_target_e::PRIORITY_BLOCK:          targetString = "Priority Block"; break;
    default:
        targetString = "unknown target (ERROR situation)";
    }

    return targetString;
}

inline std::string DynamicRoleAsString(dynamic_role_e dynamic_role) {
    std::string dynamic_role_string = "";
    switch (dynamic_role) {
    case dynamic_role_e::dr_NONE: dynamic_role_string = "NONE"; break;
    case dynamic_role_e::dr_GOALKEEPER: dynamic_role_string = "GOALKEEPER"; break;
    case dynamic_role_e::dr_ATTACKSUPPORTER: dynamic_role_string = "ATTACKSUPPORTER"; break;
    case dynamic_role_e::dr_DEFENDER: dynamic_role_string = "DEFENDER"; break;
    case dynamic_role_e::dr_INTERCEPTOR: dynamic_role_string = "INTERCEPTOR"; break;
    case dynamic_role_e::dr_SWEEPER: dynamic_role_string = "SWEEPER"; break;
    case dynamic_role_e::dr_SETPLAY_RECEIVER: dynamic_role_string = "SETPLAY_RECEIVER"; break;
    case dynamic_role_e::dr_SETPLAY_KICKER: dynamic_role_string = "SETPLAY_KICKER"; break;
    case dynamic_role_e::dr_BALLPLAYER: dynamic_role_string = "BALLPLAYER"; break;
    case dynamic_role_e::dr_SEARCH_FOR_BALL: dynamic_role_string = "SEARCH FOR BALL"; break;
    case dynamic_role_e::dr_BEGIN_POSITION: dynamic_role_string = "BEGIN_POSITION"; break;
    case dynamic_role_e::dr_PARKING: dynamic_role_string = "PARKING"; break;
    case dynamic_role_e::dr_PENALTY_KICKER: dynamic_role_string = "PENALTY_KICKER"; break;
    case dynamic_role_e::dr_PENALTY_DEFENDER: dynamic_role_string = "PENALTY_DEFENDER"; break;
    case dynamic_role_e::dr_LOB_CALIBRATION: dynamic_role_string = "LOB_CALIBRATION"; break;
    default:
        dynamic_role_string = "unknown role (ERROR situation)";
    }
    return dynamic_role_string;
}

inline dynamic_role_e StringToDynamicRole(std::string dynamic_role_str) {
    dynamic_role_e dynamic_role = dynamic_role_e::dr_NONE;

    if (dynamic_role_str == "NONE") {
        dynamic_role = dynamic_role_e::dr_NONE;
    }
    else if (dynamic_role_str == "GOALKEEPER") {
        dynamic_role = dynamic_role_e::dr_GOALKEEPER;
    }
    else if (dynamic_role_str == "ATTACKSUPPORTER") {
        dynamic_role = dynamic_role_e::dr_ATTACKSUPPORTER;
    }
    else if (dynamic_role_str == "DEFENDER") {
        dynamic_role = dynamic_role_e::dr_DEFENDER;
    }
    else if (dynamic_role_str == "INTERCEPTOR") {
        dynamic_role = dynamic_role_e::dr_INTERCEPTOR;
    }
    else if (dynamic_role_str == "SWEEPER") {
        dynamic_role = dynamic_role_e::dr_SWEEPER;
    }
    else if (dynamic_role_str == "SETPLAY_RECEIVER") {
        dynamic_role = dynamic_role_e::dr_SETPLAY_RECEIVER;
    }
    else if (dynamic_role_str == "SETPLAY_KICKER") {
        dynamic_role = dynamic_role_e::dr_SETPLAY_KICKER;
    }
    else if (dynamic_role_str == "BALLPLAYER") {
        dynamic_role = dynamic_role_e::dr_BALLPLAYER;
    }
    else if (dynamic_role_str == "SEARCH FOR BALL") {
        dynamic_role = dynamic_role_e::dr_SEARCH_FOR_BALL;
    }
    else if (dynamic_role_str == "BEGIN_POSITION") {
        dynamic_role = dynamic_role_e::dr_BEGIN_POSITION;
    }
    else if (dynamic_role_str == "PARKING") {
        dynamic_role = dynamic_role_e::dr_PARKING;
    }
    else if (dynamic_role_str == "PENALTY_KICKER") {
        dynamic_role = dynamic_role_e::dr_PENALTY_KICKER;
    }
    else if (dynamic_role_str == "PENALTY_DEFENDER") {
        dynamic_role = dynamic_role_e::dr_PENALTY_DEFENDER;
    }
    else if (dynamic_role_str == "LOB_CALIBRATION") {
        dynamic_role = dynamic_role_e::dr_LOB_CALIBRATION;
    }
    return dynamic_role;
}

inline std::string ballStatusAsString(ball_status_e ball_status) {
    std::string result = "";
    switch (ball_status) {
    case ball_status_e::FREE: result = "FREE"; break;
    case ball_status_e::OWNED_BY_PLAYER: result = "OWNED_BY_PLAYER"; break;
    case ball_status_e::OWNED_BY_TEAMMATE  : result = "OWNED_BY_TEAMMATE"; break;
    case ball_status_e::OWNED_BY_TEAM: result = "OWNED_BY_TEAM"; break;
    case ball_status_e::OWNED_BY_OPPONENT: result = "OWNED_BY_OPPONENT"; break;
    }
    return result;
}

// function checks whether value is one of the given values.
// function can be use to check against enum values e.g. isOneOf(myFruit, {APPLE, PEAR})
template <typename T>
bool isOneOf(T value, std::initializer_list<T> values) {
    for (auto v : values) {
        if (value == v) {
            return true;
        }
    }
    return false;
}


} // namespace MRA

#endif // WMTYPES_H
