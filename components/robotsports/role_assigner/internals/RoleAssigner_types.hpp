/**
 *  @file
 *  @brief   typedefs for planner
 *  @curator Jürge van Eijck
 */

#ifndef PLANNER_TYPES_HPP
#define PLANNER_TYPES_HPP 1

#include <string>

namespace MRA {

typedef enum  {
    RESERVE,
    FIELD_PLAYER,
    GOALIE
} player_type_e;

typedef enum  {
    GOTO_BALL = 0,
    DRIBBLE = 1,
    SUPPORT_DEFENSE = 2,
    SUPPORT_ATTACK = 3,
    PREPARE_RESTART_AGAINST = 4,
    PREPARE_RESTART = 5,
    PREPARE_DROPBALL = 6,
    GOTO_TARGET_POSITION = 7,
    SWEEPER = 8,
    GOALIE_POSITION = 9,
    GOTO_TARGET_POSITION_SLOW = 10,
    PRIORITY_BLOCK = 11,
    GOALKEEPER = 12
} planner_target_e;

typedef enum  {
    dr_NONE = 0,
    dr_GOALKEEPER = 1,
    dr_ATTACKSUPPORTER = 2,
    dr_DEFENDER = 3,
    dr_INTERCEPTOR = 4,
    dr_SWEEPER = 5,
    dr_SETPLAY_RECEIVER = 6,
    dr_SETPLAY_KICKER = 7,
    dr_BALLPLAYER = 8,
    dr_SEARCH_FOR_BALL = 9,
    dr_BEGIN_POSITION = 10,
    dr_PARKING = 11,
    dr_PENALTY_KICKER = 12,
    dr_LOB_CALIBRATION = 13,
    dr_PENALTY_DEFENDER = 14,
    dr_IS_ALIVE = 15
} dynamic_role_e;

typedef enum  {
    role_UNDEFINED = 0,
    role_GOALKEEPER = 1,
    role_ATTACKER_MAIN = 2,
    role_ATTACKER_ASSIST = 3,
    role_ATTACKER_GENERIC = 4,
    role_DEFENDER_MAIN = 5,
    role_DEFENDER_GENERIC = 6,
    role_DISABLED_OUT = 7,
    role_DISABLED_IN = 8
} role_e;   // matching with MRA::RobotsportsRobotStrategy::Output_DynamicRole

typedef enum  {
    FORMATION_112 = 0,
    FORMATION_013 = 1,
    FORMATION_211 = 2,
    FORMATION_310 = 3,
    FORMATION_ATTACK_SUPPORT_ONLY = 4,
    FORMATION_DEFENDER_ONLY = 5,
    FORMATION_INTERCEPTOR_ONLY = 6,
    FORMATION_SWEEPER_ONLY = 7,
    FORMATION_SETPLAY_RECEIVER_ONLY = 8,
    FORMATION_SETPLAY_KICKER_ONLY = 9,
    FORMATION_BALLPLAYER_ONLY = 10,
    FORMATION_SEARCHFORBALL_ONLY = 11,
    FORMATION_BEGINPOSITION_ONLY = 12,
    FORMATION_PARKING_ONLY = 13,
    FORMATION_PENALTYKICKER_ONLY = 14,
    FORMATION_PENALTY_SHOOTOUT = 15
} team_formation_e;

typedef enum {
    NONE = 0,
    NORMAL = 1,
    NORMAL_ATTACK = 2,
    NORMAL_DEFEND = 3,
    PARKING = 4,
    BEGIN_POSITION = 5,
    KICKOFF = 6,
    KICKOFF_AGAINST = 7,
    FREEKICK = 8,
    FREEKICK_AGAINST = 9,
    GOALKICK = 10,
    GOALKICK_AGAINST = 11,
    THROWIN = 12,
    THROWIN_AGAINST = 13,
    CORNER = 14,
    CORNER_AGAINST = 15,
    PENALTY = 16,
    PENALTY_AGAINST = 17,
    PENALTY_SHOOTOUT = 18,
    PENALTY_SHOOTOUT_AGAINST = 19,
    DROPPED_BALL = 20,
    YELLOW_CARD_AGAINST = 21,
    RED_CARD_AGAINST = 22,
    GOAL = 23,
    GOAL_AGAINST = 24,
    GAME_STATE_NR_ITEMS = 25
} game_state_e;


typedef enum {
    FREE = 0,
    OWNED_BY_PLAYER = 1,
    OWNED_BY_TEAMMATE = 2,
    OWNED_BY_TEAM = 3,
    OWNED_BY_OPPONENT = 4
} ball_status_e;

typedef struct path_piece {
    double x;
    double y;
    double cost;
    long target; // from enum-type planner_target_e
} path_piece_t;

typedef struct defend_info {
    long valid;
    long defending_id;
    double dist_from_defending_id;
    long between_ball_and_defending_pos; // true: defend between defending pos and ball, otherwise between defending_pos and own goal
} defend_info_t;


typedef struct previous_role_assigner_result  {
    bool present = false;
    double ts = 0;
    path_piece_t end_position;
    role_e role;
} previous_role_assigner_result_t;

typedef struct  previous_used_ball_by_role_assinger  {
    bool   present;
    double x;
    double y;
} previous_used_ball_by_role_assinger_t;

typedef struct moving_object_s {
    double x;
    double y;
    double rz;
    double velx;
    double vely;
    double velrz;
    long label;
    long valid;
} moving_object_t;

typedef struct ball_pickup_position_s {
    double x;
    double y;
    long valid;
    double ts;
} ball_pickup_position_t;


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
    case planner_target_e::GOALIE_POSITION:         targetString = "Goalie position"; break;
    case planner_target_e::GOTO_TARGET_POSITION_SLOW:  targetString = "goto target position slow"; break;
    case planner_target_e::PRIORITY_BLOCK:          targetString = "Priority Block"; break;
    case planner_target_e::GOALKEEPER:              targetString = "Goalie"; break;

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

inline std::string RoleAsString(role_e role) {
    std::string role_string = "";
    switch (role) {
        case role_UNDEFINED: role_string = "UNDEFINED"; break;
        case role_GOALKEEPER: role_string = "GOALKEEPER"; break;
        case role_ATTACKER_MAIN: role_string = "ATTACKER_MAIN"; break;
        case role_ATTACKER_ASSIST: role_string = "ATTACKER_ASSIST"; break;
        case role_ATTACKER_GENERIC: role_string = "ATTACKER_GENERIC"; break;
        case role_DEFENDER_MAIN: role_string = "DEFENDER_MAIN"; break;
        case role_DEFENDER_GENERIC: role_string = "DEFENDER_GENERIC"; break;
        case role_DISABLED_OUT: role_string = "DISABLED_OUT"; break;
        case role_DISABLED_IN: role_string = "DISABLED_IN"; break;
    default:
        role_string = "unknown role (ERROR situation)";
    }
    return role_string;
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


inline role_e DynamicRoleToRole(dynamic_role_e dr_role, role_e org_role) {
    role_e role = role_UNDEFINED;
    switch (dr_role) {
        case dr_NONE: role = role_UNDEFINED; break;
        case dr_GOALKEEPER: role = role_GOALKEEPER; break;
        case dr_ATTACKSUPPORTER: role = role_ATTACKER_GENERIC; break;
        case dr_DEFENDER: role = role_DEFENDER_GENERIC; break;
        case dr_INTERCEPTOR: role = role_ATTACKER_MAIN; break;
        case dr_SWEEPER: role = role_DEFENDER_MAIN; break;
        case dr_SETPLAY_RECEIVER: role = role_ATTACKER_ASSIST; break;
        case dr_SETPLAY_KICKER: role = role_ATTACKER_MAIN; break;
        case dr_BALLPLAYER: role = role_ATTACKER_MAIN; break;
        case dr_SEARCH_FOR_BALL: role = org_role; break;
        case dr_BEGIN_POSITION: role = org_role; break;
        case dr_PARKING: role = org_role; break;
        case dr_PENALTY_KICKER: role = org_role; break;
        case dr_LOB_CALIBRATION: role = org_role; break;
        case dr_PENALTY_DEFENDER: role = org_role; break;
        case dr_IS_ALIVE: role = org_role; break;
    } ;
    return role;
}

inline dynamic_role_e RoleToDynamicRole(role_e role, game_state_e gamestate, ball_status_e ball_status) {
    dynamic_role_e dr_role = dr_NONE;
    if (gamestate == BEGIN_POSITION and role != role_GOALKEEPER) {
        dr_role = dr_BEGIN_POSITION;
    }
    else if (gamestate == PARKING) {
        dr_role = dr_PARKING;
    }
    else {
        switch (role) {
            case role_UNDEFINED: dr_role = dr_NONE; break;
            case role_GOALKEEPER: dr_role = dr_GOALKEEPER; break;
            case role_ATTACKER_GENERIC: dr_role = dr_ATTACKSUPPORTER; break;
            case role_DEFENDER_GENERIC:
                if (isOneOf(gamestate, {PENALTY_SHOOTOUT, PENALTY_SHOOTOUT_AGAINST})) {
                    dr_role = dr_PENALTY_DEFENDER;
                }
                else {
                    dr_role = dr_DEFENDER;
                }
                break;
            case role_ATTACKER_MAIN:
                if (isOneOf(gamestate, {CORNER, FREEKICK, GOALKICK, KICKOFF, THROWIN})) {
                    dr_role = dr_SETPLAY_KICKER;
                }
                else if (isOneOf(gamestate, {PENALTY, PENALTY_SHOOTOUT})) {
                    dr_role = dr_PENALTY_KICKER;
                }
                else {
                    if (isOneOf(ball_status, {OWNED_BY_PLAYER, OWNED_BY_TEAMMATE})) {
                        dr_role = dr_BALLPLAYER;
                    }
                    else {
                        dr_role = dr_INTERCEPTOR;
                    }
                }
                break;
            case role_DEFENDER_MAIN: dr_role = dr_SWEEPER; break;
            case role_ATTACKER_ASSIST: dr_role = dr_SETPLAY_RECEIVER; break;
            case role_DISABLED_OUT: dr_role = dr_NONE; break; // not supported
            case role_DISABLED_IN: dr_role = dr_NONE; break;  // not supported

            //        case dr_role = dr_SETPLAY_KICKER: dr_role = role_ATTACKER_MAIN; break;
            //        case dr_role = dr_BALLPLAYER: dr_role = role_ATTACKER_MAIN; break;
            //        case dr_role = dr_SEARCH_FOR_BALL: dr_role = org_role; break;
            //        case dr_role = dr_BEGIN_POSITION: dr_role = org_role; break;
            //        case dr_role = dr_PARKING: dr_role = org_role; break;
            //        case dr_role = dr_PENALTY_KICKER: dr_role = org_role; break;
            //        case dr_role = dr_LOB_CALIBRATION: dr_role = org_role; break;
            //        case dr_role = dr_PENALTY_DEFENDER: dr_role = org_role; break;
            //        case dr_role = dr_IS_ALIVE: dr_role = org_role; break;
        } ;
    }

    return dr_role;
}

// dr_PENALTY_DEFENDER: role_DEFENDER_GENERIC and isOneOf(gamestate, {PENALTY_SHOOTOUT, PENALTY_SHOOTOUT_AGAINST}
// dr_DEFENDER : role_DEFENDER_GENERIC and NOT isOneOf(gamestate, {PENALTY_SHOOTOUT, PENALTY_SHOOTOUT_AGAINST}
// dr_ATTACKSUPPORTER role_ATTACKER_GENERIC
// dr_SETPLAY_RECEIVER role_ATTACKER_ASSIST
// dr_SWEEPER role_DEFENDER_MAIN
// dr_GOALKEEPER = role_GOALKEEPER
// dr_SETPLAY_KICKER role_ATTACKER_MAIN and isOneOf(gamestate, {CORNER, FREEKICK, GOALKICK, KICKOFF, THROWIN}
// dr_PENALTY_KICKER role_ATTACKER_MAIN and isOneOf(gamestate, {PENALTY, PENALTY_SHOOTOUT})
// dr_BALLPLAYER: role_ATTACKER_MAIN not isOneOf(gamestate, {CORNER, FREEKICK, GOALKICK, KICKOFF, THROWIN, PENALTY, PENALTY_SHOOTOUT} and isOneOf(ball_status, {OWNED_BY_PLAYER, OWNED_BY_TEAMMATE})
// dr_INTERCEPTOR: role_ATTACKER_MAIN not isOneOf(gamestate, {CORNER, FREEKICK, GOALKICK, KICKOFF, THROWIN, PENALTY, PENALTY_SHOOTOUT} and not isOneOf(ball_status, {OWNED_BY_PLAYER, OWNED_BY_TEAMMATE})


} // end namespace MRA

#endif // PLANNER_TYPES_HPP
