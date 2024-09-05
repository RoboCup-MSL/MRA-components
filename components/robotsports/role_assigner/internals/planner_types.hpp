/**
 *  @file
 *  @brief   typedefs for planner
 *  @curator Jürge van Eijck
 */

#ifndef PLANNER_TYPES_H
#define PLANNER_TYPES_H 1

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

typedef struct planner_piece {
    double x;
    double y;
    double cost;
    long target; // from enum-type planner_target_e
} planner_piece_t;

typedef struct defend_info {
    long valid;
    long defending_id;
    double dist_from_defending_id;
    long between_ball_and_defending_pos; // true: defend between defending pos and ball, otherwise between defending_pos and own goal
} defend_info_t;


typedef struct previous_planner_result  {
    long previous_result_present;
    double ts;
    planner_piece_t end_position;
    long dynamic_role;
} final_planner_result_t;

typedef struct previous_used_ball_by_planner  {
    long previous_ball_present;
    double x;
    double y;
} previous_used_ball_by_planner_t;


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

} // end namespace MRA

#endif
