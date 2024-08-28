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

//typedef struct planner_options_s {
//	long calculate_all_paths;
//	double minimum_edge_length;
//	double maximum_edge_length;
//	double minimum_distance_to_endpoint;
//	long nr_vertices_first_circle;
//	double first_circle_radius;
//	long nr_vertices_second_circle;
//	double second_circle_radius;
//	double safety_factor;
//	long add_barier_vertices;
//	long add_uniform_vertices;
//	double uniform_x_interval;
//	double uniform_y_interval;
//	double starting_velocity_penalty_factor;
//	double dist_to_apply_ball_approach_vertices;
//	long add_ball_approach_vertices;
//	double ball_approach_vertices_radius;
//	long ball_approach_number_of_vertices;
//	long man_defense_between_ball_and_player;
//	double dist_before_penalty_area_for_sweeper;
//	double grid_size;
//	double interceptionChanceStartDistance;
//	double interceptionChanceIncreasePerMeter;
//	double interceptionChancePenaltyFactor;
//	double grid_close_to_ball_normal_penalty;
//	double grid_close_to_ball_normal_radius;
//	double grid_close_to_ball_restart_normal_penalty;
//	double grid_close_to_ball_restart_normal_radius;
//	double grid_close_to_ball_restart_penalty_penalty;
//	double grid_close_to_ball_restart_penalty_radius;
//	double grid_close_to_ball_restart_dropball_penalty;
//	double grid_close_to_ball_restart_dropball_radius;
//	double grid_opponent_goal_clearance_x;
//	double grid_opponent_goal_clearance_y;
//	double grid_own_goal_clearance_x;
//	double grid_own_goal_clearance_y;
//	long nr_dynamic_planner_iterations;
//	double max_possible_linear_speed;
//	double max_possible_linear_acceleration;
//	long nr_robots_needed_for_pass_play;
//	long nr_attack_support_during_defensive_period;
//    long wait_on_non_optimal_position_during_prepare_phase;
//    long priority_block_apply;
//	double priority_block_max_ball_y;
//	double priority_block_max_opponent_to_ball_dist;
//    long priority_block_check_ball_in_area;
//    long priority_block_check_opponent_close_to_ball;
//    double priority_block_min_distance;
//    double priority_block_max_distance;
//    double priority_block_max_distance_to_defense_line;
//    double attack_supporter_extra_distance_to_stay_from_sideline;
//    long man_to_man_defense_during_normal_play;
//
//    long attack_formation;
//    long defense_formation;
//    double restart_receiver_ball_dist;
//    double restart_shooter_ball_dist;
//    double equality_cost_threshold;
//    long select_lowest_robot_nr_for_dynamic_role;
//	long previous_role_bonus_must_be_applied;
//	double previous_role_bonus_end_pos_radius;
//	long use_pass_to_position_for_attack_support;
//	long no_sweeper_during_setplay;
//	long interceptor_assign_use_ball_velocity;
//	double interceptor_assign_min_velocity_for_calculate_interception_position;
//	long man_to_man_defense_during_setplay_against;
//	double dist_to_goal_to_mark_opponent_as_goalie;
//	double setplay_against_dist_to_opponent;
//
//
//	long move_to_ball_left_field_position;
//
//    double auto_save_svg_period;
//
//	long save_grid_data_to_file;
//	long svg_robot_planner;
//
//	long preferred_setplaykicker;
//	long preferred_setplayreceiver;
//	double setplay_margin_to_penalty_area_side;
//	long dedicated_sweeper;
//	long auto_assign_goalie;
//	long lob_shot_when_possible;
//	double min_y_for_lob_shot;
//	double outside_field_margin;
//
//	double mobile_field_uniform_x_interval;
//	double mobile_field_uniform_y_interval;
//	double mobile_field_grid_size;
//	double mobile_field_grid_close_to_ball_normal_radius;
//	double mobile_field_grid_close_to_ball_restart_normal_radius;
//	double mobile_field_grid_close_to_ball_restart_penalty_radius;
//	double mobile_field_grid_close_to_ball_restart_dropball_radius;
//	double mobile_field_restart_receiver_ball_dist;
//	double mobile_field_restart_shooter_ball_dist;
//	double mobile_field_setplay_against_dist_to_opponent;
//
//	double kickoff_fp1_x;
//	double kickoff_fp1_y;
//	double kickoff_fp2_x;
//	double kickoff_fp2_y;
//	double kickoff_fp3_x;
//	double kickoff_fp3_y;
//	double kickoff_fp4_x;
//	double kickoff_fp4_y;
//
//	double kickoff_against_fp1_x;
//	double kickoff_against_fp1_y;
//	double kickoff_against_fp2_x;
//	double kickoff_against_fp2_y;
//	double kickoff_against_fp3_x;
//	double kickoff_against_fp3_y;
//	double kickoff_against_fp4_x;
//	double kickoff_against_fp4_y;
//
//	// variables for svg generation not in this struct
//} planner_options_t;
//


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
