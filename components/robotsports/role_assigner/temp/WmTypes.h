#ifndef WMTYPES_H
#define WMTYPES_H
#include <string>
#include "planner_types.hpp"

namespace trs {

typedef enum  {
	RESERVE,
	FIELD_PLAYER,
	GOALIE
} player_type_e;


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


inline std::string PlayerTypeAsString(player_type_e player_type) {
	std::string player_type_string = "";
	switch (player_type) {
	case player_type_e::RESERVE:      player_type_string = "RESERVE"; break;
	case player_type_e::FIELD_PLAYER: player_type_string = "FIELD_PLAYER"; break;
	case player_type_e::GOALIE:       player_type_string = "GOALIE"; break;
	default:
		player_type_string = "player_type_e (ERROR situation)";
	}
	return player_type_string;
}

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
	case planner_target_e::GOALIE:                  targetString = "Goalie"; break;
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


} // namespace trs

#endif // WMTYPES_H
