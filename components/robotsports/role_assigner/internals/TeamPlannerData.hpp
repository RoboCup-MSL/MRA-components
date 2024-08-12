/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */
#ifndef TEAM_PLANNER_DATA_H
#define TEAM_PLANNER_DATA_H 1

#include "WmTypes.h"
#include "FieldConfig.h"
#include <vector>

#include "TeamPlannerOpponent.hpp"
#include "TeamPlannerParameters.hpp"
#include "TeamPlannerResult.hpp"

namespace MRA {

typedef enum  {
    RESERVE,
    FIELD_PLAYER,
    GOALIE
} player_type_e;


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



class TeamPlannerRobot {
public:
    bool active; // participating in the game (robot may be inactive when figuring out where it is)
    bool assigned;
    bool human;
    long robotId;
    long labelId;  // NEW
    bool controlBall;
    bool passBall; // indicator whether a pass by this player is still on its way
    player_type_e player_type;
    MRA::Geometry::Position position;
    MRA::Geometry::Position velocity;
    final_planner_result_t previous_result;
    double time_in_own_penalty_area;
    double time_in_opponent_penalty_area;

    PlayerPlannerResult result = {};
    static bool CompareRobotId(const TeamPlannerRobot& r1, const TeamPlannerRobot&  r2);
    std::string toString() const;
};

class TeamPlannerBall {
public:
    MRA::Geometry::Position position;
    MRA::Geometry::Position velocity;
    double confidence;
    bool is_valid;

    std::string toString(bool full_details) const;
};

// class with state data (data for State.proto)
class TeamPlannerState {
public:
    previous_used_ball_by_planner_t previous_global_ball;
};

// class with outputs (data for Output.proto)
class TeamPlannerOutput {
public:
    std::vector<PlayerPlannerResult> player_paths;
    std::string pathToString();
};

// part of  input data
typedef struct pass_data_s {
    long   valid; // 1: if data is valid; 0 otherwise
    long   kicked; // 1: if pass/shot has been made; 0: otherwise
    long   target_id; // destination of kick, where 0 is goal
    double velocity; // [m/s]
    double angle; // upwards angle
    MRA::Geometry::Position  origin_pos; // field coordinates of origin
    MRA::Geometry::Position  target_pos; // field coordinates of target
    double ts; // timestamp of update
    double eta; // estimated time of arrival at target
} pass_data_t;


// class with inputs (data for input.proto)
class TeamPlannerInput {
public:
    TeamPlannerInput() {};
    game_state_e gamestate;
    bool ball_present;
    MRA::Geometry::Position ball;

    std::vector<TeamPlannerRobot> team;
    std::vector<TeamPlannerOpponent> opponents;
    std::vector<MRA::Geometry::Position> parking_positions;
    ball_pickup_position_t ball_pickup_position;
    bool passIsRequired;
    pass_data_t pass_data;
    std::vector<dynamic_role_e> teamFormation;
    MRA::FieldConfig fieldConfig;
    bool teamControlBall;
    bool playerPassedBall;
};


//-----------------------------------------------------------------------
// Internal administration classes


class TeamPlannerData {
public:
    TeamPlannerData() {};
    /* inputs */
    game_state_e gamestate;
    bool ball_present;
    TeamPlannerBall ball;
    std::vector<MRA::Geometry::Point> parking_positions;
    ball_pickup_position_t ball_pickup_position;
    bool passIsRequired;
    pass_data_t pass_data;
    std::vector<dynamic_role_e> teamFormation;
    MRA::FieldConfig fieldConfig;
    TeamPlannerParameters parameters;

    // based on inputs
    ball_status_e ball_status;
    bool ballIsObstacle;
    bool searchForBall;
    int playerWhoIsPassing;
    defend_info_t defend_info;
    previous_used_ball_by_planner_t previous_ball = {};

    // internal administration
    game_state_e original_gamestate;
    std::vector<TeamPlannerRobot> team = {}; // Team will be sorted on robotId inside the role assigner (deterministic order)
    std::vector<TeamPlannerOpponent> opponents = {};
    std::vector<TeamPlannerOpponent> original_opponents  = {};
    int nr_players_assigned = 0;

    unsigned this_player_idx = 0; // idex of this robot: Team will be sorted on RobotId.
    unsigned this_player_robotId = 0; // robotId of this robot: Team will be sorted on RobotId.

    int incrementAndGetRank();

    bool teamControlsBall() const;
};

} // namespace

#endif /* TEAM_PLANNER_DATA_H */
