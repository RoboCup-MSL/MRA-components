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
    bool controlBall;
    bool passBall; // indicator whether a pass by this player is still on its way
    player_type_e player_type;
    MRA::Geometry::Pose position;
    MRA::Geometry::Pose velocity;
    PlayerPlannerResult result;
    final_planner_result_t previous_result;
    dynamic_role_e dynamic_role;
    double time_in_own_penalty_area;
    double time_in_opponent_penalty_area;

    std::string toString() const;

};

class TeamPlannerBall {
public:
    MRA::Geometry::Pose position;
    MRA::Geometry::Pose velocity;
    double confidence;
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
    MRA::Geometry::Point  origin_pos; // field coordinates of origin
    MRA::Geometry::Point  target_pos; // field coordinates of target
    double ts; // timestamp of update
    double eta; // estimated time of arrival at target
} pass_data_t;


// class with inputs (data for input.proto)
class TeamPlannerInput {
public:
    TeamPlannerInput() {};
    game_state_e gamestate;
    bool ball_present;
    MRA::Geometry::Pose ball;

    std::vector<TeamPlannerRobot> team;
    std::vector<TeamPlannerOpponent> opponents;
    std::vector<MRA::Geometry::Point> parking_positions;
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
    bool teamControlBall;
    bool playerPassedBall;
    bool ballIsObstacle;
    bool searchForBall;
    int playerWhoIsPassing;
    defend_info_t defend_info;

    // internal administration
    std::vector<TeamPlannerRobot> team;
    std::vector<TeamPlannerOpponent> opponents;
};

} // namespace

#endif /* TEAM_PLANNER_DATA_H */