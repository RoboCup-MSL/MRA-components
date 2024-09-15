/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */
#ifndef TEAM_PLANNER_DATA_HPP
#define TEAM_PLANNER_DATA_HPP 1

#include "FieldConfig.hpp"
#include "TeamPlannerOpponent.hpp"
#include "TeamPlannerParameters.hpp"
#include "TeamPlannerResult.hpp"
#include "TeamPlannerRobot.hpp"
#include "RobotsportsRobotStrategy.hpp"  // include robot strategy to get list of roles to assign

#include <vector>

namespace MRA {


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


class TeamPlannerBall {
public:
    ball_status_e status;
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
};


//-----------------------------------------------------------------------
// Internal administration classes


class TeamPlannerData {
public:
    TeamPlannerData() {};
    /* inputs */
    std::vector<MRA::RobotsportsRobotStrategy::Output_DynamicRole> input_formation;
    game_state_e gamestate;
    TeamPlannerBall ball;
    std::vector<MRA::Geometry::Point> parking_positions;
    ball_pickup_position_t ball_pickup_position;
    bool passIsRequired;
    pass_data_t pass_data;
    std::vector<dynamic_role_e> teamFormation;
    MRA::FieldConfig fieldConfig;
    TeamPlannerParameters parameters;

    // based on inputs
    bool ballIsObstacle;
    bool searchForBall;
    defend_info_t defend_info;
    previous_used_ball_by_planner_t previous_ball = {};

    // internal administration
    game_state_e original_gamestate;
    std::vector<TeamPlannerRobot> team = {}; // Team will be sorted on robotId inside the role assigner (deterministic order)
    std::vector<TeamPlannerOpponent> opponents = {};
    std::vector<TeamPlannerOpponent> original_opponents  = {};
    int nr_players_assigned = 0;

    unsigned this_player_idx = 0; // index of this robot: Team will be sorted on RobotId.
    unsigned this_player_robotId = 0; // robotId of this robot: Team will be sorted on RobotId.

    int incrementAndGetRank();

    bool teamControlsBall() const;
};

} // namespace

#endif // TEAM_PLANNER_DATA_HPP
