/**
 *  @file
 *  @brief   Data Class for Role assigner
 *  @curator Jürge van Eijck
 */
#ifndef ROLE_ASSIGNER_DATA_HPP
#define ROLE_ASSIGNER_DATA_HPP 1

#include "FieldConfig.hpp"
#include "RobotsportsRobotStrategy.hpp"  // include robot strategy to get list of roles to assign
#include "RoleAssigner_types.hpp"

#include <vector>

#include "RoleAssignerOpponent.hpp"
#include "RoleAssignerParameters.hpp"
#include "RoleAssignerResult.hpp"
#include "RoleAssignerRobot.hpp"

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


class RoleAssignerBall {
public:
    ball_status_e status;
    MRA::Geometry::Position position;
    MRA::Geometry::Position velocity;
    bool is_valid;

    std::string toString(bool full_details) const;
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

// class with state data (data for State.proto)
class RoleAssignerAdminTeam {
public:
    bool assigned = false;
    long robotId = -1;
    RoleAssignerResult result;
    previous_role_assigner_result_t previous_result;
    // compare function to sort vector of the class on the member robotId
    static inline bool CompareRobotId(const RoleAssignerAdminTeam& r1, const RoleAssignerAdminTeam& r2) { return (r1.robotId < r2.robotId);    };
};

// class with state data (data for State.proto)
class RoleAssignerAdminOpponent {
public:
};


// class with inputs (data for input.proto)
class RoleAssignerInput {
public:
    RoleAssignerInput() {};
    game_state_e gamestate;
    RoleAssignerBall ball;

    std::vector<role_e> input_formation;
    std::vector<RoleAssignerRobot> team;
    std::vector<RoleAssignerOpponent> opponents;

    std::vector<MRA::Geometry::Point> parking_positions;
    ball_pickup_position_t ball_pickup_position;
    bool passIsRequired;
    pass_data_t pass_data;
};


// class with state data (data for State.proto)
class RoleAssignerState {
public:
    previous_used_ball_by_role_assinger_t previous_ball;
    std::vector<previous_role_assigner_result_t> previous_result;
};

// class with outputs (data for Output.proto)
class RoleAssignerOutput {
public:
    std::vector<RoleAssignerResult> player_paths;
    std::string pathToString();
};

//-----------------------------------------------------------------------
// Internal administration classes

class RoleAssignerData {
public:
    RoleAssignerData() {};
    /* inputs */
    std::vector<role_e> input_formation;
    game_state_e gamestate;
    RoleAssignerBall ball;
    std::vector<MRA::Geometry::Point> parking_positions;
    ball_pickup_position_t ball_pickup_position;
    bool passIsRequired;
    pass_data_t pass_data;
    MRA::FieldConfig fieldConfig;
    RoleAssignerParameters parameters;


    // based on inputs
    std::vector<role_e> teamFormation;
    bool ballIsObstacle;
    bool searchForBall;
    defend_info_t defend_info;
    previous_used_ball_by_role_assinger_t previous_ball = {};

    // internal administration
    game_state_e original_gamestate;
    std::vector<RoleAssignerRobot> team = {}; // Team will be sorted on robotId inside the role assigner (deterministic order)
    std::vector<RoleAssignerAdminTeam> team_admin = {};
    std::vector<RoleAssignerOpponent> opponents = {};
    std::vector<RoleAssignerAdminOpponent> opponents_admin  = {};
    std::vector<RoleAssignerOpponent> original_opponents  = {};
    int nr_players_assigned = 0;

    unsigned this_player_idx = 0; // index of this robot: Team will be sorted on RobotId.
    unsigned this_player_robotId = 0; // robotId of this robot: Team will be sorted on RobotId.

    int incrementAndGetRank();

    bool teamControlsBall() const;

    std::string toString() const;
};

} // namespace

#endif // ROLE_ASSIGNER_DATA_HPP
