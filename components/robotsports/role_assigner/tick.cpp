// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "RobotsportsRoleAssigner.hpp"

using namespace MRA;

// custom includes, if any
// ...
#include "internals/RoleAssigner.hpp"
#include "internals/RoleAssignerData.hpp"

// helper function for adding some seconds to a google timestamp
google::protobuf::Timestamp timeFromDouble(google::protobuf::Timestamp const &t0, double dt)
{
    google::protobuf::Timestamp result = t0;
    google::protobuf::Duration duration = google::protobuf::util::TimeUtil::NanosecondsToDuration((int64_t)(1e9 * dt));
    return result + duration;
}


int RobotsportsRoleAssigner::RobotsportsRoleAssigner::tick
(
    google::protobuf::Timestamp timestamp,   // absolute timestamp
    InputType  const           &input,       // input data, type generated from Input.proto
    ParamsType const           &params,      // configuration parameters, type generated from Params.proto
    StateType                  &state,       // state data, type generated from State.proto
    OutputType                 &output,      // output data, type generated from Output.proto
    DiagnosticsType            &diagnostics  // diagnostics data, type generated from Diagnostics.proto
)
{
    int error_value = 0;

    RoleAssigner teamplay = RoleAssigner();
    RoleAssignerInput ra_input = {};
    ra_input.gamestate = (MRA::game_state_e) input.gamestate();
    ra_input.ball = {};
    ra_input.ball.is_valid = input.has_ball();
    if (ra_input.ball.is_valid) {
        MRA::Datatypes::BallPossession bp = input.ball().possesion();
        if (bp == MRA::Datatypes::BallPossession::FREE) {
            ra_input.ball.status = ball_status_e::FREE;
        }
        else if (bp == MRA::Datatypes::BallPossession::OWNED_BY_TEAM) {
            ra_input.ball.status = ball_status_e::OWNED_BY_PLAYER;
            // OWNED_BY_TEAM = 2,  // TODO check if undercontrol by a player otherwise owned_by_team
        }
        else if (bp ==  MRA::Datatypes::BallPossession::OWNED_BY_OPPONENT) {
            ra_input.ball.status = ball_status_e::OWNED_BY_OPPONENT;
        }
        ra_input.ball.position = input.ball().position();
        ra_input.ball.velocity = input.ball().velocity();
    }
    ra_input.formation = {};
    for (auto idx = 0; idx <  input.formation_size(); idx++) {
        ra_input.formation.push_back(static_cast<role_e>(input.formation(idx))); // enums have the same values
    }

    for (auto idx = 0; idx <  input.team_size(); idx++) {
        auto input_team =  input.team(idx);
        if (input_team.active()) {
            RoleAssignerRobot rbt = {};
            //rbt = input_team;
            rbt.robotId = input_team.id();
            rbt.active = input_team.active();
            rbt.human = input_team.human();
            rbt.trackingId = input_team.trackingid();
            rbt.controlBall = input_team.hasball();
            rbt.passBall = input_team.passed_ball();
            if (input_team.is_keeper()) {
                rbt.player_type = player_type_e::GOALIE;
            }
            else {
                if (input_team.active()) {
                    rbt.player_type = player_type_e::FIELD_PLAYER;
                }
                else {
                    rbt.player_type = player_type_e::RESERVE;
                }
            }
            rbt.position = input_team.position();
            rbt.velocity = input_team.velocity();
            rbt.time_in_own_penalty_area = input_team.time_in_own_penalty_area();
            rbt.time_in_opponent_penalty_area = input_team.time_in_opponent_penalty_area();
            ra_input.team.push_back(rbt);
        }
    }

    for (auto idx = 0; idx <  input.no_opponent_obstacles_size(); idx++) {
        auto obstacle =  input.no_opponent_obstacles(idx);
        RoleAssignerOpponent no_opponent_obstacle = {};
        no_opponent_obstacle.position = obstacle.position();
        no_opponent_obstacle.velocity = obstacle.velocity();
        no_opponent_obstacle.trackingId = obstacle.trackingid();
        ra_input.no_opponent_obstacles.push_back(no_opponent_obstacle);

    }


    for (auto idx = 0; idx <  input.opponents_size(); idx++) {
        auto input_opponent =  input.opponents(idx);
        RoleAssignerOpponent opponent = {};
        opponent.position = input_opponent.position();
        opponent.velocity = input_opponent.velocity();
        opponent.trackingId = input_opponent.trackingid();
        ra_input.opponents.push_back(opponent);
    }

    for (auto idx = 0; idx <  input.parking_positions_size(); idx++) {
        auto parking_pos = input.parking_positions(idx);
        ra_input.parking_positions.push_back(MRA::Geometry::Point(parking_pos.x(), parking_pos.y()));
    }
    ra_input.ball_pickup_position = {};
    ra_input.ball_pickup_position.valid = input.has_pickup();
    if (ra_input.ball_pickup_position.valid) {
        ra_input.ball_pickup_position.x = input.pickup().position().x();
        ra_input.ball_pickup_position.y  = input.pickup().position().y();
        ra_input.ball_pickup_position.ts = google::protobuf::util::TimeUtil::TimestampToMilliseconds(input.pickup().timestamp()) / 1000.0;
    }


    ra_input.passIsRequired = input.passisrequired();
    ra_input.pass_data = {};
    ra_input.pass_data.valid = input.has_pass_data();
    if (ra_input.pass_data.valid) {
        ra_input.pass_data.kicked = input.pass_data().kicked(); // 1: if pass/shot has been made; 0: otherwise
        ra_input.pass_data.target_id = input.pass_data().target_id();
        ra_input.pass_data.velocity = input.pass_data().velocity();
        ra_input.pass_data.angle = input.pass_data().angle();
        ra_input.pass_data.origin_pos = input.pass_data().origin_pos();
        ra_input.pass_data.target_pos = input.pass_data().target_pos();
        ra_input.pass_data.ts = google::protobuf::util::TimeUtil::TimestampToMilliseconds(input.pass_data().timestamp()) / 1000.0;
        ra_input.pass_data.eta = google::protobuf::util::TimeUtil::TimestampToMilliseconds(input.pass_data().eta()) / 1000.0;
    }
    EnvironmentParameters env_parms = {};
    env_parms.SLM.A = params.environment().model().a();
    env_parms.SLM.B = params.environment().model().b();
    env_parms.SLM.C = params.environment().model().c();
    env_parms.SLM.D = params.environment().model().d();
    env_parms.SLM.E = params.environment().model().e();
    env_parms.SLM.F = params.environment().model().f();
    env_parms.SLM.G = params.environment().model().g();
    env_parms.SLM.H = params.environment().model().h();
    env_parms.SLM.I = params.environment().model().i();
    env_parms.SLM.J = params.environment().model().j();
    env_parms.SLM.K = params.environment().model().k();
    env_parms.SLM.L = params.environment().model().l();
    env_parms.SLM.M = params.environment().model().m();
    env_parms.SLM.N = params.environment().model().n();
    env_parms.SLM.O = params.environment().model().o();
    env_parms.SLM.P = params.environment().model().p();
    env_parms.SLM.Q = params.environment().model().q();
    env_parms.penalty_area_present = params.environment().penalty_area_present();
    env_parms.technical_team_area_present = params.environment().technical_team_area_present();
    env_parms.goal_width = params.environment().goal_width();
    env_parms.goal_length = params.environment().goal_length();
    env_parms.parking_area_width = params.environment().parking_area_width();
    env_parms.parking_area_length = params.environment().parking_area_length();
    env_parms.parking_distance_between_robots = params.environment().parking_distance_between_robots();
    env_parms.parking_distance_to_line = params.environment().parking_distance_to_line();
    env_parms.robot_size = params.environment().robot_size();
    env_parms.ball_radius = params.environment().ball_radius();


    ra_input.environment = Environment(env_parms);

    RoleAssignerState ra_state;
    ra_state.previous_ball.present = state.has_previous_ball();
    if (ra_state.previous_ball.present) {
        ra_state.previous_ball.x = state.previous_ball().x();
        ra_state.previous_ball.y = state.previous_ball().y();
    }

    for (auto idx = 0; idx < state.previous_result_size(); idx++) {
        auto result = state.previous_result(idx);
        previous_role_assigner_result_t prev_res = {};
        prev_res.present = true;
        prev_res.ts = google::protobuf::util::TimeUtil::TimestampToMilliseconds(result.timestamp()) / 1000.0;
        prev_res.end_position.x = result.end_position().x();
        prev_res.end_position.y = result.end_position().y();
        prev_res.role = (MRA::role_e) result.role();
        ra_state.previous_result.push_back(prev_res);
    }

    RoleAssignerParameters ra_parameters = {}; //    parameters;
    ra_parameters.calculateAllPaths = params.calculateallpaths();
    ra_parameters.minimumEdgeLength = params.minimumedgelength();
    ra_parameters.maximumEdgeLength = params.maximumedgelength();
    ra_parameters.minimumDistanceToEndPoint = params.minimumdistancetoendpoint();
    ra_parameters.nrVerticesFirstCircle = params.nrverticesfirstcircle();
    ra_parameters.firstCircleRadius = params.firstcircleradius();
    ra_parameters.nrVerticesSecondCircle  = params.nrverticessecondcircle();
    ra_parameters.secondCircleRadius = params.secondcircleradius();
    ra_parameters.safetyFactor = params.safetyfactor();
    ra_parameters.addBarierVertices = params.addbariervertices();
    ra_parameters.addUniformVertices  = params.adduniformvertices();
    ra_parameters.uniform_x_interval  = params.uniform_x_interval();
    ra_parameters.uniform_y_interval  = params.uniform_y_interval();
    ra_parameters.startingVelocityPenaltyFactor  = params.startingvelocitypenaltyfactor();
    ra_parameters.distToapplyBallApproachVertices  = params.disttoapplyballapproachvertices();
    ra_parameters.addBallApproachVertices = params.addballapproachvertices();
    ra_parameters.ballApproachVerticesRadius = params.ballapproachverticesradius();
    ra_parameters.ballApproachNumberOfVertices  = params.ballapproachnumberofvertices();
    ra_parameters.manDefenseBetweenBallAndPlayer  = params.mandefensebetweenballandplayer();
    ra_parameters.dist_before_penalty_area_for_sweeper  = params.dist_before_penalty_area_for_sweeper();
    ra_parameters.grid_size = params.grid_size();
    ra_parameters.nrDynamicPlannerIterations = params.nrdynamicplanneriterations();
    ra_parameters.maxPossibleLinearSpeed  = params.maxpossiblelinearspeed();
    ra_parameters.maxPossibleLinearAcceleration  = params.maxpossiblelinearacceleration();
    ra_parameters.interceptionChanceStartDistance  = params.interceptionchancestartdistance();
    ra_parameters.interceptionChanceIncreasePerMeter  = params.interceptionchanceincreasepermeter();
    ra_parameters.interceptionChancePenaltyFactor  = params.interceptionchancepenaltyfactor();
    ra_parameters.grid_close_to_ball_normal_penalty  = params.grid_close_to_ball_normal_penalty();
    ra_parameters.grid_close_to_ball_normal_radius  = params.grid_close_to_ball_normal_radius();
    ra_parameters.grid_close_to_ball_restart_normal_penalty  = params.grid_close_to_ball_restart_normal_penalty();
    ra_parameters.grid_close_to_ball_restart_normal_radius  = params.grid_close_to_ball_restart_normal_radius();
    ra_parameters.grid_close_to_ball_restart_penalty_penalty  = params.grid_close_to_ball_restart_penalty_penalty();
    ra_parameters.grid_close_to_ball_restart_penalty_radius  = params.grid_close_to_ball_restart_penalty_radius();
    ra_parameters.grid_close_to_ball_restart_dropball_penalty  = params.grid_close_to_ball_restart_dropball_penalty();
    ra_parameters.grid_close_to_ball_restart_dropball_radius  = params.grid_close_to_ball_restart_dropball_radius();
    ra_parameters.grid_opponent_goal_clearance_x  = params.grid_opponent_goal_clearance_x();
    ra_parameters.grid_opponent_goal_clearance_y  = params.grid_opponent_goal_clearance_y();
    ra_parameters.grid_own_goal_clearance_x  = params.grid_own_goal_clearance_x();
    ra_parameters.grid_own_goal_clearance_y  = params.grid_own_goal_clearance_y();
    ra_parameters.wait_on_non_optimal_position_during_prepare_phase  = params.wait_on_non_optimal_position_during_prepare_phase();
    ra_parameters.priority_block_min_distance  = params.priority_block_min_distance();
    ra_parameters.priority_block_max_distance  = params.priority_block_max_distance();
    ra_parameters.priority_block_max_distance_to_defense_line  = params.priority_block_max_distance_to_defense_line();
    ra_parameters.attack_supporter_extra_distance_to_stay_from_sideline  = params.attack_supporter_extra_distance_to_stay_from_sideline();
    ra_parameters.restart_receiver_ball_dist  = params.restart_receiver_ball_dist();
    ra_parameters.restart_shooter_ball_dist  = params.restart_shooter_ball_dist();
    ra_parameters.equality_cost_threshold  = params.equality_cost_threshold();
    ra_parameters.previous_role_bonus_must_be_applied  = params.previous_role_bonus_must_be_applied();
    ra_parameters.previous_role_end_pos_threshold  = params.previous_role_end_pos_threshold();
    ra_parameters.previous_role_bonus_end_pos_radius  = params.previous_role_bonus_end_pos_radius();
    ra_parameters.use_pass_to_position_for_attack_support  = params.use_pass_to_position_for_attack_support();
    ra_parameters.man_to_man_defense_during_normal_play = params.man_to_man_defense_during_normal_play();
    ra_parameters.man_to_man_defense_during_setplay_against  = params.man_to_man_defense_during_setplay_against();
    ra_parameters.dist_to_goal_to_mark_opponent_as_goalie  = params.dist_to_goal_to_mark_opponent_as_goalie();
    ra_parameters.setplay_against_dist_to_opponent  = params.setplay_against_dist_to_opponent();
    ra_parameters.move_to_ball_left_field_position  = params.move_to_ball_left_field_position();


    ra_parameters.auto_save_svg_period = params.auto_save_svg_period(); // -1 no save, otherwise interval for auto save svg

    // write output to svg file with the name, if empty ("") then no file is written.
//    static std::string svgOutputFileName;
//
//    static std::string svgDefaultTargetColor;
//    static std::string svgBallColor;
//    static std::string svgOriginalTargetColor;
//
//    static std::string svgTeamColor;
//    static std::string svgOpponentColor;
//    ra_parameters.svgDrawVelocity; // draw velocity vectors
//
//    ra_parameters.svgDrawEdges; // draw edges
//
//    /**
//     * if true, grid data is saved to file, for debugging purposes.
//     * file-names will be constructed from the svgOutputFileName or using a generic name if that is not available.
//     */
//    ra_parameters.saveGridDataToFile;
//
//    /**
//     * if true, robot planner will write its output to a svg file, for debugging purposes.
//     * file-names will be constructed from the svgOutputFileName or using a generic name if that is not available.
//     */
//    ra_parameters.svgRobotPlanner;
//
//    ra_parameters.preferredSetplayKicker;
//    ra_parameters.preferredSetplayReceiver;
//    ra_parameters.setplay_margin_to_penalty_area_side;  // min distance for setplay receiver to side of penalty area
//    ra_parameters.interceptor_assign_use_ball_velocity; // use ball velocity to determine the interceptor
//    ra_parameters.interceptor_assign_min_velocity_for_calculate_interception_position; // minimum ball velocity needed to calculate interception position for interceptor role assignment
//    ra_parameters.autoAssignGoalie;
//    ra_parameters.min_y_for_lob_shot;
//    ra_parameters.outsideFieldMargin;
//
//    ra_parameters.kickoff_fp1_x;
//    ra_parameters.kickoff_fp1_y;
//    ra_parameters.kickoff_fp2_x;
//    ra_parameters.kickoff_fp2_y;
//    ra_parameters.kickoff_fp3_x;
//    ra_parameters.kickoff_fp3_y;
//    ra_parameters.kickoff_fp4_x;
//    ra_parameters.kickoff_fp4_y;
//
//    ra_parameters.kickoff_against_fp1_x;
//    ra_parameters.kickoff_against_fp1_y;
//    ra_parameters.kickoff_against_fp2_x;
//    ra_parameters.kickoff_against_fp2_y;
//    ra_parameters.kickoff_against_fp3_x;
//    ra_parameters.kickoff_against_fp3_y;
//    ra_parameters.kickoff_against_fp4_x;
//    ra_parameters.kickoff_against_fp4_y;


    RoleAssignerOutput ra_output = {};
    auto ra_state_org = ra_state;
    teamplay.assign(ra_input, ra_state, ra_output, ra_parameters);

//    RoleAssignerData tpd = {};
//    tpd.parameters = tp_parameters;
//    tpd.environment = tp_input.environment;
//    tpd.input_formation = tp_input.input_formation;
//    tpd.gamestate = tp_input.gamestate;
//    tpd.original_gamestate  = tp_input.gamestate;
//    tpd.ball = tp_input.ball;
//    tpd.parking_positions = tp_input.parking_positions;
//    tpd.ball_pickup_position = tp_input.ball_pickup_position;
//    tpd.passIsRequired = tp_input.passIsRequired;
//    tpd.pass_data = tp_input.pass_data;
//    tpd.previous_ball = tp_state.previous_ball;
//    tpd.team = tp_input.team;
//    tpd.opponents = tp_input.opponents;
//
//    // inputs
//    tpd.previous_ball = tp_state_org.previous_ball;
//    for (auto idx = 0u; idx < tp_state_org.previous_result.size(); ++idx) {
//        RoleAssignerAdminTeam tp_admin = {};
//        tp_admin.previous_result = tp_state_org.previous_result[idx];
//        tpd.team_admin.push_back(tp_admin);
//    }
//
//    for (auto idx = 0u; idx < tp_output.player_paths.size(); ++idx) {
//        tpd.team_admin[idx].robotId = tp_input.team[idx].robotId;
//        tpd.team_admin[idx].assigned = true;
//        tpd.team_admin[idx].result = tp_output.player_paths[idx]; //(RoleAssignerData.team_admin[idx].result);
//    }
//    player_paths = tp_output.player_paths;
//
//    RunData run_data (tpd, player_paths);
//    run_results.push_back(run_data);
//
//    if (player_paths.size() > 0) {
//        if (not print_only_errors) {
//            cerr << "<< XML: print received path " << endl << flush;
//            cerr << RoleAssignerResultToString(player_paths, tpd.team) << endl << flush;
//        }
//        RoleAssignerSvg::role_assigner_data_to_svg(player_paths, tpd, environment, run_filename);
//
//    } else {
//        cerr << "<< XML: no path received" << endl << flush;
//    }
//    if (not print_only_errors) {
//        cerr << "<< Assign roles" << endl << flush;
//    }


    return error_value;
}

