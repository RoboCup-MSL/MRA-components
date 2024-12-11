/**
 *  @file
 *  @brief   Utility for handling xml input to test team planner
 *  @curator Jürge van Eijck
 */

#include <ostream>
#include <iostream>
#include <iomanip>
#include <istream>
#include <vector>
#include <string>
#include <cstdio>
#include <chrono>
#include <unistd.h>
#include <cmath>
#include <cstddef>
#include <filesystem>

#include "../internals/RoleAssigner.hpp"
#include "../internals/RoleAssigner_types.hpp"
#include "../internals/RoleAssignerResult.hpp"
#include "RobotsportsRobotStrategy.hpp"  // include robot strategy to get list of roles to assign
#include "RobotsportsRoleAssigner.hpp"


#include "StrategyTester_generated.h" // generated
#include "GlobalPathPlanner.hpp" // for print path
#include "xmlRoleAssigner.hpp"

#include "../internals/Environment.hpp"
#include "../internals/RoleAssignerSvg.hpp"
#include "../internals/RoleAssignerData.hpp"

using namespace MRA;
using namespace std;
using namespace robotsports;


static void xml_assign_roles(const RoleAssignerInput& ra_input,
                      RoleAssignerState& ra_state,
                      RoleAssignerOutput& ra_output,
                      const RoleAssignerParameters& ra_parameters) {

    bool useProto = true;
    std::cout << "USING PROTO: " << useProto << std::endl;

    if (useProto) {
        auto m = RobotsportsRoleAssigner::RobotsportsRoleAssigner();

        google::protobuf::Timestamp timestamp;   // absolute timestamp
        RobotsportsRoleAssigner::InputType proto_input;
        RobotsportsRoleAssigner::ParamsType proto_params;      // configuration parameters, type generated from Params.proto
        RobotsportsRoleAssigner::StateType  proto_state;       // state data, type generated from State.proto
        RobotsportsRoleAssigner::OutputType proto_output;      // output data, type generated from Output.proto
        RobotsportsRoleAssigner::DiagnosticsType proto_diagnostics;  // diagnostics data, type generated from Diagnostics.proto

        proto_input.set_gamestate(static_cast<MRA::RobotsportsRoleAssigner::Input_GameState>(ra_input.gamestate));
    
        if (ra_input.ball.is_valid) {
            proto_input.mutable_ball()->mutable_position()->set_x(ra_input.ball.position.x);
            proto_input.mutable_ball()->mutable_position()->set_y(ra_input.ball.position.y);
            proto_input.mutable_ball()->mutable_position()->set_z(ra_input.ball.position.z);
            proto_input.mutable_ball()->mutable_position()->set_rx(ra_input.ball.position.rx);
            proto_input.mutable_ball()->mutable_position()->set_ry(ra_input.ball.position.ry);
            proto_input.mutable_ball()->mutable_position()->set_rz(ra_input.ball.position.rz);
            proto_input.mutable_ball()->mutable_velocity()->set_x(ra_input.ball.velocity.x);
            proto_input.mutable_ball()->mutable_velocity()->set_y(ra_input.ball.velocity.y);
            proto_input.mutable_ball()->mutable_velocity()->set_z(ra_input.ball.velocity.z);
            proto_input.mutable_ball()->mutable_velocity()->set_rx(ra_input.ball.velocity.rx);
            proto_input.mutable_ball()->mutable_velocity()->set_ry(ra_input.ball.velocity.ry);
            proto_input.mutable_ball()->mutable_velocity()->set_rz(ra_input.ball.velocity.rz);
            if (ra_input.ball.status == ball_status_e::FREE) {
                proto_input.mutable_ball()->set_possesion(MRA::Datatypes::BallPossession::FREE);
            }
            else if (ra_input.ball.status == ball_status_e::OWNED_BY_PLAYER) {
                proto_input.mutable_ball()->set_possesion(MRA::Datatypes::BallPossession::OWNED_BY_TEAM);
            }
            else {
                proto_input.mutable_ball()->set_possesion(MRA::Datatypes::BallPossession::OWNED_BY_OPPONENT);
            }
        }
        proto_input.clear_formation();
        for (auto idx = 0u; idx < ra_input.formation.size(); idx++) {
        	// enums have the same values
        	proto_input.add_formation(static_cast<MRA::RobotsportsRoleAssigner::DynamicRole>(ra_input.formation[idx]));
        }

        proto_input.mutable_team()->Clear();
        for (auto idx = 0u; idx <  ra_input.team.size(); idx++) {
            if (ra_input.team[idx].active) {
                auto input_team =  ra_input.team[idx];
                auto player = MRA::RobotsportsRoleAssigner::Player();
                player.set_id(input_team.robotId);  // T-shirt number
                player.set_active(input_team.active);
                player.set_human(input_team.human);
                player.set_trackingid(input_team.trackingId );
                player.set_hasball(input_team.controlBall);
                player.set_passed_ball(input_team.passBall);
                player.set_is_keeper(input_team.player_type == player_type_e::GOALIE);
                player.set_time_in_own_penalty_area(input_team.time_in_own_penalty_area);
                player.set_time_in_opponent_penalty_area(input_team.time_in_opponent_penalty_area);
                player.set_trackingid(input_team.trackingId);
                player.mutable_position()->set_x(input_team.position.x);
                player.mutable_position()->set_y(input_team.position.y);
                player.mutable_position()->set_z(input_team.position.z);
                player.mutable_position()->set_rx(input_team.position.rx);
                player.mutable_position()->set_ry(input_team.position.ry);
                player.mutable_position()->set_rz(input_team.position.rz);
                player.mutable_velocity()->set_x(input_team.velocity.x);
                player.mutable_velocity()->set_y(input_team.velocity.y);
                player.mutable_velocity()->set_z(input_team.velocity.z);
                player.mutable_velocity()->set_rx(input_team.velocity.rx);
                player.mutable_velocity()->set_ry(input_team.velocity.ry);
                player.mutable_velocity()->set_rz(input_team.velocity.rz);
                proto_input.mutable_team()->Add()->CopyFrom(player);
            }
        }

        for (auto idx = 0u; idx < ra_input.opponents.size(); idx++) {
        	auto input_opponent = ra_input.opponents[idx];
            auto opponent = MRA::RobotsportsRoleAssigner::Opponent();
        	opponent.mutable_position()->set_x(input_opponent.position.x);
            opponent.mutable_position()->set_y(input_opponent.position.y);
            opponent.mutable_position()->set_z(input_opponent.position.z);
            opponent.mutable_position()->set_rx(input_opponent.position.rx);
            opponent.mutable_position()->set_ry(input_opponent.position.ry);
            opponent.mutable_position()->set_rz(input_opponent.position.rz);
            opponent.mutable_velocity()->set_x(input_opponent.velocity.x);
            opponent.mutable_velocity()->set_y(input_opponent.velocity.y);
            opponent.mutable_velocity()->set_z(input_opponent.velocity.z);
            opponent.mutable_velocity()->set_rx(input_opponent.velocity.rx);
            opponent.mutable_velocity()->set_ry(input_opponent.velocity.ry);
            opponent.mutable_velocity()->set_rz(input_opponent.velocity.rz);
        	opponent.set_trackingid(input_opponent.trackingId);
        	proto_input.mutable_opponents()->Add()->CopyFrom(opponent);
        }

        for (auto idx = 0u; idx < ra_input.no_opponent_obstacles.size(); idx++) {
        	auto input_obstacle = ra_input.no_opponent_obstacles[idx];
            auto obstacle = MRA::RobotsportsRoleAssigner::Opponent();
            obstacle.mutable_position()->set_x(input_obstacle.position.x);
        	obstacle.mutable_position()->set_y(input_obstacle.position.y);
        	obstacle.mutable_position()->set_z(input_obstacle.position.z);
        	obstacle.mutable_position()->set_rx(input_obstacle.position.rx);
        	obstacle.mutable_position()->set_ry(input_obstacle.position.ry);
        	obstacle.mutable_position()->set_rz(input_obstacle.position.rz);
        	obstacle.mutable_velocity()->set_x(input_obstacle.velocity.x);
        	obstacle.mutable_velocity()->set_y(input_obstacle.velocity.y);
        	obstacle.mutable_velocity()->set_z(input_obstacle.velocity.z);
        	obstacle.mutable_velocity()->set_rx(input_obstacle.velocity.rx);
        	obstacle.mutable_velocity()->set_ry(input_obstacle.velocity.ry);
        	obstacle.mutable_velocity()->set_rz(input_obstacle.velocity.rz);
        	obstacle.set_trackingid(input_obstacle.trackingId);
        	proto_input.mutable_no_opponent_obstacles()->Add()->CopyFrom(obstacle);
        }

        if (ra_input.ball_pickup_position.valid) {
            google::protobuf::Timestamp timestamp = google::protobuf::util::TimeUtil::MillisecondsToTimestamp(ra_input.ball_pickup_position.ts * 1000);
            proto_input.mutable_pickup()->mutable_timestamp()->CopyFrom(timestamp);
            proto_input.mutable_pickup()->mutable_position()->set_x(ra_input.ball_pickup_position.x);
            proto_input.mutable_pickup()->mutable_position()->set_y(ra_input.ball_pickup_position.y);
        }

        proto_input.set_passisrequired(ra_input.passIsRequired);

        for (auto idx = 0u; idx < ra_input.parking_positions.size(); idx++) {
        	auto park_pose = MRA::Datatypes::Pose();
        	park_pose.set_x(ra_input.parking_positions[idx].x);
        	park_pose.set_y(ra_input.parking_positions[idx].y);
        	proto_input.mutable_parking_positions()->Add()->CopyFrom(park_pose);
        }
        if (ra_input.pass_data.valid) {
        	auto pass_data = MRA::RobotsportsRoleAssigner::PassData();
        	pass_data.set_angle(ra_input.pass_data.angle);
        	pass_data.set_target_id(ra_input.pass_data.target_id);
        	pass_data.set_kicked(ra_input.pass_data.kicked);
        	pass_data.set_valid(ra_input.pass_data.valid);
        	pass_data.set_velocity(ra_input.pass_data.velocity);
        	pass_data.mutable_origin_pos()->set_x(ra_input.pass_data.origin_pos.x);
        	pass_data.mutable_origin_pos()->set_y(ra_input.pass_data.origin_pos.y);
        	pass_data.mutable_target_pos()->set_x(ra_input.pass_data.target_pos.x);
        	pass_data.mutable_target_pos()->set_y(ra_input.pass_data.target_pos.y);
            google::protobuf::Timestamp timestamp = google::protobuf::util::TimeUtil::MillisecondsToTimestamp(ra_input.pass_data.ts * 1000);
        	pass_data.mutable_timestamp()->CopyFrom(timestamp);
            google::protobuf::Timestamp eta = google::protobuf::util::TimeUtil::MillisecondsToTimestamp(ra_input.pass_data.eta * 1000);
			pass_data.mutable_eta()->CopyFrom(eta);
        	proto_input.mutable_pass_data()->CopyFrom(pass_data);
        }

        auto proto_env_params =  MRA::RobotsportsRoleAssigner::Environment_Parameters();
        EnvironmentParameters env_params = {};
        ra_input.environment.getEnvironmentParameters(env_params);
        proto_env_params.mutable_model()->set_a(env_params.SLM.A);
        proto_env_params.mutable_model()->set_b(env_params.SLM.B);
        proto_env_params.mutable_model()->set_c(env_params.SLM.C);
        proto_env_params.mutable_model()->set_d(env_params.SLM.D);
        proto_env_params.mutable_model()->set_e(env_params.SLM.E);
        proto_env_params.mutable_model()->set_f(env_params.SLM.F);
        proto_env_params.mutable_model()->set_g(env_params.SLM.G);
        proto_env_params.mutable_model()->set_h(env_params.SLM.H);
        proto_env_params.mutable_model()->set_i(env_params.SLM.I);
        proto_env_params.mutable_model()->set_j(env_params.SLM.J);
        proto_env_params.mutable_model()->set_k(env_params.SLM.K);
        proto_env_params.mutable_model()->set_l(env_params.SLM.L);
        proto_env_params.mutable_model()->set_m(env_params.SLM.M);
        proto_env_params.mutable_model()->set_n(env_params.SLM.N);
        proto_env_params.mutable_model()->set_o(env_params.SLM.O);
        proto_env_params.mutable_model()->set_p(env_params.SLM.P);
        proto_env_params.mutable_model()->set_p(env_params.SLM.Q);
        proto_env_params.set_ball_radius(env_params.ball_radius);
        proto_env_params.set_goal_length(env_params.goal_length);
        proto_env_params.set_goal_width(env_params.goal_width);
        proto_env_params.set_parking_area_length(env_params.parking_area_length);
        proto_env_params.set_parking_area_width(env_params.parking_area_width);
        proto_env_params.set_parking_distance_between_robots(env_params.parking_distance_between_robots);
        proto_env_params.set_parking_distance_to_line(env_params.parking_distance_to_line);
        proto_env_params.set_penalty_area_present(env_params.penalty_area_present);
        proto_env_params.set_robot_size(env_params.robot_size);
        proto_env_params.set_technical_team_area_present(env_params.technical_team_area_present);

        proto_params.mutable_environment()->CopyFrom(proto_env_params);

        proto_params.set_calculateallpaths(ra_parameters.calculateAllPaths);
        proto_params.set_minimumedgelength(ra_parameters.minimumEdgeLength);
        proto_params.set_maximumedgelength(ra_parameters.maximumEdgeLength);
        proto_params.set_minimumdistancetoendpoint(ra_parameters.minimumDistanceToEndPoint);
        proto_params.set_nrverticesfirstcircle(ra_parameters.nrVerticesFirstCircle);
        proto_params.set_firstcircleradius(ra_parameters.firstCircleRadius);
        proto_params.set_nrverticessecondcircle(ra_parameters.nrVerticesSecondCircle);
        proto_params.set_secondcircleradius(ra_parameters.secondCircleRadius);
        proto_params.set_safetyfactor(ra_parameters.safetyFactor);
        proto_params.set_addbariervertices(ra_parameters.addBarierVertices);
        proto_params.set_adduniformvertices(ra_parameters.addUniformVertices);
        proto_params.set_uniform_x_interval(ra_parameters.uniform_x_interval);
        proto_params.set_uniform_y_interval(ra_parameters.uniform_y_interval);
        proto_params.set_startingvelocitypenaltyfactor(ra_parameters.startingVelocityPenaltyFactor);
        proto_params.set_disttoapplyballapproachvertices(ra_parameters.distToapplyBallApproachVertices);
        proto_params.set_addballapproachvertices(ra_parameters.addBallApproachVertices);
        proto_params.set_ballapproachverticesradius(ra_parameters.ballApproachVerticesRadius);
        proto_params.set_ballapproachnumberofvertices(ra_parameters.ballApproachNumberOfVertices);
        proto_params.set_mandefensebetweenballandplayer(ra_parameters.manDefenseBetweenBallAndPlayer);
        proto_params.set_dist_before_penalty_area_for_sweeper(ra_parameters.dist_before_penalty_area_for_sweeper);
        proto_params.set_grid_size(ra_parameters.grid_size);
        proto_params.set_nrdynamicplanneriterations(ra_parameters.nrDynamicPlannerIterations);
        proto_params.set_maxpossiblelinearspeed(ra_parameters.maxPossibleLinearSpeed);
        proto_params.set_maxpossiblelinearacceleration(ra_parameters.maxPossibleLinearAcceleration);
        proto_params.set_interceptionchancestartdistance(ra_parameters.interceptionChanceStartDistance);
        proto_params.set_interceptionchanceincreasepermeter(ra_parameters.interceptionChanceIncreasePerMeter);
        proto_params.set_interceptionchancepenaltyfactor(ra_parameters.interceptionChancePenaltyFactor);
        proto_params.set_grid_close_to_ball_normal_penalty(ra_parameters.grid_close_to_ball_normal_penalty);
        proto_params.set_grid_close_to_ball_normal_radius(ra_parameters.grid_close_to_ball_normal_radius);
        proto_params.set_grid_close_to_ball_restart_normal_penalty(ra_parameters.grid_close_to_ball_restart_normal_penalty);
        proto_params.set_grid_close_to_ball_restart_normal_radius(ra_parameters.grid_close_to_ball_restart_normal_radius);
        proto_params.set_grid_close_to_ball_restart_penalty_penalty(ra_parameters.grid_close_to_ball_restart_penalty_penalty);
        proto_params.set_grid_close_to_ball_restart_penalty_radius(ra_parameters.grid_close_to_ball_restart_penalty_radius);
        proto_params.set_grid_close_to_ball_restart_dropball_penalty(ra_parameters.grid_close_to_ball_restart_dropball_penalty);
        proto_params.set_grid_close_to_ball_restart_dropball_radius(ra_parameters.grid_close_to_ball_restart_dropball_radius);
        proto_params.set_grid_opponent_goal_clearance_x(ra_parameters.grid_opponent_goal_clearance_x);
        proto_params.set_grid_opponent_goal_clearance_y(ra_parameters.grid_opponent_goal_clearance_y);
        proto_params.set_grid_own_goal_clearance_x(ra_parameters.grid_own_goal_clearance_x);
        proto_params.set_grid_own_goal_clearance_y(ra_parameters.grid_own_goal_clearance_y);
        proto_params.set_wait_on_non_optimal_position_during_prepare_phase(ra_parameters.wait_on_non_optimal_position_during_prepare_phase);
        proto_params.set_priority_block_min_distance(ra_parameters.priority_block_min_distance);
        proto_params.set_priority_block_max_distance(ra_parameters.priority_block_max_distance);
        proto_params.set_priority_block_max_distance_to_defense_line(ra_parameters.priority_block_max_distance_to_defense_line);
        proto_params.set_attack_supporter_extra_distance_to_stay_from_sideline(ra_parameters.attack_supporter_extra_distance_to_stay_from_sideline);
        proto_params.set_restart_receiver_ball_dist(ra_parameters.restart_receiver_ball_dist);
        proto_params.set_restart_shooter_ball_dist(ra_parameters.restart_shooter_ball_dist);
        proto_params.set_equality_cost_threshold(ra_parameters.equality_cost_threshold);
        proto_params.set_previous_role_bonus_must_be_applied(ra_parameters.previous_role_bonus_must_be_applied);
        proto_params.set_previous_role_end_pos_threshold(ra_parameters.previous_role_end_pos_threshold);
        proto_params.set_previous_role_bonus_end_pos_radius(ra_parameters.previous_role_bonus_end_pos_radius);
        proto_params.set_use_pass_to_position_for_attack_support(ra_parameters.use_pass_to_position_for_attack_support);
        proto_params.set_man_to_man_defense_during_normal_play(ra_parameters.man_to_man_defense_during_normal_play);
        proto_params.set_man_to_man_defense_during_setplay_against(ra_parameters.man_to_man_defense_during_setplay_against);
        proto_params.set_dist_to_goal_to_mark_opponent_as_goalie(ra_parameters.dist_to_goal_to_mark_opponent_as_goalie);
        proto_params.set_setplay_against_dist_to_opponent(ra_parameters.setplay_against_dist_to_opponent);
        proto_params.set_move_to_ball_left_field_position(ra_parameters.move_to_ball_left_field_position);
        proto_params.set_auto_save_svg_period(ra_parameters.auto_save_svg_period);
        proto_params.set_svgoutputfilename(ra_parameters.svgOutputFileName);
		proto_params.set_svgdefaulttargetcolor(ra_parameters.svgDefaultTargetColor);
		proto_params.set_svgballcolor(ra_parameters.svgBallColor);
		proto_params.set_svgoriginaltargetcolor(ra_parameters.svgOriginalTargetColor);
		proto_params.set_svgteamcolor(ra_parameters.svgTeamColor);
		proto_params.set_svgopponentcolor(ra_parameters.svgOpponentColor);
		proto_params.set_svgdrawvelocity(ra_parameters.svgDrawVelocity);
		proto_params.set_svgdrawedges(ra_parameters.svgDrawEdges);
		proto_params.set_savegriddatatofile(ra_parameters.saveGridDataToFile);
		proto_params.set_svgrobotplanner(ra_parameters.svgRobotPlanner);
		proto_params.set_preferredsetplaykicker(ra_parameters.preferredSetplayKicker);
		proto_params.set_preferredsetplayreceiver(ra_parameters.preferredSetplayReceiver);
		proto_params.set_setplay_margin_to_penalty_area_side(ra_parameters.setplay_margin_to_penalty_area_side);
		proto_params.set_interceptor_assign_use_ball_velocity(ra_parameters.interceptor_assign_use_ball_velocity);
		proto_params.set_interceptor_assign_min_velocity_for_calculate_interception_position(ra_parameters.interceptor_assign_min_velocity_for_calculate_interception_position);
		proto_params.set_autoassigngoalie(ra_parameters.autoAssignGoalie);
		proto_params.set_min_y_for_lob_shot(ra_parameters.min_y_for_lob_shot);
		proto_params.set_outsidefieldmargin(ra_parameters.outsideFieldMargin);
		proto_params.set_kickoff_fp1_x(ra_parameters.kickoff_fp1_x);
		proto_params.set_kickoff_fp1_y(ra_parameters.kickoff_fp1_y);
		proto_params.set_kickoff_fp2_x(ra_parameters.kickoff_fp2_x);
		proto_params.set_kickoff_fp2_y(ra_parameters.kickoff_fp2_y);
		proto_params.set_kickoff_fp3_x(ra_parameters.kickoff_fp3_x);
		proto_params.set_kickoff_fp3_y(ra_parameters.kickoff_fp3_y);
		proto_params.set_kickoff_fp4_x(ra_parameters.kickoff_fp4_x);
		proto_params.set_kickoff_fp4_y(ra_parameters.kickoff_fp4_y);
		proto_params.set_kickoff_against_fp1_x(ra_parameters.kickoff_against_fp1_x);
		proto_params.set_kickoff_against_fp1_y(ra_parameters.kickoff_against_fp1_y);
		proto_params.set_kickoff_against_fp2_x(ra_parameters.kickoff_against_fp2_x);
		proto_params.set_kickoff_against_fp2_y(ra_parameters.kickoff_against_fp2_y);
		proto_params.set_kickoff_against_fp3_x(ra_parameters.kickoff_against_fp3_x);
		proto_params.set_kickoff_against_fp3_y(ra_parameters.kickoff_against_fp3_y);
		proto_params.set_kickoff_against_fp4_x(ra_parameters.kickoff_against_fp4_x);
		proto_params.set_kickoff_against_fp4_y(ra_parameters.kickoff_against_fp4_y);

		if (ra_state.previous_ball.present) {
			proto_state.mutable_previous_ball()->set_x(ra_state.previous_ball.x);
			proto_state.mutable_previous_ball()->set_y(ra_state.previous_ball.y);
		}
		for (auto prev_idx = 0u; prev_idx < ra_state.previous_results.size(); prev_idx++) {
            auto proto_prev_result = MRA::RobotsportsRoleAssigner::PreviousResult();
            auto prev_res = ra_state.previous_results[prev_idx];
            google::protobuf::Timestamp timestamp = google::protobuf::util::TimeUtil::MillisecondsToTimestamp(prev_res.ts * 1000);
            proto_prev_result.mutable_timestamp()->CopyFrom(timestamp);
            proto_prev_result.mutable_end_position()->set_x(prev_res.end_position.x);
            proto_prev_result.mutable_end_position()->set_y(prev_res.end_position.y);
            proto_prev_result.mutable_end_position()->set_cost(prev_res.end_position.cost);
            proto_prev_result.mutable_end_position()->set_target(
            proto_prev_result.set_role(static_cast<MRA::RobotsportsRoleAssigner::DynamicRole>(prev_res.role));
                static_cast<MRA::RobotsportsRoleAssigner::PathPurpose>(prev_res.end_position.target));
            proto_state.mutable_previous_result()->Add()->CopyFrom(proto_prev_result);
        }

        int error_value = m.tick(timestamp, proto_input, proto_params, proto_state, proto_output, proto_diagnostics);
        if (error_value != 0) {
            cout << "RobotsportsRoleAssigner failed" << endl;
            exit(1);
        }

        for (auto idx= 0; idx < proto_output.assignments_size(); idx++) {
        	auto assignement = proto_output.assignments(idx);
        	RoleAssignerResult result = {};
        	result.role = static_cast<MRA::role_e>(assignement.role());
        	result.role_rank = assignement.role_rank();
        	result.gamestate = ra_input.gamestate;
        	result.target = assignement.target();
        	result.planner_target = static_cast<MRA::planner_target_e>(assignement.purpose());
        	result.is_pass_desitination = assignement.is_pass_desitination();
        	result.defend_info = {};
        	result.defend_info.valid = assignement.has_defend_info();
        	if (result.defend_info.valid) {
        		result.defend_info.defending_id = assignement.defend_info().trackingid();
        		result.defend_info.dist_from_defending_id = assignement.defend_info().dist_from_defending_id();
        		result.defend_info.between_ball_and_defending_pos = assignement.defend_info().between_ball_and_defending_pos();
        	}
        	result.path = std::vector<path_piece_t>();
            for (auto path_idx = 0; path_idx < assignement.path_size(); path_idx++) {
                auto proto_path = assignement.path(path_idx);
                auto piece = path_piece_t();
                piece.cost = proto_path.cost();
                piece.target = static_cast<MRA::planner_target_e>(proto_path.target());
                piece.x = proto_path.x();
                piece.y = proto_path.y();
                result.path.push_back(piece);
            }
        	ra_output.player_paths.push_back(result);
        }
    }
    else {
        RoleAssigner teamplay = RoleAssigner();
        teamplay.assign(ra_input, ra_state, ra_output, ra_parameters);
    }
}



class RunData {
public:
    RunData(const RoleAssignerData& r_tpd, const std::vector<RoleAssignerResult>& r_result) :
        RoleAssigner_data(r_tpd),
        player_paths(r_result)
    {

    }
    RoleAssignerData RoleAssigner_data;
    std::vector<RoleAssignerResult> player_paths;
};

#include <string>
#include <sstream>
#include <vector>

std::vector<role_e> getListWithRoles(game_state_e gameState, ball_status_e ball_status,
                                     bool no_defender_main_during_setplay,
                                     team_formation_e attack_formation,
                                     team_formation_e defense_formation ) {

    auto robot_strategy = RobotsportsRobotStrategy::RobotsportsRobotStrategy();
    auto robot_strategy_input = RobotsportsRobotStrategy::Input();
    auto robot_strategy_output = RobotsportsRobotStrategy::Output();
    auto robot_strategy_params = RobotsportsRobotStrategy::Params();
    robot_strategy_params.set_no_defender_main_during_setplay(no_defender_main_during_setplay);
    robot_strategy_params.set_attack_formation((RobotsportsRobotStrategy::Params_TeamFormation)attack_formation);
    robot_strategy_params.set_defense_formation((RobotsportsRobotStrategy::Params_TeamFormation)defense_formation);

    MRA::RobotsportsRobotStrategy::Input_GameState gs = (MRA::RobotsportsRobotStrategy::Input_GameState) (gameState);
    robot_strategy_input.set_game_state(gs);
    robot_strategy_input.set_ball_status((MRA::RobotsportsRobotStrategy::Input_BallStatus) ball_status);

    auto error_value = robot_strategy.tick(robot_strategy_input, robot_strategy_params, robot_strategy_output);
    if (error_value != 0) {
        cout << "robot_strategy failed" << endl;
        exit(1);
    }

    std::vector<role_e> roles_to_assign = {};
    for (auto idx = 0; idx < robot_strategy_output.dynamic_roles_size(); idx++) {
        MRA::Datatypes::DynamicRole odr = robot_strategy_output.dynamic_roles(idx);
        roles_to_assign.push_back(static_cast<role_e>(odr));
    }

    return roles_to_assign;
}



static std::string RoleAssignerResultToString(const RoleAssignerOutput& output, const RoleAssignerInput& input) {
    std::stringstream buffer;
    for (unsigned player_idx = 0; player_idx != output.player_paths.size(); player_idx++) {
        buffer << "path for player  " << player_idx <<  " id: " << output.player_paths[player_idx].robotId <<  " -> " 
               << RoleAsString(output.player_paths[player_idx].role) <<  endl;
        if (output.player_paths[player_idx].defend_info.valid) {
            buffer << " Defend info: valid: true id: "<< output.player_paths[player_idx].defend_info.defending_id;
            buffer << " dist to id: " << output.player_paths[player_idx].defend_info.dist_from_defending_id;
            buffer << " between ball and id: " << output.player_paths[player_idx].defend_info.between_ball_and_defending_pos << endl;
        }
        else {
            buffer << " Defend info: valid: false" << endl;
        }
        std::vector<path_piece_t> path = output.player_paths[player_idx].path;
        for (unsigned int path_idx = 0; path_idx != path.size(); path_idx++) {
            buffer << "path piece [ " << path_idx << "]  = (" << path[path_idx].x << ", "<< path[path_idx].y << ")" << endl;
        }

    }
    return buffer.str();
}

MRA::team_formation_e StringToFormation(const string& formation_string) {
    MRA::team_formation_e formation = team_formation_e::FORMATION_013;
    if (formation_string == "FORMATION_013") {
        formation = team_formation_e::FORMATION_013;
    } else if (formation_string == "FORMATION_112") {
        formation = team_formation_e::FORMATION_112;
    } else if (formation_string == "FORMATION_211") {
        formation = team_formation_e::FORMATION_211;
    } else if (formation_string == "FORMATION_310") {
        formation = team_formation_e::FORMATION_310;
    } else if (formation_string == "FORMATION_ATTACK_SUPPORT_ONLY") {
        formation = team_formation_e::FORMATION_ATTACK_SUPPORT_ONLY;
    } else if (formation_string == "FORMATION_DEFENDER_ONLY") {
        formation = team_formation_e::FORMATION_DEFENDER_ONLY;
    } else if (formation_string == "FORMATION_INTERCEPTOR_ONLY") {
        formation = team_formation_e::FORMATION_INTERCEPTOR_ONLY;
    } else if (formation_string == "FORMATION_SWEEPER_ONLY") {
        formation = team_formation_e::FORMATION_SWEEPER_ONLY;
    } else if (formation_string == "FORMATION_SETPLAY_RECEIVER_ONLY") {
        formation = team_formation_e::FORMATION_SETPLAY_RECEIVER_ONLY;
    } else if (formation_string == "FORMATION_SETPLAY_KICKER_ONLY") {
        formation = team_formation_e::FORMATION_SETPLAY_KICKER_ONLY;
    } else if (formation_string == "FORMATION_BALLPLAYER_ONLY") {
        formation = team_formation_e::FORMATION_BALLPLAYER_ONLY;
    } else if (formation_string == "FORMATION_SEARCHFORBALL_ONLY") {
        formation = team_formation_e::FORMATION_SEARCHFORBALL_ONLY;
    } else if (formation_string == "FORMATION_BEGINPOSITION_ONLY") {
        formation = team_formation_e::FORMATION_BEGINPOSITION_ONLY;
    } else if (formation_string == "FORMATION_PARKING_ONLY") {
        formation = team_formation_e::FORMATION_PARKING_ONLY;
    } else if (formation_string == "FORMATION_PENALTYKICKER_ONLY") {
        formation = team_formation_e::FORMATION_PENALTYKICKER_ONLY;
    } else {
        cerr << "UNKNOWN FORMATION xmlRoleAssigner.cpp: " << formation_string
                << endl;
    }
    return formation;
}

void getRoleAssignParameters(RoleAssignerParameters & parameters, unique_ptr<robotsports::StrategyType>& c) {
    parameters.calculateAllPaths = c->Options().calculateAllPaths();
    parameters.minimumEdgeLength = c->Options().minimumEdgeLength();
    parameters.maximumEdgeLength = c->Options().maximumEdgeLength();
    parameters.minimumDistanceToEndPoint = c->Options().minimumDistanceToEndPoint();
    parameters.nrVerticesFirstCircle = c->Options().nrVerticesFirstCircle();
    parameters.firstCircleRadius = c->Options().firstCircleRadius();
    parameters.nrVerticesSecondCircle = c->Options().nrVerticesSecondCircle();
    parameters.secondCircleRadius = c->Options().secondCircleRadius();
    parameters.safetyFactor = c->Options().safetyFactor();
    parameters.addBarierVertices = c->Options().addBarierVertices();
    parameters.addUniformVertices = c->Options().addUniformVertices();
    parameters.uniform_x_interval = c->Options().uniform_x_interval();
    parameters.uniform_y_interval = c->Options().uniform_y_interval();
    parameters.startingVelocityPenaltyFactor = c->Options().startingVelocityPenaltyFactor();
    parameters.addBallApproachVertices = c->Options().addBallApproachVertices();
    parameters.distToapplyBallApproachVertices = c->Options().distToapplyBallApproachVertices();
    parameters.ballApproachVerticesRadius = c->Options().ballApproachVerticesRadius();
    parameters.ballApproachNumberOfVertices = c->Options().ballApproachNumberOfVertices();
    parameters.manDefenseBetweenBallAndPlayer = c->Options().manDefenseBetweenBallAndPlayer();
    parameters.dist_before_penalty_area_for_sweeper = c->Options().dist_before_penalty_area_for_sweeper();
    parameters.grid_size = c->Options().grid_size();
    parameters.interceptionChanceStartDistance = c->Options().interceptionChanceStartDistance();
    parameters.interceptionChanceIncreasePerMeter = c->Options().interceptionChanceIncreasePerMeter();
    parameters.interceptionChancePenaltyFactor = c->Options().interceptionChancePenaltyFactor();
    parameters.grid_close_to_ball_normal_penalty = c->Options().grid_close_to_ball_normal_penalty();
    parameters.grid_close_to_ball_normal_radius = c->Options().grid_close_to_ball_normal_radius();
    parameters.grid_close_to_ball_restart_normal_penalty = c->Options().grid_close_to_ball_restart_normal_penalty();
    parameters.grid_close_to_ball_restart_normal_radius = c->Options().grid_close_to_ball_restart_normal_radius();
    parameters.grid_close_to_ball_restart_penalty_penalty = c->Options().grid_close_to_ball_restart_penalty_penalty();
    parameters.grid_close_to_ball_restart_penalty_radius = c->Options().grid_close_to_ball_restart_penalty_radius();
    parameters.grid_close_to_ball_restart_dropball_penalty = c->Options().grid_close_to_ball_restart_dropball_penalty();
    parameters.grid_close_to_ball_restart_dropball_radius = c->Options().grid_close_to_ball_restart_dropball_radius();
    parameters.grid_opponent_goal_clearance_x = c->Options().grid_opponent_goal_clearance_x();
    parameters.grid_opponent_goal_clearance_y = c->Options().grid_opponent_goal_clearance_y();
    parameters.grid_own_goal_clearance_x = c->Options().grid_own_goal_clearance_x();
    parameters.grid_own_goal_clearance_y = c->Options().grid_own_goal_clearance_y();
    parameters.nrDynamicPlannerIterations = c->Options().nrDynamicPlannerIterations();
    parameters.maxPossibleLinearSpeed = c->Options().maxPossibleLinearSpeed();
    parameters.maxPossibleLinearAcceleration = c->Options().maxPossibleLinearAcceleration();
    parameters.wait_on_non_optimal_position_during_prepare_phase = c->Options().wait_on_non_optimal_position_during_prepare_phase();
    // plannerOptions.auto_save_svg_period not handled for xml only when RoleAssigner skill is used
    parameters.autoAssignGoalie = c->Options().autoAssignGoalie();
    parameters.preferredSetplayKicker = c->Options().preferredSetplayKicker();
    parameters.preferredSetplayReceiver = c->Options().preferredSetplayReceiver();
    parameters.use_pass_to_position_for_attack_support = c->Options().use_pass_to_position_for_attack_support();
    parameters.man_to_man_defense_during_normal_play = c->Options().man_to_man_defense_during_normal_play();
    parameters.man_to_man_defense_during_setplay_against = c->Options().man_to_man_defense_during_setplay_against();
    parameters.interceptor_assign_use_ball_velocity = c->Options().interceptor_assign_use_ball_velocity();
    parameters.interceptor_assign_min_velocity_for_calculate_interception_position =  c->Options().interceptor_assign_min_velocity_for_calculate_interception_position();
    parameters.dist_to_goal_to_mark_opponent_as_goalie = c->Options().dist_to_goal_to_mark_opponent_as_goalie();
    parameters.setplay_against_dist_to_opponent = c->Options().setplay_against_dist_to_opponent();
    parameters.move_to_ball_left_field_position = c->Options().move_to_ball_left_field_position();

    parameters.svgDrawVelocity = c->Options().svgDrawVelocity();
    parameters.svgDrawEdges = c->Options().svgDrawEdges();
    parameters.svgDefaultTargetColor =  c->Options().svgDefaultTargetColor();
    parameters.svgBallColor = c->Options().svgBallColor();
    parameters.svgOriginalTargetColor = c->Options().svgOriginalTargetColor();
    parameters.svgTeamColor = c->Options().svgTeamColor();
    parameters.svgOpponentColor = c->Options().svgOpponentColor();
    parameters.svgOutputFileName = c->Options().svgOutputFileName();
    parameters.saveGridDataToFile = c->Options().saveGridDataToFile();
    parameters.svgRobotPlanner = c->Options().svgRobotPlanner();
    parameters.previous_role_end_pos_threshold  = c->Options().previous_role_end_pos_threshold();
    parameters.previous_role_bonus_end_pos_radius = c->Options().previous_role_bonus_end_pos_radius();
    parameters.priority_block_min_distance = c->Options().priority_block_min_distance();
    parameters.priority_block_max_distance = c->Options().priority_block_max_distance();
}

game_state_e gamestate_string_to_enum(std::string& gs) {
    game_state_e gameState  = game_state_e::NONE;
    if (gs == "Normal") {
        gameState = game_state_e::NORMAL;
    } else if (gs == "Parking") {
        gameState = game_state_e::PARKING;
    } else if (gs == "Begin Position") {
        gameState = game_state_e::BEGIN_POSITION;
    } else if (gs == "Kickoff") {
        gameState = game_state_e::KICKOFF;
    } else if (gs == "Kickoff Against") {
        gameState = game_state_e::KICKOFF_AGAINST;
    } else if (gs == "Freekick") {
        gameState = game_state_e::FREEKICK;
    } else if (gs == "Freekick Against") {
        gameState = game_state_e::FREEKICK_AGAINST;
    } else if (gs == "Goalkick") {
        gameState = game_state_e::GOALKICK;
    } else if (gs == "Goalkick Against") {
        gameState = game_state_e::GOALKICK_AGAINST;
    } else if (gs == "Throwin") {
        gameState = game_state_e::THROWIN;
    } else if (gs == "Throwin Against") {
        gameState = game_state_e::THROWIN_AGAINST;
    } else if (gs == "Corner") {
        gameState = game_state_e::CORNER;
    } else if (gs == "Corner Against") {
        gameState = game_state_e::CORNER_AGAINST;
    } else if (gs == "Penalty") {
        gameState = game_state_e::PENALTY;
    } else if (gs == "Penalty Against") {
        gameState = game_state_e::PENALTY_AGAINST;
    } else if (gs == "Penalty Shootout") {
        gameState = game_state_e::PENALTY_SHOOTOUT;
    } else if (gs == "Penalty Shootout Against") {
        gameState = game_state_e::PENALTY_SHOOTOUT_AGAINST;
    } else if (gs == "Dropped Ball") {
        gameState = game_state_e::DROPPED_BALL;
    } else {
        cerr << "Unknown game state in xml file: " << gs << endl;
        exit(-1);
    }
    return gameState;
}

ball_status_e ball_status_to_enum(const xsd::cxx::tree::optional<robotsports::BallStatusType>& ball_status)
{
    if (!ball_status) {
        return ball_status_e::FREE;
    }
    switch (ball_status.get()) {
        case BallStatusType::value::FREE:              return ball_status_e::FREE;
        case BallStatusType::value::OWNED_BY_PLAYER:   return ball_status_e::OWNED_BY_PLAYER;
        case BallStatusType::value::OWNED_BY_TEAM:     return ball_status_e::OWNED_BY_TEAM;
        case BallStatusType::value::OWNED_BY_OPPONENT: return ball_status_e::OWNED_BY_OPPONENT;
        default:
            cerr << "Unknown ball_status in xml file: " << ball_status.get() << endl;
            exit(-1);
    }
    return ball_status_e::FREE;
}

void fillEnvironment(Environment& rEnvironment, unique_ptr<robotsports::StrategyType>& c)
{
    if (c->Field() == 0) {
        // not defined, use default
        rEnvironment = FillDefaultEnvironment();
    }
    else {
        // If field info is present then overwrite the defaults with values from the xml file
        EnvironmentParameters env_params = {};
        env_params.SLM.A = c->Field()->field_length();  // [22.0]   field length including lines (y)
        env_params.SLM.B = c->Field()->field_width();  // [14.0]   field width including lines (x)
        env_params.SLM.C = c->Field()->penalty_area_width();   // [ 6.9]   penalty area width including lines (x)
        env_params.SLM.D = c->Field()->goal_area_width();   // [ 3.9]   goal area width including lines (x)
        env_params.SLM.E = c->Field()->penalty_area_length();  // [ 2.25]  penalty area length including lines (y)
        env_params.SLM.F = c->Field()->goal_area_length();  // [ 0.75]  goal area length including lines (y)
        env_params.SLM.G = c->Field()->corner_circle_diameter();  // [ 0.75]  corner circle radius including lines
        env_params.SLM.H = c->Field()->center_circle_diameter();   // [ 4.0]   inner circle diameter including lines
        env_params.SLM.I = c->Field()->penalty_spot_to_backline();   // [ 3.6]   penalty mark distance (y) including line to mark center (?)
        env_params.SLM.J = 0.15;  // [ 0.15]  penalty- and center mark diameter
        env_params.SLM.K = c->Field()->field_markings_width(); // [ 0.125] line width
        env_params.SLM.L = c->Field()->field_margin();   // [ 1.0]   field border (x) (between outer line and black safety border)
        env_params.SLM.M = 1.0;   // [ 1.0]   Technical Team Area width (x)
        env_params.SLM.N = 7.5;   // [ 7.5]   Technical Team Area length (y) (between safety borders)
        env_params.SLM.O = 1.0;   // [ 1.0]   Technical Team Area ramp length (y)
        env_params.SLM.P = 0.5;   // [ 0.5]   Technical Team Area ramp width (x)
        env_params.SLM.Q = 3.5;   // [ 3.5]   off-center distance to restart spots (x)
        env_params.penalty_area_present = c->Field()->penalty_area_present();
        env_params.technical_team_area_present = true;

        env_params.goal_width = c->Field()->goal_width();
        env_params.goal_length = c->Field()->goal_length();
        //        // parking info in case no technical area is present
        //        // park robots on the field.
        env_params.parking_area_width = c->Field()->parking_area_width();
        env_params.parking_area_length = c->Field()->parking_area_length();
        env_params.parking_distance_between_robots = c->Field()->parking_distance_between_robots();
        env_params.parking_distance_to_line = c->Field()->parking_distance_to_line();
        env_params.robot_size = c->Field()->robot_size();
        env_params.ball_radius = c->Field()->ball_radius();

        rEnvironment = Environment(env_params);
    }

}

void fillTeam(std::vector<RoleAssignerRobot>& Team, std::vector<RoleAssignerAdminTeam>& TeamAdmin,  std::vector<previous_role_assigner_result_t>& previous_results, bool& r_playerPassedBall, bool& r_team_has_ball, unique_ptr<robotsports::StrategyType>& c)
{
    long playerId = 0;
    r_playerPassedBall = false;
    for (StrategyType::Team_const_iterator team_iter = c->Team().begin(); team_iter != c->Team().end(); ++team_iter) {
        playerId++;
        RoleAssignerRobot P = {};
        RoleAssignerAdminTeam PA  = {};

        previous_role_assigner_result_t previous_result  = {};
        previous_result.present = (*team_iter).previous_result_present();
        previous_result.role = DynamicRoleToRole(StringToDynamicRole((*team_iter).previous_result_dynamic_role()), role_UNDEFINED);
        previous_result.end_position.x = (*team_iter).previous_result_x();
        previous_result.end_position.y = (*team_iter).previous_result_y();
        previous_result.ts = (*team_iter).previous_result_ts();
        previous_results.push_back(previous_result);
        PA.assigned = false;
        PA.result = {};

        if ((*team_iter).passedBall()) {
            r_playerPassedBall = true;
        }

        if ((*team_iter).isGoalie()) {
            P.player_type = player_type_e::GOALIE;
        } else {
            P.player_type = player_type_e::FIELD_PLAYER;
        }


        P.active = true;
        P.position = Geometry::Position(*team_iter->x(), *team_iter->y(), 0.0, 0.0, 0.0, team_iter->rz());
        P.velocity= Geometry::Position(team_iter->velx(), team_iter->vely(), 0.0, 0.0, 0.0, team_iter->velrz());
        
        // id of robot must be defined in xml-file
        if (not (*team_iter).id()) {
            cout << "No id defined for robot" << endl;
            exit(1);
        }
         P.robotId = (*team_iter).id().get();

        // trackingId of robot must be defined in xml-file
        if (not (*team_iter).trackingId()) {
            cout << "trackingId not defined for an own player" << endl;
            exit(1);
        }
        P.trackingId = (*team_iter).trackingId().get();


        P.controlBall = (*team_iter).hasBall();
        if (P.controlBall) {
            r_team_has_ball = true;
        }
        P.time_in_own_penalty_area = (*team_iter).time_in_own_penalty_area();
        P.time_in_opponent_penalty_area =  (*team_iter).time_in_enemy_penalty_area();
        P.passBall = (*team_iter).passedBall();
        PA.robotId = P.robotId;
        Team.push_back(P);
        TeamAdmin.push_back(PA);
    }
}



void fillOpponents(std::vector<RoleAssignerOpponent>& Opponents, unique_ptr<robotsports::StrategyType>& c)
{
    long playerId = 0;
    for (StrategyType::Opponent_const_iterator opponent_iter =
            c->Opponent().begin(); opponent_iter != c->Opponent().end();
            ++opponent_iter) {
        playerId++;
        if (not (*opponent_iter).trackingId()) {
            cout << "trackingId not defined for a player of the opponents" << endl;
            exit(1);
        }
        long trackingId = (*opponent_iter).trackingId().get();
        if (trackingId < 10) {
            trackingId = playerId + 10;
            cout <<  " trackingId must be above 10 for an opponent." << endl;
            exit(1);
        }
        RoleAssignerOpponent opponent;
        opponent.position = Geometry::Position(*opponent_iter->x(), *opponent_iter->y(), 0.0, 0.0, 0.0, opponent_iter->rz());
        opponent.velocity= Geometry::Position(opponent_iter->velx(), opponent_iter->vely(), 0.0, 0.0, 0.0, opponent_iter->velrz());
        opponent.trackingId = playerId;
        Opponents.push_back(opponent);
    }
}

void role_assigner_with_xml_input(const std::string& input_filename, const std::string& output_base_directory) {
    string filename = input_filename;
    bool print_only_errors = false;
    if (not print_only_errors) {
        cout << "reading file : " << filename << endl;
    }

    bool robot_strategy_parameter_no_defender_main_during_setplay = true;
    team_formation_e robot_strategy_parameter_attack_formation = FORMATION_013;
    team_formation_e robot_strategy_parameter_defense_formation = FORMATION_013;

    RoleAssignerParameters parameters = {};
    Geometry::Position ball_pos = Geometry::Position();
    Geometry::Position ball_vel = Geometry::Position();
    game_state_e gameState;
    std::string description = "";
    MRA::Environment environment = {};
    ball_pickup_position_t pickup_pos = {};
    bool pickup_pos_set = false;
    std::vector<previous_role_assigner_result_t> previous_results = {};

    std::vector<RoleAssignerRobot> Team = {};
    std::vector<RoleAssignerAdminTeam> TeamAdmin = {};

    std::vector<RoleAssignerOpponent> Opponents = {};
    vector<Geometry::Point> parking_positions = {};

    bool passIsRequired = false;
    long ownPlayerWithBall = -1;
    bool playerPassedBall = false;
    bool team_has_ball = false;
    bool ball_is_valid = false;
    pass_data_t pass_data = {.target_id=-1};
    previous_used_ball_by_role_assinger_t previous_ball = {};
    ball_status_e ball_status = ball_status_e::FREE;
    try {
        unique_ptr<robotsports::StrategyType> c(robotsports::Situation(filename));
        description = c->Description();
        if (c->Ball() != 0) {
            ball_is_valid = true;
            ball_pos.x = *c->Ball()->x();
            ball_pos.y = *c->Ball()->y();
            ball_vel.x = c->Ball()->velx();
            ball_vel.y = c->Ball()->vely();
        }

        fillTeam(Team, TeamAdmin, previous_results, playerPassedBall, team_has_ball, c);
        fillOpponents(Opponents, c);

        gameState = gamestate_string_to_enum(c->GameState());
        fillEnvironment(environment, c);
        getRoleAssignParameters(parameters, c);
        robot_strategy_parameter_no_defender_main_during_setplay = c->Options().no_defender_main_during_setplay();
        robot_strategy_parameter_attack_formation =  StringToFormation(c->AttackFormation());
        robot_strategy_parameter_defense_formation = StringToFormation(c->DefenseFormation());

        string svgOutputFileName = parameters.svgOutputFileName;
        std::size_t found = svgOutputFileName.find_last_of("/");
        if (found != string::npos) {
            // svgOutputFileName contains /,the sub directory relative to current directory should exists
            auto svg_output_dir = output_base_directory + svgOutputFileName.substr(0,found);
            if (not std::filesystem::exists(svg_output_dir)) {
                auto created_new_directory = std::filesystem::create_directory(svg_output_dir);
                if (not created_new_directory) {
                    // Either creation failed or the directory was already present.
                    cout << "FAILED to create directory: " << svg_output_dir << endl;
                }
                else {
                    cout << "Successful created directory: " << svg_output_dir << endl;
                }
            }
        }

        if (c->PickupPosition()) {
            pickup_pos.x = c->PickupPosition()->x();
            pickup_pos.y = c->PickupPosition()->y();
            pickup_pos.valid = c->PickupPosition()->valid();
            pickup_pos.ts = c->PickupPosition()->ts();
            pickup_pos_set = true;
        }
        if (c->SituationInfo()) {
            if (not c->SituationInfo()->passing_required()) {
                cout << "SituationInfo must have attribute \"passing_required\"" << endl;
                exit(1);
            }
            passIsRequired = c->SituationInfo()->passing_required().get();
            ball_status = ball_status_to_enum(c->SituationInfo()->ball_status());
            if (c->SituationInfo()->PassData() != 0) {
                pass_data.ts = c->SituationInfo()->PassData()->ts();
                pass_data.origin_pos.x = c->SituationInfo()->PassData()->origin_x();
                pass_data.origin_pos.y = c->SituationInfo()->PassData()->origin_y();
                pass_data.target_pos.x = c->SituationInfo()->PassData()->target_x();
                pass_data.target_pos.y = c->SituationInfo()->PassData()->target_y();
                pass_data.valid = c->SituationInfo()->PassData()->valid();
                pass_data.target_id = c->SituationInfo()->PassData()->target_id();
            }
        }
        if (c->PreviousBall()) {
            previous_ball.present = true;
            previous_ball.x = c->PreviousBall()->x();
            previous_ball.y = c->PreviousBall()->y();
        }
        else {
            previous_ball.present = false;
        }

        if (c->ParkingInfo()) {
            auto parkpos_list = c->ParkingInfo().get().ParkingPosition();
            for (auto parkpos_iter = parkpos_list.begin(); parkpos_iter != parkpos_list.end(); ++parkpos_iter) {
                parking_positions.push_back(Geometry::Point((*parkpos_iter).x().get(), (*parkpos_iter).y().get()));
            }
        }
        else {
            // default parking positions
            parking_positions.push_back(Geometry::Point(-6.375, -0.25));
            parking_positions.push_back(Geometry::Point(-6.375, -1.25));
            parking_positions.push_back(Geometry::Point(-6.375, -2.25));
            parking_positions.push_back(Geometry::Point(-6.375, -3.25));
            parking_positions.push_back(Geometry::Point(-6.375, -4.25));
        }

    } catch (const xml_schema::exception& e) {
        cerr << "exception: " << e << endl;
        return;
    } catch (const xml_schema::properties::argument&) {
        cerr << "invalid property argument (empty namespace or location)"
                << endl;
        return;
    } catch (const xsd::cxx::xml::invalid_utf16_string&) {
        cerr << "invalid UTF-16 text in DOM model" << endl;
        return;
    } catch (const xsd::cxx::xml::invalid_utf8_string&) {
        cerr << "invalid UTF-8 text in object model" << endl;
        return;
    }
    if (not print_only_errors) {
        cerr << "start planner" << endl;
        cout << "running : " << endl << description << endl;
    }

    string orginal_svgOutputFileName = parameters.svgOutputFileName;
    auto start = std::chrono::system_clock::now();

    std::vector<RoleAssignerResult> player_paths = {};

    char buffer[250];
    sprintf(buffer, ".svg");
    filename = orginal_svgOutputFileName;
    if (filename.size() > 4) {
        filename.replace(filename.end() - 4, filename.end(), buffer);
    }
    parameters.svgOutputFileName = filename;

    if (!pickup_pos_set) {
        pickup_pos.x = ball_pos.x;
        pickup_pos.y = ball_pos.y;
        pickup_pos.valid = ownPlayerWithBall >= 0;
        pickup_pos.ts = 0.0;
    }

    std::vector<RunData> run_results = {};
    string run_filename = parameters.svgOutputFileName;
    auto formation = getListWithRoles(gameState, ball_status,
                                      robot_strategy_parameter_no_defender_main_during_setplay,
                                      robot_strategy_parameter_attack_formation,
                                      robot_strategy_parameter_defense_formation);


    RoleAssignerInput ra_input = {};
    ra_input.gamestate = gameState;
    ra_input.ball.status = ball_status;
    ra_input.ball.position = ball_pos;
    ra_input.ball.velocity = ball_vel;
    ra_input.ball.is_valid = ball_is_valid;
    ra_input.formation = formation;
    ra_input.team = Team;
    ra_input.opponents = Opponents;
    ra_input.parking_positions = parking_positions;
    ra_input.ball_pickup_position = pickup_pos;
    ra_input.passIsRequired = passIsRequired;
    ra_input.pass_data = pass_data;
    ra_input.environment = environment;

    RoleAssignerState ra_state;
    ra_state.previous_ball = previous_ball;
    ra_state.previous_results  = previous_results;

    RoleAssignerParameters ra_parameters = parameters;
    RoleAssignerOutput ra_output = {};
    auto ra_state_org = ra_state;

    xml_assign_roles(ra_input, ra_state, ra_output, ra_parameters);

    if (ra_output.player_paths.size() > 0) {
        if (not print_only_errors) {
            cerr << RoleAssignerResultToString(ra_output, ra_input) << endl << flush;
        }
        RoleAssignerSvg::role_assigner_data_to_svg(ra_input, ra_state_org, ra_output, ra_parameters, run_filename);

    } else {
        cerr << "<< XML: no path received" << endl << flush;
    }

    auto finish = std::chrono::system_clock::now();
    double elapsed_seconds = std::chrono::duration_cast< std::chrono::duration<double> > (finish - start).count();
    // test start position assignments
    if (not print_only_errors) {
        std::cout << "elapsed time: " << elapsed_seconds * 1000 << " [ms]" << std::endl;
    }
}
