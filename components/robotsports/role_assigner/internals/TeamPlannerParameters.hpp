/**
 *  @file
 *  @brief    Options for generating the visibility graph and the path planner
 *  @curator Jürge van Eijck
 */
#ifndef ROLE_ASSIGNER_PARAMETERS_HPP
#define  ROLE_ASSIGNER_PARAMETERS_HPP 1

#include "planner_types.hpp" // define of planner_options_t

#include <string>

namespace MRA {
// Constructing a new object will set the default options.
class TeamPlannerParameters {
public:
    // Calculate for all player a path, otherwise stop if own path is calculated.
    static bool calculateAllPaths;

    // Minimum distance between to vertices. This will make the path finding algorithm faster
    static double minimumEdgeLength;

    // Maximum length of and edge. Edges that are longer will not be added to the graph and there not be part of a path.
    static double maximumEdgeLength;

    // Barriers that are far from a reasonable path are not considered. A
    // barrier is "near", if it is somewhat in between the end points or at small distance of an end point.
    static double minimumDistanceToEndPoint;

    // Number of vertices on the first circle around the object. Set to 0 if      * none.
    static int nrVerticesFirstCircle;

    // Radius of the first circle of vertices around the object.
    static double firstCircleRadius;

    // Number of vertices on the second circle around the object. Set to 0 if none.
    static int nrVerticesSecondCircle;

    // Radius of the second circle of vertices around the object.
    static double secondCircleRadius;

    // Factor for obstacle cost
    static double safetyFactor;

    // If vertices around barriers should be added
    static bool addBarierVertices;

    // If uniformly distributed vertices should be added
    static bool addUniformVertices;

    // spacing in x between vertices when applying uniform grid
    static double uniform_x_interval;

    // spacing in y between vertices when applying uniform grid
    static double uniform_y_interval;

    // Penalty for starting velocity in other direction than intended direction
    static double startingVelocityPenaltyFactor;
    
    // The distance from the border of the field, when approach vertices must be applied
    static double distToapplyBallApproachVertices;

    // If ball approach distributed vertices should be added
    static bool addBallApproachVertices;
    
    // approach vertices radius around ball
    static double ballApproachVerticesRadius;
    
    // number of approach vertices around ball
    static int ballApproachNumberOfVertices;

    // when playing man defense, the player will position itself between the ball and the player.
    static bool manDefenseBetweenBallAndPlayer;

    //      distance for sweeper to penalty area
    static double dist_before_penalty_area_for_sweeper;

    //  grid size for grid-based calculations
    static double grid_size;

    //  number of iterations for the dynamic planner (intercept ball, 0 is normal robotplanner)
    static int nrDynamicPlannerIterations;

    //  maximum possible linear speed (used for interception calculations)
    static double maxPossibleLinearSpeed;

    //  maximum possible linear acceleration
    static double maxPossibleLinearAcceleration;

    // distance when penalty for interception thread must be applied.
    static double interceptionChanceStartDistance;

    // distance increase when penalty for interception thread must be applied.
    static double interceptionChanceIncreasePerMeter;

    // penalty factor for interception thread calculation
    static double interceptionChancePenaltyFactor;


    // parameters to get a minimal distance to the ball (used function is a parabool).
    // default dist to ball 4 meter + 17000
    // restart dist to ball: freekick, goalkick, corner, throwin:  2.5 meter + 17000
    // restart dist to dropball: :  1.5 meter + 17000

    // penalty in grid for being to close to the ball within the normal radius.
    static double grid_close_to_ball_normal_penalty;

    // radius to apply penalty in grid for being to close to the ball during normal situation.
    static double grid_close_to_ball_normal_radius;

    // penalty in grid for being to close to the ball within the restart radius.
    static double grid_close_to_ball_restart_normal_penalty;

    // radius to apply penalty in grid for being to close to the ball during normal restart situation.
    static double grid_close_to_ball_restart_normal_radius;

    // penalty in grid for being to close to the ball within the penalty radius.
    static double grid_close_to_ball_restart_penalty_penalty;

    // radius to apply penalty in grid for being to close to the ball during penalty  situation.
    static double grid_close_to_ball_restart_penalty_radius;

    // penalty in grid for being to close to the ball within the dropball radius.
    static double grid_close_to_ball_restart_dropball_penalty;

    // radius to apply penalty in grid for being to close to the ball during dropball situation.
    static double grid_close_to_ball_restart_dropball_radius;

    // x of area where no supporting robot should be positioned (forbidden area).
    static double grid_opponent_goal_clearance_x;

    // y of area where no supporting robot should be positioned (forbidden area).
    static double grid_opponent_goal_clearance_y;

    // x of area where no supporting robot should be positioned (forbidden area).
    static double grid_own_goal_clearance_x;

    // y of area where no supporting robot should be positioned (forbidden area).
    static double grid_own_goal_clearance_y;

    // number of robots that must be present before pass play is used (otherwise player with ball goes to goal)
    static int nr_robots_needed_for_pass_play;

    //  number of attack supporting robots when opponent is attacking
    static int nr_attack_support_during_defensive_period;

    // must attackers wait on non optimal position during prepare phase (and move to best position when game starts)
    static bool wait_on_non_optimal_position_during_prepare_phase;

    // must priority block be applied : yes | no
    static bool priority_block_apply;
    // priority block minimal distance to ball during priority block
    static double priority_block_min_distance;

    // priority block maximum distance to ball during priority block
    static double priority_block_max_distance;
    /** priority block max allowed distance to defense line (between min and max priortity block distance to the ball */
    static double priority_block_max_distance_to_defense_line;
    /** max y location of the ball before applying priority block */
    static double priority_block_max_ball_y;
    /** max allowed distance of opponents to ball before priority block is applied */
    static double priority_block_max_opponent_to_ball_dist;
    /* must priority block algorithm check if ball is area  (< priority_block_max_ball_y) */
    static bool priority_block_check_ball_in_area;
    /* must priority block algorithm check if opponent is close to ball (< priority_block_max_opponent_to_ball_dist) */
    static bool priority_block_check_opponent_close_to_ball;

    // addition distance for attack supporter to stay from sideline: give more room to adjust position when pass is not perfect
    static double attack_supporter_extra_distance_to_stay_from_sideline;

    // distance between receiver and ball during game restart
    static double restart_receiver_ball_dist;

    //  distance between shooter and ball during game restart
    static double restart_shooter_ball_dist;

    // path cost equality threshold: if equal additional rule (x,y location) is applied to determine the best path
    static double equality_cost_threshold;

    // select_lowest_robot_nr_for_dynamic_role: flag if lowest robot must be selected if multiple robots want a certain dynamic role.
    static  bool select_lowest_robot_nr_for_dynamic_role;

    // Must previous role bonus be applied (better if previous dynamic_role was the same)
    static bool previous_role_bonus_must_be_applied;

    // Radius of current end-position compared to previous end-position to apply previous role bonus
    static double previous_role_end_pos_threshold;

    // cost function bonus if previous role end position is close current role end position
    static double previous_role_bonus_end_pos_radius;


    // Use pass to position for attack supporter as ball position
    static bool use_pass_to_position_for_attack_support;

    // apply man to man defense during setplay against (bool)
    static bool man_to_man_defense_during_normal_play;

    // apply man to man defense during setplay against (bool)
    static bool man_to_man_defense_during_setplay_against;

    /* max distance to opponent goal to mark an opponent as goalie */
    static double dist_to_goal_to_mark_opponent_as_goalie;

    /* preferred distance to keep to opponent during setplay against */
    static double setplay_against_dist_to_opponent;

    // Move to ball left field position (or move to current ball position)
    static bool move_to_ball_left_field_position;


    static double auto_save_svg_period; // -1 no save, otherwise interval for auto save svg
    static bool logCsvOutput;  // should team-planner log the output as csv (on request of client)

    // write output to svg file with the name, if empty ("") then no file is written.
    static std::string svgOutputFileName;

    static std::string svgDefaultTargetColor;
    static std::string svgBallColor;
    static std::string svgOriginalTargetColor;

    static std::string svgTeamColor;
    static std::string svgOpponentColor;
    static bool svgDrawVelocity; // draw velocity vectors

    static bool svgDrawEdges; // draw edges

    /**
     * if true, grid data is saved to file, for debugging purposes.
     * file-names will be constructed from the svgOutputFileName or using a generic name if that is not available.
     */
    static bool saveGridDataToFile;

    /**
     * if true, robot planner will write its output to a svg file, for debugging purposes.
     * file-names will be constructed from the svgOutputFileName or using a generic name if that is not available.
     */
    static bool svgRobotPlanner;

    static int preferredSetplayKicker;
    static int preferredSetplayReceiver;
    static double setplay_margin_to_penalty_area_side;  // min distance for setplay receiver to side of penalty area
    static bool interceptor_assign_use_ball_velocity; // use ball velocity to determine the interceptor
    static double interceptor_assign_min_velocity_for_calculate_interception_position; // minimum ball velocity needed to calculate interception position for interceptor role assignment
    static int dedicatedSweeper;
    static bool autoAssignGoalie;
    static bool lobShotWhenPossible;
    static double min_y_for_lob_shot;
    static double outsideFieldMargin;

    static double kickoff_fp1_x;
    static double kickoff_fp1_y;
    static double kickoff_fp2_x;
    static double kickoff_fp2_y;
    static double kickoff_fp3_x;
    static double kickoff_fp3_y;
    static double kickoff_fp4_x;
    static double kickoff_fp4_y;

    static double kickoff_against_fp1_x;
    static double kickoff_against_fp1_y;
    static double kickoff_against_fp2_x;
    static double kickoff_against_fp2_y;
    static double kickoff_against_fp3_x;
    static double kickoff_against_fp3_y;
    static double kickoff_against_fp4_x;
    static double kickoff_against_fp4_y;

    TeamPlannerParameters();
    std::string toString() const;
};

std::string FormationAsString(team_formation_e formation);

} // namespace

#endif //  ROLE_ASSIGNER_PARAMETERS_HPP
