#include "ActionGetBall.hpp"
#include "mra_tracing/tracing.hpp"
#include <cmath>

using namespace falcons::action_planning;

ActionGetBall::ActionGetBall()
: ActionBase("getball", types::ActionType::ACTION_GETBALL)
{
}

void ActionGetBall::tick(
    const types::WorldState& world_state,
    const types::Settings& settings,
    types::ActionResult& action_result,
    types::Targets& targets)
{
    TRACE_FUNCTION_INPUTS(settings);
    // Unpack inputs/settings
    double action_radius = settings["radius"].as<double>();

    // Example: always enable ballhandlers (if output msg supports it)
    // output.bh_enabled = true;

    // Check if robot has the ball
    if (world_state.robot.hasball) {
        action_result.status = types::ActionResult::ACTIONRESULT_PASSED;
        action_result.details = "robot has the ball";
        return;
    }

    // Fail if robot is inactive
    if (!world_state.robot.active) {
        action_result.status = types::ActionResult::ACTIONRESULT_FAILED;
        action_result.details = "robot is inactive";
        return;
    }

    // Fail if no ball
    if (!world_state.balls.size()) {
        action_result.status = types::ActionResult::ACTIONRESULT_FAILED;
        action_result.details = "robot lost track of the ball";
        return;
    }

    // Fail if teammate has the ball
    for (const auto& teammate : world_state.teammates) {
        if (teammate.hasball) {
            action_result.status = types::ActionResult::ACTIONRESULT_FAILED;
            action_result.details = "teammate got the ball";
            return;
        }
    }

    // Fail if ball is too far away
    double dx = world_state.balls[0].pose.position.x - world_state.robot.pose.position.x;
    double dy = world_state.balls[0].pose.position.y - world_state.robot.pose.position.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    if (action_radius < 0.1) {
        action_result.status = types::ActionResult::ACTIONRESULT_FAILED;
        action_result.details = "invalid configuration, settings.radius too small";
        return;
    }
    if (dist > action_radius) {
        action_result.status = types::ActionResult::ACTIONRESULT_FAILED;
        action_result.details = "ball too far away";
        return;
    }

    // Otherwise, keep running
    action_result.status = types::ActionResult::ACTIONRESULT_RUNNING;
    action_result.details = "fetching ball";
}
