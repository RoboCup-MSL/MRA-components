#include "ActionGetBall.hpp"
#include <cmath>

using namespace falcons::action_planning;

ActionGetBall::ActionGetBall()
: ActionBase("getball", types::ActionType::ACTION_GETBALL)
{
}

void ActionGetBall::tick(const types::WorldState& world_state, const types::Settings& settings)
{
    // Reset verdict and result
    verdict_.clear();
    actionresult_ = types::ActionResult::ACTIONRESULT_INVALID;

    // Example: always enable ballhandlers (if output msg supports it)
    // output.bh_enabled = true;

    // Check if robot has the ball
    if (world_state.robot.hasball) {
        actionresult_ = types::ActionResult::ACTIONRESULT_PASSED;
        verdict_ = "robot has the ball";
        return;
    }

    // Fail if robot is inactive
    if (!world_state.robot.active) {
        actionresult_ = types::ActionResult::ACTIONRESULT_FAILED;
        verdict_ = "robot is inactive";
        return;
    }

    // Fail if no ball
    if (!world_state.balls.size()) {
        actionresult_ = types::ActionResult::ACTIONRESULT_FAILED;
        verdict_ = "robot lost track of the ball";
        return;
    }

    // Fail if teammate has the ball
    for (const auto& teammate : world_state.teammates) {
        if (teammate.hasball) {
            actionresult_ = types::ActionResult::ACTIONRESULT_FAILED;
            verdict_ = "teammate got the ball";
            return;
        }
    }

    // Fail if ball is too far away
    double dx = world_state.balls[0].pose.position.x - world_state.robot.pose.position.x;
    double dy = world_state.balls[0].pose.position.y - world_state.robot.pose.position.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    double action_radius = settings.radius;
    if (settings.radius < 0.1) {
        actionresult_ = types::ActionResult::ACTIONRESULT_FAILED;
        verdict_ = "invalid configuration, settings.radius too small";
        return;
    }
    if (dist > settings.radius) {
        actionresult_ = types::ActionResult::ACTIONRESULT_FAILED;
        verdict_ = "ball too far away";
        return;
    }

    // Otherwise, keep running
    actionresult_ = types::ActionResult::ACTIONRESULT_RUNNING;
    verdict_ = "fetching ball";
}
