#ifndef FALCONS_ACTION_GET_BALL_HPP
#define FALCONS_ACTION_GET_BALL_HPP

#include "action_base.hpp"
#include "mra_falcons_action_get_ball/msg/input.hpp"
#include "mra_falcons_action_get_ball/msg/config.hpp"
#include <string>

class ActionGetBall : public ActionBase<mra_falcons_action_get_ball::msg::Input, mra_falcons_action_get_ball::msg::Config>
{
public:
    ActionGetBall();

    void tick(const mra_common_msgs::msg::WorldState& world_state, const input_type& input) override;
};

#endif // FALCONS_ACTION_GET_BALL_HPP
