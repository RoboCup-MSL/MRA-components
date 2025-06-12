#ifndef FALCONS_ACTION_FETCH_BALL_HPP
#define FALCONS_ACTION_FETCH_BALL_HPP

#include "action_base.hpp"

class ActionFetchBall : public ActionBase
{
public:
    ActionFetchBall();
    virtual ~ActionFetchBall();

    void execute() override;

private:
    // Add any private members or methods specific to fetching the ball
};

#endif // FALCONS_ACTION_FETCH_BALL_HPP
