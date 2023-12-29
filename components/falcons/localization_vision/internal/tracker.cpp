#include "tracker.hpp"

using namespace MRA::FalconsLocalizationVision;


static int s_id = 0;

Tracker::Tracker(Params const &params)
{
    id = s_id++;
    this->params = params;
    step = params.solver().actionradius();
}

Tracker::Tracker(Params const &params, MRA::Datatypes::Circle const &c)
{
    id = s_id++;
    this->params = params;
    guess.x = c.center().x();
    guess.y = c.center().y();
    guess.rz = 0.0;
    step.x = c.radius();
    step.y = c.radius();
    step.rz = M_PI * 2;
}

Tracker::Tracker(Params const &params, TrackerState const &st)
{
    id = st.id();
    this->params = params;
    guess = st.pose();
    step = params.solver().actionradius();
    creation = st.creation();
    lastActive = st.lastactive();
}

float Tracker::confidence() const
{
    return 1.0; // TODO
}

