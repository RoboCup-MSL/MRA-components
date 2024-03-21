#include "tracker.hpp"
#include <cmath>

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

Tracker::operator TrackerState() const
{
    MRA::FalconsLocalizationVision::TrackerState result;
    *result.mutable_pose() = fitResult;
    *result.mutable_creation() = creation;
    *result.mutable_lastactive() = lastActive;
    result.set_id(id);
    return result;
}

float Tracker::confidence() const
{
    // original Falcons localization also included things like age into this heuristic
    // but later, responsibility and capability was shifted to WorldModel
    // so here we keep it simple: pixel-based confidence
    // fitScore is in range [0.0, 1.0] where 0.0 is perfect
    // confidence however is inverted: higher is better
    // clipping just to be safe
    return std::max(0.0, std::min(1.0, 1.0 - fitScore));
}

