#ifndef MRA_ROBOTSPORTS_LOCAL_BALL_TRACKING_NATIVE_FILTER_HPP
#define MRA_ROBOTSPORTS_LOCAL_BALL_TRACKING_NATIVE_FILTER_HPP

// generated protobuf types from interface of this component
#include "RobotsportsLocalBallTracking.hpp"
#include "seq_clustering_balldef.hpp"

void local_ball_tracking_native_filter(unsigned nrBallsThisTime,
                                       const std::vector<ball_feature_t>& ballData,
                                       const MRA::RobotsportsLocalBallTracking::ParamsType &params,
                                       const MRA::RobotsportsLocalBallTracking::InputType &input,
                                       MRA::RobotsportsLocalBallTracking::StateType &state);

#endif
