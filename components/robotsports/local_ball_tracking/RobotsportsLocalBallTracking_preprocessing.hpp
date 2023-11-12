#ifndef MRA_ROBOTSPORTS_LOCAL_BALL_TRACKING_PREPROCESSING_HPP
#define MRA_ROBOTSPORTS_LOCAL_BALL_TRACKING_PREPROCESSING_HPP

// generated protobuf types from interface of this component
#include "RobotsportsLocalBallTracking.hpp"
#include "sequence_clustering_balldef.hpp"

int local_ball_tracking_preprocessing(std::vector<ball_candidate_t>& ballData,
                                    const MRA::RobotsportsLocalBallTracking::InputType &input,
                                    const MRA::RobotsportsLocalBallTracking::ParamsType &params);

#endif
