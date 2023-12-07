#ifndef MRA_ROBOTSPORTS_LOCAL_BALL_TRACKING_PREPROCESSING_HPP
#define MRA_ROBOTSPORTS_LOCAL_BALL_TRACKING_PREPROCESSING_HPP

#include "../RobotsportsLocalBallTracking_datatypes.hpp"
#include "sequence_clustering_common_defintions.hpp"

int local_ball_tracking_preprocessing(std::vector<ball_candidate_t>& ballData,
                                    const MRA::RobotsportsLocalBallTracking::Input &input,
                                    const MRA::RobotsportsLocalBallTracking::Params &params);

#endif
