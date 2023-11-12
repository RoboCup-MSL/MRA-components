#ifndef MRA_ROBOTSPORTS_LOCAL_BALL_TRACKING_SEQUENCE_CLUSTERING_HPP
#define MRA_ROBOTSPORTS_LOCAL_BALL_TRACKING_SEQUENCE_CLUSTERING_HPP

// generated protobuf types from interface of this component
#include "RobotsportsLocalBallTracking.hpp"
#include "sequence_clustering_balldef.hpp"

void local_ball_tracking_sequence_clustering(   unsigned nrBallsThisTime,
                            const std::vector<ball_candidate_t>& ballData,
                            const MRA::RobotsportsLocalBallTracking::InputType &input,
                            const MRA::RobotsportsLocalBallTracking::ParamsType &params,
                            MRA::RobotsportsLocalBallTracking::OutputType &output,
                            MRA::RobotsportsLocalBallTracking::StateType &state,
                            unsigned max_num_balls);

#endif
