#ifndef MRA_ROBOTSPORTS_LOCAL_BALL_TRACKING_SEQUENCE_CLUSTERING_HPP
#define MRA_ROBOTSPORTS_LOCAL_BALL_TRACKING_SEQUENCE_CLUSTERING_HPP

// generated protobuf types from interface of this component
#include "../RobotsportsLocalBallTracking_datatypes.hpp"
#include "sequence_clustering_track_ball.hpp"

void local_ball_tracking_sequence_clustering(
                            double timestamp,
                            unsigned nrBallsThisTime,
                            const std::vector<ball_candidate_t>& ballData,
                            const MRA::RobotsportsLocalBallTracking::Input &input,
                            const MRA::RobotsportsLocalBallTracking::Params &params,
                            MRA::RobotsportsLocalBallTracking::Output &output,
                            MRA::RobotsportsLocalBallTracking::State &state,
                            unsigned max_num_balls);

#endif
