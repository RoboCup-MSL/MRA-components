#include "RobotsportsLocalBallTracking_sequence_clustering.hpp"
#include "sequence_clustering_best_uid.hpp"
#include "sequence_clustering_track_ball.hpp"
#include <cmath>
#include <google/protobuf/util/time_util.h>
#include "sequence_clustering_common_defintions.hpp"

static sc_global_data_t pscgd;


void local_ball_tracking_sequence_clustering(
                            double timestamp,
                            const std::vector<ball_candidate_t>& ballData,
                            const MRA::RobotsportsLocalBallTracking::Input& input,
                            const MRA::RobotsportsLocalBallTracking::Params& params,
                            MRA::RobotsportsLocalBallTracking::Output& output,
                            MRA::RobotsportsLocalBallTracking::State& state,
                            unsigned max_num_balls)
{

    if (not state.is_initialized()) {
        sequence_clustering_initialize(pscgd, params);
        state.set_is_initialized(true);
    }

    // now run sc_bm code
    ball_estimate_t ball_estimate;
    int use_next_best_ball = 0; // if 1, then go to next best ball

    sc_result_e ret = sequence_clustering_track_ball(ball_estimate, ballData, timestamp, use_next_best_ball, pscgd, params, max_num_balls);
    if (ret == SC_SUCCESS) {
        // update ball position in world model since a successful step has been done

        auto timestamp_obj = google::protobuf::util::TimeUtil::MillisecondsToTimestamp(ball_estimate.timestamp* 1000);
        output.mutable_ball()->mutable_pos_vel_fcs()->mutable_position()->set_x(ball_estimate.xhat); // position X, replaced TPB on 20161210 from ball_estimate.x
        output.mutable_ball()->mutable_pos_vel_fcs()->mutable_position()->set_y(ball_estimate.yhat); // position Y, replaced TPB on 20161210 from ball_estimate.y
        output.mutable_ball()->mutable_pos_vel_fcs()->mutable_position()->set_z(ball_estimate.z); // position Z
        output.mutable_ball()->mutable_pos_vel_fcs()->mutable_velocity()->set_x(ball_estimate.xdot); // velocity in X
        output.mutable_ball()->mutable_pos_vel_fcs()->mutable_velocity()->set_y(ball_estimate.ydot); // velocity in Y
        output.mutable_ball()->mutable_pos_vel_fcs()->mutable_velocity()->set_z(ball_estimate.zdot); // velocity in Z
        output.mutable_ball()->set_confidence(ball_estimate.hconf); // moving average confidence

        output.mutable_ball()->mutable_timestamp()->CopyFrom(timestamp_obj); // timestamp based off of liveseconds
        double age = (timestamp - ball_estimate.timestamp);
        if (age < 0) {
            age = 0.0;
        }
        if (age > params.ball_time_to_forget()) {

            output.mutable_ball()->set_confidence(0.0);
        }
    }
}
