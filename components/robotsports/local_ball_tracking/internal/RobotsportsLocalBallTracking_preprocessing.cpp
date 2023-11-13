#include "RobotsportsLocalBallTracking_preprocessing.hpp"
#include "sequence_clustering_track_ball.hpp"
#include <google/protobuf/util/time_util.h>
#include "sequence_clustering_common_defintions.hpp"


static void copy_to_ball_candidate_struct(ball_candidate_t &r_bf,
        const ::MRA::RobotsportsLocalBallTracking::BallCandidate observed_ball_candidate,
        balltype_e ball_type,
        const MRA::RobotsportsLocalBallTracking::Params &params) {
    r_bf.x = observed_ball_candidate.measured_pose_fcs().x();
    r_bf.y = observed_ball_candidate.measured_pose_fcs().y();
    r_bf.z = observed_ball_candidate.measured_pose_fcs().z();
    r_bf.confidence = observed_ball_candidate.confidence();
    r_bf.type = ball_type;
    r_bf.sigma = observed_ball_candidate.sigma();
    r_bf.timestamp = google::protobuf::util::TimeUtil::TimestampToMilliseconds(observed_ball_candidate.timestamp()) / 1000.0;
    r_bf.in_air = observed_ball_candidate.measured_pose_fcs().z() > params.min_height_in_air();
    r_bf.is_free = true; /* is rolling freely (false or true) */   // TODO implement check if ball is free. Disable in original
};


int local_ball_tracking_preprocessing(std::vector<ball_candidate_t>& ballData,
                                      const MRA::RobotsportsLocalBallTracking::Input &input,
                                      const MRA::RobotsportsLocalBallTracking::Params &params) {

    // fill measurements
    unsigned nrBallsThisTime = 0;
    unsigned max_balls = ballData.size();

    bool isFrontCameraBallAvailable = input.frontcamera_balls_size() > 0;
    const bool suppressOmni = params.suppress_omni();
    const bool includeOmni = !suppressOmni || (!isFrontCameraBallAvailable);

    // first sensor is omni_vision
    // check for new features from omni camera
    if (includeOmni) {
        // Take omnivision features into account
        for (auto idx = 0; idx < input.omnivision_balls_size(); ++idx) {
            auto ov_ball_feature = input.omnivision_balls(idx);
            if (ov_ball_feature.confidence() > params.ball_min_confidence()) {
                // enough confidence for using the feature
                if (nrBallsThisTime < max_balls) {
                    copy_to_ball_candidate_struct((ballData[nrBallsThisTime]), ov_ball_feature, balltype_e::OMNIVISION_b, params);
                    nrBallsThisTime++;
                }
            }
        }
    }
    // check for new features from front camera process
    if (isFrontCameraBallAvailable) {
        // Take front camera candidates into account
        for (auto idx = 0; idx < input.frontcamera_balls_size(); ++idx) {
            auto frontcam_ball_candidate = input.frontcamera_balls(idx);
            if (frontcam_ball_candidate.confidence() > params.ball_min_confidence()) {
                // enough confidence for using the candidate
                if (nrBallsThisTime < max_balls) {
                    copy_to_ball_candidate_struct((ballData[nrBallsThisTime]), frontcam_ball_candidate, balltype_e::FRONT_CAMERA_b, params);
                    nrBallsThisTime++;
                }
            }
        }
    }

    // other sensors may be added here (FUTURE)


    // administration, make sure confidence of non-updated slots is set to 0
    if (nrBallsThisTime < max_balls) {
        unsigned j = nrBallsThisTime;
        while (j < max_balls) {
            ballData[j].confidence = 0.0;
            j++;
        }
    } else {
        // we have more observations than maximum number and will limit
        nrBallsThisTime = max_balls;
    }

    return nrBallsThisTime;
}
