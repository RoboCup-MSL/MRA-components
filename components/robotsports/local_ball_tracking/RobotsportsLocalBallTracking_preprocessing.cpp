#include "RobotsportsLocalBallTracking_preprocessing.hpp"


void local_ball_tracking_calculate_ball_now(const MRA::RobotsportsLocalBallTracking::InputType &input,
                               const MRA::RobotsportsLocalBallTracking::ParamsType &params,
                               MRA::RobotsportsLocalBallTracking::OutputType &output) {
    // Calculate ball_now()
    // Check usage : Seems not in use outside ball_process
    // TODO timestamp is updated when copy from ball_new to ball
    // TODO ball_prev is not updated, it seems - stays at 0
    // calculate ball position at current time, based on position with timestamp ts and estimate of ball speed
    double timeLeap = input.ts() - output.ball().timestamp();
    // copy ball position to _now position
    output.mutable_ball_now()->CopyFrom(output.ball());
    if (timeLeap > 0) {
        // current time is actually larger than ball observation, so we can extrapolate for _now position
        output.mutable_ball_now()->set_x(output.ball().x() + timeLeap * output.ball().vx()); // position X extrapolated
        output.mutable_ball_now()->set_y(output.ball().y() + timeLeap * output.ball().vy()); // position X extrapolated
        output.mutable_ball_now()->set_z(output.ball().z() + timeLeap * output.ball().vz()); // position X extrapolated
        output.mutable_ball_now()->set_confidence(
                output.ball().confidence() * pow(params.confidence_decay(), timeLeap)); // degrade confidence for extrapolation based on timeLeap
        output.mutable_ball_now()->set_timestamp(input.ts()); // timestamp for extrapolation is current time
    }
}


static void copy_to_ball_feature_struct(ball_feature_t &r_bf,
        const ::MRA::RobotsportsLocalBallTracking::BallFeature observed_ball_feature,
        balltype_e ball_type,
        const MRA::RobotsportsLocalBallTracking::ParamsType &params) {
    r_bf.x = observed_ball_feature.x();
    r_bf.y = observed_ball_feature.y();
    r_bf.z = observed_ball_feature.z();
    r_bf.conf = observed_ball_feature.confidence();
    r_bf.dist = observed_ball_feature.dist();
    r_bf.type = ball_type;
    r_bf.sigma = observed_ball_feature.sigma();
    r_bf.timestamp = observed_ball_feature.timestamp();
    r_bf.initializeBallVelFlag = 0;
    r_bf.initializeBallVel_xy[0] = 0.0;
    r_bf.initializeBallVel_xy[1] = 0.0;
    r_bf.inAir = observed_ball_feature.z() > params.min_height_in_air();
    r_bf.isFree = 1; /* is rolling freely (0 or 1) */   // TODO implement check if ball is free. Disable in original
};


int local_ball_tracking_preprocessing(std::vector<ball_feature_t>& ballData,
                                      const MRA::RobotsportsLocalBallTracking::InputType &input,
                                      const MRA::RobotsportsLocalBallTracking::ParamsType &params) {

    // fill measurements
    unsigned nrBallsThisTime = 0;
    unsigned max_balls = ballData.size();

    bool isStereoBallAvailable = input.stereovision_balls_size() > 0;
    const bool suppressOmni = params.suppress_omni();
    const bool includeOmni = !suppressOmni || (!isStereoBallAvailable);

    // first sensor is omni_vision
    // check for new features from omni camera
    if (includeOmni) {
        // Take omnivision features into account
        for (auto idx = 0; idx < input.omnivision_balls_size(); ++idx) {
            auto ov_ball_feature = input.omnivision_balls(idx);
            if (ov_ball_feature.confidence() > params.ball_min_confidence()) {
                // enough confidence for using the feature
                if (nrBallsThisTime < max_balls) {
                    copy_to_ball_feature_struct((ballData[nrBallsThisTime]), ov_ball_feature, balltype_e::OV_b, params);
                    nrBallsThisTime++;
                }
            }
        }
    }
    // check for new features from stereo process
    if (isStereoBallAvailable) {
        // Take stereo vision features into account
        for (auto idx = 0; idx < input.stereovision_balls_size(); ++idx) {
            auto stereo_ball_feature = input.stereovision_balls(idx);
            if (stereo_ball_feature.confidence() > params.ball_min_confidence()) {
                // enough confidence for using the feature
                if (nrBallsThisTime < max_balls) {
                    copy_to_ball_feature_struct((ballData[nrBallsThisTime]), stereo_ball_feature, balltype_e::STEREO_b, params);
                    nrBallsThisTime++;
                }
            }
        }
    }

    // other sensors may be added here


    // administration, make sure confidence of non-updated slots is set to 0
    if (nrBallsThisTime < max_balls) {
        unsigned j = nrBallsThisTime;
        while (j < max_balls) {
            ballData[j].conf = 0.0;
            j++;
        }
    } else {
        // we have more observations than maximum number and will limit
        nrBallsThisTime = max_balls;
    }

    return nrBallsThisTime;
}
