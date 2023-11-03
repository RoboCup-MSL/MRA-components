// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "RobotsportsLocalBallTracking.hpp"
#include "logging.hpp" // TODO: automate, perhaps via generated hpp
#include <google/protobuf/util/time_util.h>
#include <array>
#include <string>

using namespace MRA;
using namespace google::protobuf::util;

#include "seq_clustering_ball_model.hpp"
#include "seq_clustering_best_uid.hpp"
#include <vector>

// custom includes, if any
// ...

#define MAXBALLS_OV		20			/* maximum number of candidate balls found by omnivision and send to tracker
 	 	 	 	 	 	 	 	 	   NUM_BALLS defined in omni.h */
#define MAXBALLS_FC		3			/* maximum number of candidate balls found by front_cam and send to tracker */
#define MAXBALLS                        (MAXBALLS_OV+MAXBALLS_FC+2)     /* maximum number of balls send to tracker */
static ball_feature_t ballData[2 * MAXBALLS];
#define MIN_CONFIDENCE_BALL 0.05
#define MIN_HEIGHT_IN_AIR               0.2                             /* when ball is above this heigth, ball is in air */
#define BALL_MAX_HISTORY 10

typedef struct xy_struct_t {
    double x;
    double y;
} xy_t;

static std::vector<xy_t> ball_position_history(BALL_MAX_HISTORY);
static sc_global_data pscgd;
static bool initialized = false;

static void copy_to_ball_feature_struct(ball_feature_t &r_bf,
        const ::MRA::RobotsportsLocalBallTracking::BallFeature observed_ball_feature, long sensor_label) {
    r_bf.x = observed_ball_feature.x();
    r_bf.y = observed_ball_feature.y();
    r_bf.z = observed_ball_feature.z();
    r_bf.conf = observed_ball_feature.confidence();
    r_bf.dist = observed_ball_feature.dist();
    r_bf.type = sensor_label;
    r_bf.sigma = observed_ball_feature.sigma();
    r_bf.timestamp = observed_ball_feature.timestamp();
    r_bf.initializeBallVelFlag = 0;
    r_bf.initializeBallVel_xy[0] = 0.0;
    r_bf.initializeBallVel_xy[1] = 0.0;
    r_bf.inAir = observed_ball_feature.z() > MIN_HEIGHT_IN_AIR;
    r_bf.isFree = 1; /* is rolling freely (0 or 1) */
    // TODO implement check if ball is free. Disable in original
}
;

//typedef struct tag_ball_feature_t {
//        double conf;                    /* confidence */
//        double dist;                    /* distance to robot */
//        long  type;                    /* sensor label */
//        double sigma;                   /* standard deviation of sensor noise */
//        long  isFree;                  /* is rolling freely (0 or 1) */
//        long  inAir;                   /* is flying in the air (0 or 1) */
//        double timestamp;               /* timestamp */
//        long  initializeBallVelFlag;   /* initialize ball velocity if one */
//        double initializeBallVel_xy[2]; /* initialize ball velocity at kick vel_xy */
//} ball_feature_t, *pball_feature_t;
//
////				if (processBall(getso(sm.omni_process.found_balls_fc[idx]),	&ballData[nrBallsThisTime]) ) {
////					ballData[nrBallsThisTime].type = OV_b;
////					ballData[nrBallsThisTime].sigma = getso(sm.ball_process.BMsigma_OV);
////					// update distance from robot to ball for sorting
////					putso(sm.ball_process.distance_from_self[nrBallsThisTime], ballData[nrBallsThisTime].dist);
////					// increment nrBallsThisTime
////					nrBallsThisTime++;

static int calculate_velocity(xy_t& velocity, const std::vector<xy_t>& data, unsigned order, double sample_time) {
    // return differentiated value calculated based on one-sided hybrid differentiation algorithms
    // source of algorithms: http://www.holoborodko.com/pavel/wp-content/uploads/OneSidedNoiseRobustDifferentiators.pdf
    // (there are expressions up to order 15 there)
    //
    // TODO a range check should be implemented to ensure order < size of array with data
    //
    int result = 0;
    switch (order) {
    case 3:
        velocity.x = (2 * data[0].x - data[1].x - 2 * data[2].x + data[3].x) / (2 * sample_time);
        velocity.y = (2 * data[0].y - data[1].y - 2 * data[2].y + data[3].y) / (2 * sample_time);
        break;
    case 4:
        velocity.x = (7 * data[0].x + data[1].x - 10 * data[2].x - data[3].x + 3 * data[4].x) / (10 * sample_time);
        velocity.y = (7 * data[0].y + data[1].y - 10 * data[2].y - data[3].y + 3 * data[4].y) / (10 * sample_time);
        break;
    case 5:
        velocity.x = (16 * data[0].x + data[1].x - 10 * data[2].x - 10 * data[3].x - 6 * data[4].x + 9 * data[5].x)
                / (28 * sample_time);
        velocity.y = (16 * data[0].y + data[1].y - 10 * data[2].y - 10 * data[3].y - 6 * data[4].y + 9 * data[5].y)
                / (28 * sample_time);
        break;
    case 6:
        velocity.x = (12 * data[0].x + 5 * data[1].x - 8 * data[2].x - 6 * data[3].x - 10 * data[4].x + data[5].x
                + 6 * data[6].x) / (28 * sample_time);
        velocity.y = (12 * data[0].y + 5 * data[1].y - 8 * data[2].y - 6 * data[3].y - 10 * data[4].y + data[5].y
                + 6 * data[6].y) / (28 * sample_time);
        break;
    case 7:
        velocity.x = (22 * data[0].x + 7 * data[1].x - 6 * data[2].x - 11 * data[3].x - 14 * data[4].x - 9 * data[5].x
                - 2 * data[6].x + 13 * data[7].x) / (60 * sample_time);
        velocity.y = (22 * data[0].y + 7 * data[1].y - 6 * data[2].y - 11 * data[3].y - 14 * data[4].y - 9 * data[5].y
                - 2 * data[6].y + 13 * data[7].y) / (60 * sample_time);
        break;
    default:
        velocity.x = 0.0;
        velocity.y = 0.0;
        result = 1;
    }
    return (result); // result is 0 if a proper deriative has been calculated, 1 otherwise
}


static int fbuf_init(hypothesis* phyp)
{
        /* clear feature buf */
        memset(&(phyp->fbuf), 0, sizeof(featbuf_t));

        /* no valid features yet */
        phyp->nfbuf = 0;

        /* start position in buffer */
        phyp->fbuf_idx = 0;

        return BM_SUCCESS;
}

static int ma_init(hypothesis* phyp)
{
        int i;

        phyp->ma_first = 1;

        phyp->ma_idx = MA_N;
        if (phyp->ma_idx >= MA_N + 1) {
                phyp->ma_idx = phyp->ma_idx - (MA_N + 1);
        }

        /* initialize buffer */
        for (i = 0; i < MA_N + 1; i++) {
               phyp->ma_buf[i] = 0.01;
        }

        phyp->mavg = 0.01;

        return BM_SUCCESS;
}


static int init_hyp(hypothesis* p_phyp) {
    /* initialize hypotheses */
    for (int i = 0; i < MAXHYP; i++) {
        (p_phyp + i)->nobj = 0;
        (p_phyp + i)->p = 1.0;
        ma_init(p_phyp + i);
        fbuf_init(p_phyp + i);
    }

    return BM_SUCCESS;
}

static int initialize_tracking(MRA::RobotsportsLocalBallTracking::ParamsType const &params) {
    /* initial number of hypotheses */
    pscgd.nhyp = 1;

    /* initialize hypotheses */
    init_hyp(pscgd.hyp);
    init_hyp(pscgd.hyp2);

    pscgd.new_uid = 0;

    pscgd.track_uid = INVALID_UID;

    return BM_SUCCESS;
}

static void calculate_ball_now(const MRA::RobotsportsLocalBallTracking::InputType &input,
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

int RobotsportsLocalBallTracking::RobotsportsLocalBallTracking::tick(google::protobuf::Timestamp timestamp, // absolute timestamp
        InputType const &input,       // input data, type generated from Input.proto
        ParamsType const &params,      // configuration parameters, type generated from Params.proto
        StateType &state,       // state data, type generated from State.proto
        OutputType &output,      // output data, type generated from Output.proto
        LocalType &local        // local/diagnostics data, type generated from state.proto
        ) {
    int error_value = 0;

    MRA_LOG_TICK();

    if (not initialized) {
        initialize_tracking(params);
        initialized = true;
    }

    //    xy_t ball_position_history[BALL_MAX_HISTORY];
    //    auto ball_pos_index = 0;
    //    for (auto ball_pos: state.ball_position_history()) {
    //    	if  (ball_pos_index < BALL_MAX_HISTORY) {
    //        	ball_position_history[ball_pos_index].x = ball_pos.x();
    //        	ball_position_history[ball_pos_index].y = ball_pos.y();
    //    	}
    //    	ball_pos_index++;
    //    }

    //    for (auto idx = 0; idx < state.BALL_MAX_HISTORY)
    state.mutable_ball_prev()->CopyFrom(state.ball());

    // fill measurements just like balltrackpreproc.c does in Turtle2 code
    int nrBallsThisTime = 0;
    //	ball_estimate_t ball_estimate;    // TODO: use or remove

    // provide proper init for (static) ballData array; sc_bm will scan the whole list and assume all data with conf > 0.0 to be valid measurements
    ballData[0].conf = 0.0;

    bool isStereoBallAvailable = input.stereovision_balls_size() > 0;

    const bool suppressOmni = params.suppress_omni();
    const bool includeOmni = !suppressOmni || (!isStereoBallAvailable);

    // first sensor is omni_vision
    std::vector < BallFeature > filtered_ball_features = std::vector<BallFeature>();
    // check for new features from omni camera
    if (includeOmni) {
        // Take omnivision features into account
        for (auto idx = 0; idx < input.omnivision_balls_size(); ++idx) {
            auto ov_ball_feature = input.omnivision_balls(idx);
            if (ov_ball_feature.confidence() > MIN_CONFIDENCE_BALL) {
                // enough confidence for using the feature
                if (nrBallsThisTime < MAXBALLS) {
                    copy_to_ball_feature_struct(ballData[nrBallsThisTime], ov_ball_feature, balltype_e::OV_b);
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
            if (stereo_ball_feature.confidence() > MIN_CONFIDENCE_BALL) {
                // enough confidence for using the feature
                if (nrBallsThisTime < MAXBALLS) {
                    copy_to_ball_feature_struct(ballData[nrBallsThisTime], stereo_ball_feature, balltype_e::STEREO_b);
                    nrBallsThisTime++;
                }
            }
        }
    }

    // other sensors may be added

    // administration, make sure confidence of non-updated slots is set to 0
    if (nrBallsThisTime < MAXBALLS) {
        int j = nrBallsThisTime;
        while (j < MAXBALLS) {
            ballData[j].conf = 0.0;
            j++;
        }
    } else {
        // we have more observations than maximum number and will limit
        nrBallsThisTime = MAXBALLS;
    }

    // update administration in so for debugging => TODO write to local
    //	putso(sm.ball_process.nr_balls, nrBallsThisTime);
    // other sensors may be added

    // administration, make sure confidence of non-updated slots is set to 0
    if (nrBallsThisTime < MAXBALLS) {
        int j = nrBallsThisTime;
        while (j < MAXBALLS) {
            ballData[j].conf = 0.0;
            j++;
        }
    } else {
        // we have more observations than maximum number and will limit
        nrBallsThisTime = MAXBALLS;
    }

    // update administration in so for debugging
    // TODO Local: putso(sm.ball_process.nr_balls, nrBallsThisTime);

    if (nrBallsThisTime == 0) {
        // nothing to do; better spend time more wisely
        return error_value;
    }

    // naive filter - select closest measurement
    // find close

    if (params.run_native_filter()) {

        // update previous ball
        state.mutable_native_filter()->mutable_ball_prev()->CopyFrom(state.native_filter().ball());


        auto min_dist = ballData[nrBallsThisTime].dist;
        unsigned winning_idx = 0;
        // loop over measurements, starting from 2nd measurement
        for (auto idx = 1; idx < nrBallsThisTime; ++idx) {
            if (ballData[nrBallsThisTime].dist < min_dist) {
                winning_idx = idx;
            }
        }
        state.mutable_native_filter()->mutable_ball()->set_x(ballData[winning_idx].x);
        state.mutable_native_filter()->mutable_ball()->set_y(ballData[winning_idx].y);
        state.mutable_native_filter()->mutable_ball()->set_z(ballData[winning_idx].z);
        state.mutable_native_filter()->mutable_ball()->set_confidence(ballData[winning_idx].conf);
        state.mutable_native_filter()->mutable_ball()->set_timestamp(ballData[winning_idx].timestamp);	// timestamp based off of liveseconds

        // update speed
        auto delta_t = state.native_filter().ball().timestamp() - state.native_filter().ball_prev().timestamp(); //getso(sm.ball_process.out.ball_prev.ts);
        auto speed_x = 0.0;
        auto speed_y = 0.0;
        if (delta_t > 0.0) {
            // calculate speed based on displacement and difference in timestamps
            //		// use better differentiation
            xy_t xy_pos;
            xy_pos.x = output.ball().x();
            xy_pos.y = output.ball().y();

            ball_position_history.pop_back();
            ball_position_history.insert(ball_position_history.begin(),xy_pos);
            //double sample = getso(tasktime);	// use last sample time value as tHE sample time value, knowing we do have jitter
            xy_t speed = {0, 0};
            int ret = calculate_velocity(speed, ball_position_history, params.ball_vel_filter_order(), 1 / 40.0);
            if (ret == 0) {
                // valid update of speed, replace simple backward difference with improved velocity numbers
                speed_x = speed.x;
                speed_y = speed.y;
            } else {
                // filter failed, use linear estimation
                speed_x = (output.ball().x() - state.ball_prev().x()) / delta_t;
                speed_y = (output.ball().y() - state.ball_prev().y()) / delta_t;
            }
        } else {
            // write 0 because no better alternative exists
            // alternatively, the previous speed can be maintained, with scaling to zero?
            // the latter option is now implemented; writing 0 causes a lot of noise
            speed_x = state.ball_prev().vx();
            speed_y = state.ball_prev().vy();
        }
        state.mutable_native_filter()->mutable_ball()->set_vx(speed_x);
        state.mutable_native_filter()->mutable_ball()->set_vy(speed_y);
        state.mutable_native_filter()->mutable_ball()->set_vz(0.0);  // TODO no airborne balls yet

        // Calculate ball_now() for this filter
        calculate_ball_now(input, params, *(state.mutable_native_filter()));
    }

    if (params.run_sequential_clustering_filter()) {
        // now run sc_bm code
        ball_estimate_t ball_estimate;
        int use_next_best_ball = 0; // if 1, then go to next best ball

        int ret = seq_clustering_ball_model(&ball_estimate, ballData, input.ts(), use_next_best_ball, &pscgd, params);

        if (ret == BM_SUCCESS) {
            // update ball position in world model since a successful step has been done

            // update previous ball
            state.mutable_sequence_filter()->mutable_ball_prev()->CopyFrom(state.sequence_filter().ball());

            state.mutable_sequence_filter()->mutable_ball()->set_x(ball_estimate.xhat); // position X, replaced TPB on 20161210 from ball_estimate.x
            state.mutable_sequence_filter()->mutable_ball()->set_y(ball_estimate.yhat); // position Y, replaced TPB on 20161210 from ball_estimate.y
            state.mutable_sequence_filter()->mutable_ball()->set_z(ball_estimate.z);             // position Z
            state.mutable_sequence_filter()->mutable_ball()->set_vx(ball_estimate.xdot);          // velocity in X
            state.mutable_sequence_filter()->mutable_ball()->set_vy(ball_estimate.ydot);          // velocity in Y
            state.mutable_sequence_filter()->mutable_ball()->set_vz(ball_estimate.zdot);          // velocity in Z
            state.mutable_sequence_filter()->mutable_ball()->set_confidence(ball_estimate.hconf);    // moving average confidence
            state.mutable_sequence_filter()->mutable_ball()->set_timestamp(ball_estimate.timestamp);     // timestamp based off of liveseconds

            double age = (input.ts() - ball_estimate.timestamp);
            if (age < 0) {
                age = 0.0;
            }

            // decay confidence
            state.mutable_sequence_filter()->mutable_ball_now()->set_confidence(ball_estimate.hconf * pow(params.confidence_decay(), age)); // degrade confidence for extrapolation based on age

            // retire when ball observation is too old
            if (age > params.ball_time_to_forget()) {
                state.mutable_sequence_filter()->mutable_ball()->set_confidence(0.0);
            }

            // Calculate ball_now() for this filter
            calculate_ball_now(input, params, *(state.mutable_sequence_filter()));
        }

    }

    // Check usage : Seems not in use outside ball_process

    // TODO timestamp is updated when copy from ball_new to ball
    // TODO ball_prev is not updated, it seems - stays at 0
    // calculate ball position at current time, based on position with timestamp ts and estimate of ball speed

    if (params.ball_filter() == 0) {
        // use native_filter
        output.mutable_ball_prev()->CopyFrom(state.native_filter().ball_prev());
        output.mutable_ball()->CopyFrom(state.native_filter().ball());
        output.mutable_ball_now()->CopyFrom(state.native_filter().ball_now());
    }
    else {
        // use sequential_clustering_filter
        output.mutable_ball_prev()->CopyFrom(state.sequence_filter().ball_prev());
        output.mutable_ball()->CopyFrom(state.sequence_filter().ball());
        output.mutable_ball_now()->CopyFrom(state.sequence_filter().ball_now());
    }

    // copy to state
    state.mutable_ball_prev()->CopyFrom(output.ball_prev());
    state.mutable_ball()->CopyFrom(output.ball());
    state.mutable_ball_now()->CopyFrom(output.ball_now());

    local.mutable_native_filter()->CopyFrom(state.native_filter());
    local.mutable_sequence_filter()->CopyFrom(state.sequence_filter());

    //	std::vector<BallPositionHistoryState> ball_pos_hist;
    //	for (auto idx = 0; idx < BALL_MAX_HISTORY; ++idx)
    //    {
    //    	BallPositionHistoryState hist;
    //    	hist.set_x(ball_position_history[ball_pos_index].x);
    //    	hist.set_y(ball_position_history[ball_pos_index].y);
    //    }
    //	*state.mutable_ball_position_history() = {ball_pos_hist.begin(), ball_pos_hist.end()};


    return error_value;
}

