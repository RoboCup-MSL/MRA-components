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
#include "seq_clustering_ball_model_log.hpp"
//#include "constants_ball_model.hpp"
#include "seq_clustering_best_uid.hpp"

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

static xy_t ball_position_history[BALL_MAX_HISTORY];
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

static void add_measurement(xy_t *data, xy_t position) {
    // add new position measurement to data which contains position history
    // shift all samples one back and add new measurement
    for (int i = BALL_MAX_HISTORY - 1; i > 0; i--) {
        memcpy(&data[i], &data[i - 1], sizeof(xy_t));
    }
    memcpy(&data[0], &position, sizeof(xy_t));
}

static int calculate_velocity(xy_t *velocity, xy_t *data, long order, double sample) {
    // return differentiated value calculated based on one-sided hybrid differentiation algorithms
    // source of algorithms: http://www.holoborodko.com/pavel/wp-content/uploads/OneSidedNoiseRobustDifferentiators.pdf
    // (there are expressions up to order 15 there)
    //
    // TODO a range check should be implemented to ensure order < size of array with data
    //

    int result = 0;

    switch (order) {
    case 3:
        velocity->x = (2 * data[0].x - data[1].x - 2 * data[2].x + data[3].x) / (2 * sample);
        velocity->y = (2 * data[0].y - data[1].y - 2 * data[2].y + data[3].y) / (2 * sample);
        break;
    case 4:
        velocity->x = (7 * data[0].x + data[1].x - 10 * data[2].x - data[3].x + 3 * data[4].x) / (10 * sample);
        velocity->y = (7 * data[0].y + data[1].y - 10 * data[2].y - data[3].y + 3 * data[4].y) / (10 * sample);
        break;
    case 5:
        velocity->x = (16 * data[0].x + data[1].x - 10 * data[2].x - 10 * data[3].x - 6 * data[4].x + 9 * data[5].x)
                / (28 * sample);
        velocity->y = (16 * data[0].y + data[1].y - 10 * data[2].y - 10 * data[3].y - 6 * data[4].y + 9 * data[5].y)
                / (28 * sample);
        break;
    case 6:
        velocity->x = (12 * data[0].x + 5 * data[1].x - 8 * data[2].x - 6 * data[3].x - 10 * data[4].x + data[5].x
                + 6 * data[6].x) / (28 * sample);
        velocity->y = (12 * data[0].y + 5 * data[1].y - 8 * data[2].y - 6 * data[3].y - 10 * data[4].y + data[5].y
                + 6 * data[6].y) / (28 * sample);
        break;
    case 7:
        velocity->x = (22 * data[0].x + 7 * data[1].x - 6 * data[2].x - 11 * data[3].x - 14 * data[4].x - 9 * data[5].x
                - 2 * data[6].x + 13 * data[7].x) / (60 * sample);
        velocity->y = (22 * data[0].y + 7 * data[1].y - 6 * data[2].y - 11 * data[3].y - 14 * data[4].y - 9 * data[5].y
                - 2 * data[6].y + 13 * data[7].y) / (60 * sample);
        break;
    default:
        velocity->x = 0.0;
        velocity->y = 0.0;
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
        log_init(p_phyp + i);
        fbuf_init(p_phyp + i);
    }

    return BM_SUCCESS;
}

static int initialize_tracking(void) {

    // clear ball history
    xy_t zero_pos = {};
    for (int i = 0; i < BALL_MAX_HISTORY; i++) {
        memcpy(&ball_position_history, &zero_pos, sizeof(xy_t));
    }
//    putso(sm.ball_process.ball_vel_filter_order, BALL_VELOCITY_FILTER_ORDER);

    // tunable parameters taken from tunable_pardata_strategy_bus.h in Tech United stack
    pscgd.par.nkeep = 10;
    pscgd.par.pfactor = 100.0;
    pscgd.par.maxage = 0.5;
    pscgd.par.alpha = 1.700;
    pscgd.par.beta = 0.950;
    pscgd.par.min_allowed_sigma = 0.01; /* to prevent observer from exploding */
    pscgd.par.exp_time_free = 0.9;
    pscgd.par.exp_time_non_free = 0.3;

    // time before last seen ball is retired by reducing confidence
//    putso(sm.ball_process.ball_time_to_forget, BALL_TIME_TO_FORGET);
//    putso(sm.ball_process.confidence_decay, BALL_CONFIDENCE_DECAY);

    // initialize data structure for sequential clustering with first hypothesis
    /* tunable parameters of clustering algorithm */
    pscgd.par.nkeep = 16;
    pscgd.par.pfactor = 100.0;
    pscgd.par.maxage = 100.;

    /* initial number of hypotheses */
    pscgd.nhyp = 1;

    /* initialize hypotheses */
    init_hyp(pscgd.hyp);
    init_hyp(pscgd.hyp2);

    pscgd.new_uid = 0;

    pscgd.track_uid = INVALID_UID;

    return BM_SUCCESS;
}

int RobotsportsLocalBallTracking::RobotsportsLocalBallTracking::tick(google::protobuf::Timestamp timestamp, // absolute timestamp
        InputType const &input,       // input data, type generated from Input.proto
        ParamsType const &params,      // configuration parameters, type generated from Params.proto
        StateType &state,       // state data, type generated from State.proto
        OutputType &output,      // output data, type generated from Output.proto
        LocalType &local        // local/diagnostics data, type generated from Local.proto
        ) {
    int error_value = 0;

    MRA_LOG_TICK();

    if (not initialized) {
        initialize_tracking();
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
    auto use_naive_filter = false;
    if (use_naive_filter) {
        auto min_dist = ballData[nrBallsThisTime].dist;
        unsigned winning_idx = 0;
        // loop over measurements, starting from 2nd measurement
        for (auto idx = 1; idx < nrBallsThisTime; ++idx) {
            if (ballData[nrBallsThisTime].dist < min_dist) {
                winning_idx = idx;
            }
        }
        output.mutable_ball()->set_x(ballData[winning_idx].x);
        output.mutable_ball()->set_y(ballData[winning_idx].y);
        output.mutable_ball()->set_z(ballData[winning_idx].z);
        output.mutable_ball()->set_confidence(ballData[winning_idx].conf);
        output.mutable_ball()->set_timestamp(ballData[winning_idx].timestamp);	// timestamp based off of liveseconds

        // update speed
        auto delta_t = output.ball().timestamp() - state.ball_prev().timestamp(); //getso(sm.ball_process.out.ball_prev.ts);
        auto speed_x = 0.0;
        auto speed_y = 0.0;
        if (delta_t > 0.0) {
            // calculate speed based on displacement and difference in timestamps
            //		// use better differentiation
            xy_t xy_pos;
            xy_pos.x = output.ball().x();
            xy_pos.y = output.ball().y();
            add_measurement(ball_position_history, xy_pos);
            //double sample = getso(tasktime);	// use last sample time value as tHE sample time value, knowing we do have jitter
            xy_t speed;
            int ret = calculate_velocity(ball_position_history, &speed, params.ball_vel_filter_order(), 1 / 40.0);
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
        output.mutable_ball()->set_vx(speed_x);
        output.mutable_ball()->set_vy(speed_y);
        output.mutable_ball()->set_vz(0.0);  // TODO no airborne balls yet
        // copy position to old
        // TODO:	memcpy(&getso(sm.ball_process.out.ball_prev),getsoaddr(sm.ball_process.out.ball),sizeof(object_t));
        //	// and exit normal
    }

    if (not use_naive_filter) {
        // now run sc_bm code
        ball_estimate_t ball_estimate;
        int use_next_best_ball = 0; // if 1, then go to next best ball

        int ret = seq_clustering_ball_model(&ball_estimate, ballData, input.ts(), use_next_best_ball, &pscgd);

        if (ret == BM_SUCCESS) {
            // update ball position in world model since a successful step has been done
            output.mutable_ball()->set_x(ball_estimate.xhat); // position X, replaced TPB on 20161210 from ball_estimate.x
            output.mutable_ball()->set_y(ball_estimate.yhat); // position Y, replaced TPB on 20161210 from ball_estimate.y
            output.mutable_ball()->set_z(ball_estimate.z);             // position Z
            output.mutable_ball()->set_vx(ball_estimate.xdot);          // velocity in X
            output.mutable_ball()->set_vy(ball_estimate.ydot);          // velocity in Y
            output.mutable_ball()->set_vz(ball_estimate.zdot);          // velocity in Z
            output.mutable_ball()->set_confidence(ball_estimate.hconf);    // moving average confidence
            output.mutable_ball()->set_timestamp(ball_estimate.timestamp);     // timestamp based off of liveseconds

            double age = (input.ts() - ball_estimate.timestamp);
            if (age < 0) {
                age = 0.0;
            }

            // decay confidence
            output.mutable_ball_now()->set_confidence(ball_estimate.hconf * pow(params.confidence_decay(), age)); // degrade confidence for extrapolation based on age

            // retire when ball observation is too old
            if (age > params.ball_time_to_forget()) {
                output.mutable_ball()->set_confidence(0.0);
            }
        }

        //        if (getso(sm.ball_process.dump_sc) > 0 ) {
        //            if (getso(sm.ball_process.dump_sc) > 1) {
        //    			ret = print_hypotheses(&pscgd);
        //            }
        //            for (int j = 0; j < nrBallsThisTime; j++) {
        //            	logAlways("ball in  %2d on %6.3f %6.3f with confidence %4.2f from sensor %2d\n", j, ballData[j].x, ballData[j].y, ballData[j].conf, (int)ballData[j].type);
        //            }
        //            logAlways("ball out    on %6.3f %6.3f with confidence %4.2f\n", getso(sm.ball_process.out.ball.pos.x), getso(sm.ball_process.out.ball.pos.y), getso(sm.ball_process.out.ball.confidence));
        //            putso(sm.ball_process.dump_sc, 0);
        //        }

    }

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
        output.mutable_ball_now()->set_x(output.ball().x() + timeLeap * output.ball().vx());  // position X extrapolated
        output.mutable_ball_now()->set_y(output.ball().y() + timeLeap * output.ball().vy());  // position X extrapolated
        output.mutable_ball_now()->set_z(output.ball().z() + timeLeap * output.ball().vz());  // position X extrapolated
        output.mutable_ball_now()->set_confidence(
                output.ball().confidence() * pow(params.confidence_decay(), timeLeap));	// degrade confidence for extrapolation based on timeLeap
        output.mutable_ball_now()->set_timestamp(input.ts());     // timestamp for extrapolation is current time
    }
    output.mutable_ball_prev()->CopyFrom(state.ball_prev());

    //	std::vector<BallPositionHistoryState> ball_pos_hist;
    //	for (auto idx = 0; idx < BALL_MAX_HISTORY; ++idx)
    //    {
    //    	BallPositionHistoryState hist;
    //    	hist.set_x(ball_position_history[ball_pos_index].x);
    //    	hist.set_y(ball_position_history[ball_pos_index].y);
    //    }
    //	*state.mutable_ball_position_history() = {ball_pos_hist.begin(), ball_pos_hist.end()};
    //	state.mutable_ball() = output.mutable_ball();

    // copy output to state for the next tick
    state.mutable_ball_now()->CopyFrom(output.ball_now());
    state.mutable_ball()->CopyFrom(output.ball());

    return error_value;
}

