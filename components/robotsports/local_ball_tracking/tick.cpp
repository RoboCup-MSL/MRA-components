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

#include "RobotsportsLocalBallTracking_preprocessing.hpp"
#include "RobotsportsLocalBallTracking_native_filter.hpp"
#include "RobotsportsLocalBallTracking_sequence_clustering.hpp"
#include <vector>

// custom includes, if any
// ...

#define MAXBALLS_OV		20			/* maximum number of candidate balls found by omnivision and send to tracker NUM_BALLS defined in omni.h */
#define MAXBALLS_FC		3			/* maximum number of candidate balls found by front_cam and send to tracker */
#define MAXBALLS  (MAXBALLS_OV+MAXBALLS_FC+2)     /* maximum number of balls send to tracker */
static std::vector<ball_feature_t> ballData(2 * MAXBALLS);

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

int RobotsportsLocalBallTracking::RobotsportsLocalBallTracking::tick(google::protobuf::Timestamp timestamp, // absolute timestamp
        InputType const &input,       // input data, type generated from Input.proto
        ParamsType const &params,      // configuration parameters, type generated from Params.proto
        StateType &state,       // state data, type generated from State.proto
        OutputType &output,      // output data, type generated from Output.proto
        LocalType &local        // local/diagnostics data, type generated from state.proto
        ) {
    int error_value = 0;

    MRA_LOG_TICK();

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
    int nrBallsThisTime = local_ball_tracking_preprocessing(ballData, input, params);
    MRA_LOG_DEBUG("> tick::local_ball_tracking_preprocessing %d", nrBallsThisTime);

    if (nrBallsThisTime == 0) {
        // nothing to do; better spend time more wisely
        return error_value;
    }


    if (params.run_native_filter()) {
        // naive filter - select closest measurement
        // find close
        // update previous ball
        local_ball_tracking_native_filter(nrBallsThisTime, ballData, params, input, state, output);
    }

    if (params.run_sequential_clustering_filter()) {
        // now run sc_bm code
        local_ball_tracking_sequence_clustering(nrBallsThisTime, ballData, input, params, state, MAXBALLS);
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

