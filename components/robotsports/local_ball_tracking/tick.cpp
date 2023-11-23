// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "RobotsportsLocalBallTracking.hpp"
#include <google/protobuf/util/time_util.h>
#include <array>
#include <string>

using namespace MRA;
using namespace google::protobuf::util;

#include "./internal/RobotsportsLocalBallTracking_preprocessing.hpp"
#include "./internal/RobotsportsLocalBallTracking_sequence_clustering.hpp"
#include <vector>
#include "logging.hpp" // TODO: automate, perhaps via generated hpp


const unsigned MAXBALLS_OV = 20; /* maximum number of candidate balls found by omnivision and send to tracker NUM_BALLS defined in omni.h */
const unsigned MAXBALLS_FC = 3;  /* maximum number of candidate balls found by front_cam and send to tracker */
const unsigned MAXBALLS = MAXBALLS_OV+MAXBALLS_FC+2;  /* maximum number of balls send to tracker */

static std::vector<ball_candidate_t> ballData(2 * MAXBALLS);

int RobotsportsLocalBallTracking::RobotsportsLocalBallTracking::tick(google::protobuf::Timestamp timestamp, // absolute timestamp
        InputType const &input,       // input data, type generated from Input.proto
        ParamsType const &params,      // configuration parameters, type generated from Params.proto
        StateType &state,       // state data, type generated from State.proto
        OutputType &output,      // output data, type generated from Output.proto
        LocalType &local        // local/diagnostics data, type generated from state.proto
        ) {
    int error_value = 0;

    MRA_LOG_TICK();

    // pre process the measurements: only keep interesting measurements
    unsigned nrBallsThisTime = local_ball_tracking_preprocessing(ballData, input, params);
    MRA_LOG_DEBUG("> tick::local_ball_tracking_preprocessing %d", nrBallsThisTime);

    if (nrBallsThisTime == 0) {
        // nothing to do; better spend time more wisely
        return error_value;
    }

    // now run sc_bm code
    double timestamp_as_double = google::protobuf::util::TimeUtil::TimestampToMilliseconds(timestamp) / 1000.0;
    local_ball_tracking_sequence_clustering(timestamp_as_double, nrBallsThisTime, ballData,  input, params, output, state, MAXBALLS);


    return error_value;
}

