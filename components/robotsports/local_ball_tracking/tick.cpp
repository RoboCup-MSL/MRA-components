// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "RobotsportsLocalBallTracking.hpp"
#include <google/protobuf/util/time_util.h>
#include <array>
#include <string>

using namespace MRA;
using namespace google::protobuf::util;

#include "./internal/RobotsportsLocalBallTracking_sequence_clustering.hpp"
#include <vector>
#include "logging.hpp" // TODO: automate, perhaps via generated hpp


const unsigned MAXBALLS_OV = 20; /* maximum number of candidate balls found by omnivision and send to tracker NUM_BALLS defined in omni.h */
const unsigned MAXBALLS_FC = 3;  /* maximum number of candidate balls found by front_cam and send to tracker */
const unsigned MAXBALLS = MAXBALLS_OV+MAXBALLS_FC+2;  /* maximum number of balls send to tracker */



int RobotsportsLocalBallTracking::RobotsportsLocalBallTracking::tick(google::protobuf::Timestamp timestamp, // absolute timestamp
        InputType const &input,       // input data, type generated from Input.proto
        ParamsType const &params,      // configuration parameters, type generated from Params.proto
        StateType &state,       // state data, type generated from State.proto
        OutputType &output,      // output data, type generated from Output.proto
        LocalType &local        // local/diagnostics data, type generated from state.proto
        ) {
    int error_value = 0;

    MRA_LOG_TICK();


    if (input.candidates_size() == 0) {
        // nothing to do; better spend time more wisely
        return error_value;
    }

    std::vector<ball_candidate_t> ballData;
    for (auto idx = 0; idx < input.candidates_size(); ++idx) {
        auto input_candidate = MRA::Datatypes::BallCandidate();
        input_candidate.CopyFrom(input.candidates(idx));
        ball_candidate_t candidate;
        candidate.x = input_candidate.measured_pose_fcs().x();
        candidate.y = input_candidate.measured_pose_fcs().y();
        candidate.z = input_candidate.measured_pose_fcs().z();
        candidate.confidence = input_candidate.confidence();
        candidate.type  = input_candidate.source();
        candidate.sigma  = input_candidate.sigma();
        candidate.is_free =  input_candidate.is_free();
        candidate.in_air =  input_candidate.in_air();
        candidate.timestamp = (google::protobuf::util::TimeUtil::TimestampToMilliseconds(input_candidate.timestamp()) / 1000.0);
        ballData.push_back(candidate);
    }

    // now run sc_bm code
    double timestamp_as_double = google::protobuf::util::TimeUtil::TimestampToMilliseconds(timestamp) / 1000.0;
    local_ball_tracking_sequence_clustering(timestamp_as_double, ballData,  input, params, output, state, MAXBALLS);


    return error_value;
}

