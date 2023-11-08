#include "RobotsportsLocalBallTracking_preprocessing.hpp"

#define BALL_MAX_HISTORY 10  // TODO from params ?


typedef struct xy_struct_t {
    double x;
    double y;
} xy_t;

static std::vector<xy_t> ball_position_history;
static bool initialized = false;

static int calculate_velocity(xy_t& velocity, const std::vector<xy_t>& data, unsigned order, double sample_time) {
    // return differentiated value calculated based on one-sided hybrid differentiation algorithms
    // source of algorithms: http://www.holoborodko.com/pavel/wp-content/uploads/OneSidedNoiseRobustDifferentiators.pdf
    //
    // return: 0 if velocity could be calculated, otherwise error occurred
    //
    int result = 0;

    MRA_LOG_INFO("MF calc_vel line: %d : order: %d sample_time %g data-size: %d", __LINE__, order, sample_time, data.size());
    for (auto idx = 0u; idx < ball_position_history.size(); ++idx) {
        MRA_LOG_INFO("MF calc_vel line: %d : idx: %d x: %g y: %g", __LINE__, idx, ball_position_history[idx].x, ball_position_history[idx].y);
    }


    if (order >= data.size()) {
        // size of array with data must be big enough for the requested order
        velocity.x = 0.0;
        velocity.y = 0.0;
        result = 1;
    }
    else {
        switch (order) {
        case 3:
            // 3th order:  1/2h (2 fi − fi−1 − 2 fi−2 + fi−3)
            velocity.x = (2 * data[0].x - data[1].x - 2 * data[2].x + data[3].x) / (2 * sample_time);
            velocity.y = (2 * data[0].y - data[1].y - 2 * data[2].y + data[3].y) / (2 * sample_time);
            break;
        case 4:
            // 4th order:  1/10h (7 fi + fi−1 − 10 fi−2 − fi−3 + 3 fi−4)
            velocity.x = (7 * data[0].x + data[1].x - 10 * data[2].x - data[3].x + 3 * data[4].x) / (10 * sample_time);
            velocity.y = (7 * data[0].y + data[1].y - 10 * data[2].y - data[3].y + 3 * data[4].y) / (10 * sample_time);
            break;
        case 5:
            // 5th order: 1/28h (16 fi + fi−1 − 10 fi−2 − 10 fi−3 − 6 fi−4 + 9 fi−5)
            velocity.x = (16 * data[0].x + data[1].x - 10 * data[2].x - 10 * data[3].x - 6 * data[4].x + 9 * data[5].x)
                    / (28 * sample_time);
            velocity.y = (16 * data[0].y + data[1].y - 10 * data[2].y - 10 * data[3].y - 6 * data[4].y + 9 * data[5].y)
                    / (28 * sample_time);
            break;
        case 6:
            // 6th order:  1/28h (12 fi + 5 fi−1 − 8 fi−2 − 6 fi−3 − 10 fi−4 + fi−5 + 6 fi−6)
            velocity.x = (12 * data[0].x + 5 * data[1].x - 8 * data[2].x - 6 * data[3].x - 10 * data[4].x + data[5].x
                    + 6 * data[6].x) / (28 * sample_time);
            velocity.y = (12 * data[0].y + 5 * data[1].y - 8 * data[2].y - 6 * data[3].y - 10 * data[4].y + data[5].y
                    + 6 * data[6].y) / (28 * sample_time);
            break;
        case 7:
            // 7th order:   1/60h (22 fi + 7 fi−1 − 6 fi−2 − 11 fi−3 − 14 fi−4 − 9 fi−5 − 2 fi−6 + 13 fi−7)
            velocity.x = (22 * data[0].x + 7 * data[1].x - 6 * data[2].x - 11 * data[3].x - 14 * data[4].x - 9 * data[5].x
                    - 2 * data[6].x + 13 * data[7].x) / (60 * sample_time);
            velocity.y = (22 * data[0].y + 7 * data[1].y - 6 * data[2].y - 11 * data[3].y - 14 * data[4].y - 9 * data[5].y
                    - 2 * data[6].y + 13 * data[7].y) / (60 * sample_time);
            break;
        case 8:
            // 8th order:  1/180h (52 fi + 29 fi−1 − 14 fi−2 − 17 fi−3 − 40 fi−4 −23 fi−5 − 26 fi−6 + 11 fi−7 + 28 fi−8)
            velocity.x = (52 * data[0].x + 29 * data[1].x - 14 * data[2].x - 17 * data[3].x - 40 * data[4].x - 23 * data[5].x
                    - 26 * data[6].x + 11 * data[7].x + 28 * data[8].x) / (180 * sample_time);
            velocity.y = (52 * data[0].y + 29 * data[1].y - 14 * data[2].y - 17 * data[3].y - 40 * data[4].y - 23 * data[5].y
                    - 26 * data[6].y + 11 * data[7].y + 28 * data[8].y) / (180 * sample_time);
            break;
        case 9:
            // 9th order: 1/220h (56 fi + 26 fi−1 − 2 fi−2 − 17 fi−3 − 30 fi−4 −30 fi−5 − 28 fi−6 − 13 fi−7 + 4 fi−8 + 34 fi−9)
            velocity.x = (56 * data[0].x + 26 * data[1].x - 2 * data[2].x - 17 * data[3].x - 30 * data[4].x - 30 * data[5].x
                    - 28 * data[6].x -13 * data[7].x + 4 * data[8].x + 34 * data[9].x) / (220 * sample_time);
            velocity.y = (56 * data[0].y + 26 * data[1].y - 2 * data[2].y - 17 * data[3].y - 30 * data[4].y - 30 * data[5].y
                    - 28 * data[6].y -13 * data[7].y + 4 * data[8].y + 34 * data[9].y) / (220 * sample_time);
            break;
        case 10:
            // 10th order: 1/1540h (320 fi + 206 fi−1 − 8 fi−2 − 47 fi−3 − 186 fi−4 −150 fi−5 − 214 fi−6 − 103 fi−7 − 92 fi−8 + 94 fi−9 + 180 fi−10)
            velocity.x = (320 * data[0].x + 206 * data[1].x - 8 * data[2].x - 47 * data[3].x - 186 * data[4].x - 150 * data[5].x
                    - 214 * data[6].x -103 * data[7].x -92 * data[8].x + 94 * data[9].x + 180 * data[10].x) / (1540 * sample_time);
            velocity.y = (320 * data[0].y + 206 * data[1].y - 8 * data[2].y - 47 * data[3].y - 186 * data[4].y - 150 * data[5].y
                    - 214 * data[6].y -103 * data[7].y -92 * data[8].y + 94 * data[9].y + 180 * data[10].y) / (1540 * sample_time);
            break;
        case 15:
            // 15th order: 1/2856h (322 fi + 217 fi−1 + 110 fi−2 + 35 fi−3 − 42 fi−4 −87 fi−5 − 134 fi−6 − 149 fi−7 − 166 fi−8 − 151 fi−9
            //                      −138 fi−10 − 93 fi−11 − 50 fi−12 + 25 fi−13 + 98 fi−14 + 203 fi−15)
            velocity.x = (322 * data[0].x + 217 * data[1].x + 110 * data[2].x + 35 * data[3].x - 42 * data[4].x
                    - 87 * data[5].x - 134 * data[6].x - 149 * data[7].x - 166 * data[8].x - 151 * data[9].x
                    - 138 * data[10].x - 93 * data[11].x - 50 * data[12].x + 25 * data[13].x + 98 * data[14].x
                    + 203 * data[15].x) / (2856 * sample_time);
            velocity.y = (322 * data[0].y + 217 * data[1].y + 110 * data[2].y + 35 * data[3].y - 42 * data[4].y
                    - 87 * data[5].y - 134 * data[6].y - 149 * data[7].y - 166 * data[8].y - 151 * data[9].y
                    - 138 * data[10].y - 93 * data[11].y - 50 * data[12].y + 25 * data[13].y + 98 * data[14].y
                    + 203 * data[15].y) / (2856 * sample_time);
            break;
        default:
            // order without formula
            velocity.x = 0.0;
            velocity.y = 0.0;
            result = 1;
        }
    }
    return (result); // result is 0 if a proper deriative has been calculated, 1 otherwise
}


void local_ball_tracking_native_filter(unsigned nrBallsThisTime,
                                    const std::vector<ball_feature_t>& ballData,
                                        const MRA::RobotsportsLocalBallTracking::ParamsType &params,
                                        const MRA::RobotsportsLocalBallTracking::InputType &input,
                                        MRA::RobotsportsLocalBallTracking::StateType &state)
{
    if (not initialized) {
        initialized = true;
        state.mutable_native_filter()->mutable_ball()->set_x(0.0);
        state.mutable_native_filter()->mutable_ball()->set_y(0.0);
        state.mutable_native_filter()->mutable_ball()->set_z(0.0);
        state.mutable_native_filter()->mutable_ball()->set_vx(0.0);
        state.mutable_native_filter()->mutable_ball()->set_vy(0.0);
        state.mutable_native_filter()->mutable_ball()->set_vz(0.0);
        state.mutable_native_filter()->mutable_ball()->set_confidence(0.0);
        state.mutable_native_filter()->mutable_ball()->set_timestamp(0.0);
    }
    // update previous ball
    state.mutable_native_filter()->mutable_ball_prev()->CopyFrom(state.native_filter().ball());
    MRA_LOG_INFO("MF line: %d : %s", __LINE__, MRA::convert_proto_to_json_str(state.native_filter()).c_str());

    auto min_dist = ballData[nrBallsThisTime].dist;
    unsigned winning_idx = 0;
    // loop over measurements, starting from 2nd measurement
    for (unsigned idx = 1; idx < nrBallsThisTime; ++idx) {
        if (ballData[nrBallsThisTime].dist < min_dist) {
            winning_idx = idx;
        }
    }
    state.mutable_native_filter()->mutable_ball()->set_x(ballData[winning_idx].x);
    state.mutable_native_filter()->mutable_ball()->set_y(ballData[winning_idx].y);
    state.mutable_native_filter()->mutable_ball()->set_z(ballData[winning_idx].z);
    state.mutable_native_filter()->mutable_ball()->set_confidence(ballData[winning_idx].conf);
    state.mutable_native_filter()->mutable_ball()->set_timestamp(ballData[winning_idx].timestamp);
    MRA_LOG_INFO("MF line: %d : %s", __LINE__, MRA::convert_proto_to_json_str(state.native_filter()).c_str());

    // update speed
    auto delta_t = state.native_filter().ball().timestamp() - state.native_filter().ball_prev().timestamp();
    MRA_LOG_INFO("MF line: %d : delta_t: %6.4f", __LINE__, delta_t);
    auto speed_x = 0.0;
    auto speed_y = 0.0;
    if (delta_t > 0.0) {
        MRA_LOG_INFO("MF line: %d : delta_t: %6.4f > 0", __LINE__, delta_t);
        // calculate speed based on displacement and difference in timestamps
        // use better differentiation
        xy_t xy_pos;
        xy_pos.x = state.native_filter().ball().x();
        xy_pos.y = state.native_filter().ball().y();

        if (ball_position_history.size() >= BALL_MAX_HISTORY) {
            // remove first added position when BALL_MAX_HISTORY or more measurements are handled
            ball_position_history.pop_back();
        }
        ball_position_history.insert(ball_position_history.begin(), xy_pos);
        //TODO use last sample time value as the sample time value, knowing we do have jitter
        xy_t speed = { 0, 0 };
        int ret = calculate_velocity(speed, ball_position_history, params.ball_vel_filter_order(), delta_t);
        if (ret == 0) {
            // valid update of speed, replace simple backward difference with improved velocity numbers
            speed_x = speed.x;
            speed_y = speed.y;
            MRA_LOG_INFO("MF line: %d : speed x: %6.4f y: %6.4f", __LINE__, speed_x , speed_y);
        } else {
            // filter failed, use linear estimation
            speed_x = (state.native_filter().ball().x() - state.native_filter().ball_prev().x()) / delta_t;
            speed_y = (state.native_filter().ball().y() - state.native_filter().ball_prev().y()) / delta_t;
            MRA_LOG_INFO("MF line: %d : speed x: %6.4f y: %6.4f", __LINE__, speed_x , speed_y);
        }
    } else {
        MRA_LOG_INFO("MF line: %d : delta_t: %6.4f <= 0", __LINE__, delta_t);
        // write 0 because no better alternative exists
        // alternatively, the previous speed can be maintained, with scaling to zero?
        // the latter option is now implemented; writing 0 causes a lot of noise
        if (state.ball_prev().confidence() > 0.001) {
            speed_x = state.ball_prev().vx();
            speed_y = state.ball_prev().vy();
        }
        else {
            speed_x = 0.000001;
            speed_y = 0.000001;
        }
        MRA_LOG_INFO("MF line: %d : speed x: %6.4f y: %6.4f", __LINE__, speed_x , speed_y);
    }
    state.mutable_native_filter()->mutable_ball()->set_vx(speed_x);
    state.mutable_native_filter()->mutable_ball()->set_vy(speed_y);
    state.mutable_native_filter()->mutable_ball()->set_vz(0.0); // TODO no airborne balls yet
    // Calculate ball_now() for this filter
    MRA_LOG_INFO("MF line: %d : %s", __LINE__, MRA::convert_proto_to_json_str(state.native_filter()).c_str());
    local_ball_tracking_calculate_ball_now(input, params, *(state.mutable_native_filter()));
    MRA_LOG_INFO("MF line: %d : %s", __LINE__, MRA::convert_proto_to_json_str(state.native_filter()).c_str());
}
