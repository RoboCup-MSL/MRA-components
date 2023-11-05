#include "RobotsportsLocalBallTracking_preprocessing.hpp"

#define BALL_MAX_HISTORY 10  // TODO from params ?


typedef struct xy_struct_t {
    double x;
    double y;
} xy_t;

static std::vector<xy_t> ball_position_history(BALL_MAX_HISTORY);


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


void local_ball_tracking_native_filter(int nrBallsThisTime,
                                    std::vector<ball_feature_t>& ballData,
                                        const MRA::RobotsportsLocalBallTracking::ParamsType &params,
                                        const MRA::RobotsportsLocalBallTracking::InputType &input,
                                        MRA::RobotsportsLocalBallTracking::StateType &state,
                                        MRA::RobotsportsLocalBallTracking::OutputType &output)
{
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
    state.mutable_native_filter()->mutable_ball()->set_timestamp(ballData[winning_idx].timestamp); // timestamp based off of liveseconds
    // update speed
    auto delta_t = state.native_filter().ball().timestamp() - state.native_filter().ball_prev().timestamp(); //getso(sm.ball_process.out.ball_prev.ts);
    auto speed_x = 0.0;
    auto speed_y = 0.0;
    if (delta_t > 0.0) {
        // calculate speed based on displacement and difference in timestamps
        //      // use better differentiation
        xy_t xy_pos;
        xy_pos.x = output.ball().x();
        xy_pos.y = output.ball().y();
        ball_position_history.pop_back();
        ball_position_history.insert(ball_position_history.begin(), xy_pos);
        //double sample = getso(tasktime);  // use last sample time value as tHE sample time value, knowing we do have jitter
        xy_t speed = { 0, 0 };
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
    state.mutable_native_filter()->mutable_ball()->set_vz(0.0); // TODO no airborne balls yet
    // Calculate ball_now() for this filter
    local_ball_tracking_calculate_ball_now(input, params, *(state.mutable_native_filter()));
}
