// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests


// Include testframework
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_factory.hpp"
using namespace ::testing;

#include "angles.hpp" // MRA::Geometry

// System under test:
#include "RobotsportsLocalBall.hpp"
using namespace MRA;

class SimulatedBallCandidate {
public:
    double x;
    double y;
    double z;
    double confidence;
    double sigma;
    double ts;
    int source;
    bool  is_free;
    bool  in_air;
};

class BallTrajectData
{
public:
    double rel_time;
    double ball_x;
    double ball_y;
    double robot_x;
    double robot_y;
    double robot_rz;
    std::vector<SimulatedBallCandidate> omni_candidates;
    std::vector<SimulatedBallCandidate> frontcamera_candidates;
};

class BallTrajectGenerator
{
public:
    BallTrajectGenerator() {
//        std;:random_device rd;
//        std::mt19937 gen(rd());
//        normal_distribution<double>(0.0, sigmga);

    }

    void set_ball_traject(double ball_x, double ball_y, double ball_vx, double ball_vy) {
        m_ball_x = ball_x;
        m_ball_y = ball_y;
        m_ball_vx = ball_vx;
        m_ball_vy = ball_vy;

    };
    void set_robot_traject(double robot_x, double robot_y, double robot_rz, double robot_vx, double robot_vy, double robot_vrz) {
        m_robot_x = robot_x;
        m_robot_y = robot_y;
        m_robot_rz = robot_rz;
        m_robot_vx = robot_vx;
        m_robot_vy = robot_vy;
        m_robot_vrz = robot_vrz;
    };
    void set_omni_camera(double range, double sigma, double sample_time_ms) {
        m_ov_range = range;
        m_ov_sigma = sigma;
        m_ov_sample_time_ms = sample_time_ms;
    };
    void set_front_camera(double range, double angle, double sigma, double sample_time_ms) {
        m_sv_range = range;
        m_sv_angle = angle;
        m_sv_sigma = sigma;
        m_sv_sample_time_ms = sample_time_ms;
    }



    int greatest_common_divisor( int a, int b) {
        int gcd = 1;
        for (int i = 2; i <= a && i <= b; i++) {
            if (a % i == 0 && b % i == 0) {
                gcd = i;
            }
        }
        return gcd;
    }

    unsigned generate(double distance) {
        // calculate sample tick time in milliseconds
        if (m_ov_sample_time_ms - m_sv_sample_time_ms == 0)
        {
            throw std::logic_error("currenlty only supporting of same sample rate for vision devices");
            return 0;
        }
        sim_sample_time = greatest_common_divisor(m_sv_sample_time_ms, m_ov_sample_time_ms) / 1000.0;
        double ball_vel = hypot(m_ball_vx, m_ball_vy);
        double tot_time =  distance / ball_vel;
        m_nr_samples = ceil(tot_time / sim_sample_time) + 1; // add extra sample for initial position

        return m_nr_samples;
    }

    bool get_sample_data(unsigned sample_idx, BallTrajectData& r_sample_data)
    {
        bool object_detected = false;
        r_sample_data.rel_time = sample_idx * sim_sample_time;
        r_sample_data.ball_x = m_ball_x + r_sample_data.rel_time * m_ball_vx;
        r_sample_data.ball_y = m_ball_y + r_sample_data.rel_time * m_ball_vy;
        r_sample_data.robot_x = m_robot_x + r_sample_data.rel_time * m_robot_vx;
        r_sample_data.robot_y = m_robot_y + r_sample_data.rel_time * m_robot_vy;
        r_sample_data.robot_rz = m_robot_rz + r_sample_data.rel_time * m_robot_vrz;

        double angle_robot_to_ball = MRA::Geometry::wrap_pi( r_sample_data.robot_rz +
                                        MRA::Geometry::calc_facing_angle_fcs(r_sample_data.robot_x, r_sample_data.robot_y, r_sample_data.ball_x, r_sample_data.ball_y));

        r_sample_data.omni_candidates.clear();
        r_sample_data.frontcamera_candidates.clear();
        auto dist_robot_to_ball = hypot(fabs(r_sample_data.ball_x - r_sample_data.robot_x), fabs(r_sample_data.ball_y - r_sample_data.robot_y));
        if ((dist_robot_to_ball < m_sv_range) && fabs(angle_robot_to_ball) <= (0.5*m_sv_angle)) {
            // ball with frontcamera range and within the frontcamera vision angle
            object_detected = true;

            SimulatedBallCandidate bf;
            bf.x = r_sample_data.ball_x;
            bf.y = r_sample_data.ball_y;
            bf.z = 0.1;
            bf.confidence = 0.8;
            bf.sigma = m_sv_sigma;
            bf.ts = r_sample_data.rel_time;
            bf.source = 1;
            bf.in_air = false;
            bf.is_free = true;
            r_sample_data.frontcamera_candidates.push_back(bf);

        }

        if (dist_robot_to_ball < m_ov_range) {
            // ball with omnivision range
            object_detected = true;
            SimulatedBallCandidate bf;
            bf.x = r_sample_data.ball_x;
            bf.y = r_sample_data.ball_y;
            bf.z = 0.1;
            bf.confidence = 0.8;
            bf.sigma = m_sv_sigma;
            bf.ts = r_sample_data.rel_time;
            r_sample_data.omni_candidates.push_back(bf);
        }
        return object_detected;
    }


    private:
        double m_ball_x;
        double m_ball_y;
        double m_ball_vx;
        double m_ball_vy;
        double m_robot_x;
        double m_robot_y;
        double m_robot_rz;
        double m_robot_vx;
        double m_robot_vy;
        double m_robot_vrz;
        double m_ov_range;
        double m_ov_sigma;
        int m_ov_sample_time_ms;
        double m_sv_range;
        double m_sv_angle;
        double m_sv_sigma;
        int m_sv_sample_time_ms;
        double sim_sample_time;
        int m_nr_samples;
        bool position_noise = false;
        double position_sigma = 0.1;
        bool measure_noise = false;
        double measure_sigma = 0.1;
};


RobotsportsLocalBall::Output execute_ball_traject_test(BallTrajectGenerator traject_generator, double distance)
{
    auto m = RobotsportsLocalBall::RobotsportsLocalBall();
    auto output = RobotsportsLocalBall::Output();
    auto state = RobotsportsLocalBall::State();
    auto diagnostics = RobotsportsLocalBall::Diagnostics();
    auto params = m.defaultParams();
    int n_samples = traject_generator.generate(distance);
    int error_value = 0;
    for (auto sample = 0; sample < n_samples; sample++)
    {
        BallTrajectData data;
        if  (traject_generator.get_sample_data(sample, data)) {
            auto input = RobotsportsLocalBall::Input();
            input.Clear();
            input.mutable_omnivision_balls()->Clear();
            MRA_LOG_DEBUG("test_input {\"time\": %6.4f, \"ball_x\": %6.4f, \"ball_y\": %6.4f, \"robot_x\": %6.4f, \"robot_y\": %6.4f, \"robot_rz\": %6.4f}",
                          data.rel_time, data.ball_x, data.ball_y, data.robot_x, data.robot_y, data.robot_rz);

            google::protobuf::Timestamp timestamp = google::protobuf::util::TimeUtil::MillisecondsToTimestamp(data.rel_time * 1000);
            if (data.omni_candidates.size() > 0 || (data.frontcamera_candidates.size() > 0)) {

                input.mutable_omnivision_balls()->Clear();
                for (unsigned idx = 0; idx < data.omni_candidates.size(); idx++) {
                    auto  candidate = MRA::Datatypes::BallCandidate();
                    candidate.mutable_pose_fcs()->set_x(data.omni_candidates[idx].x);
                    candidate.mutable_pose_fcs()->set_y(data.omni_candidates[idx].y);
                    candidate.mutable_pose_fcs()->set_z(data.omni_candidates[idx].z);
                    candidate.set_confidence(data.omni_candidates[idx].confidence);
                    candidate.set_sigma(data.omni_candidates[idx].sigma);
                    google::protobuf::Timestamp timestamp = google::protobuf::util::TimeUtil::MillisecondsToTimestamp(data.omni_candidates[idx].ts*1000.0);
                    candidate.mutable_timestamp()->CopyFrom(timestamp);
                    candidate.set_source(data.omni_candidates[idx].source);

                    input.mutable_omnivision_balls()->Add()->CopyFrom(candidate);
                }


                input.mutable_frontcamera_balls()->Clear();
                for (unsigned idx = 0; idx < data.frontcamera_candidates.size(); idx++) {
                    auto  candidate = MRA::Datatypes::BallCandidate();
                    candidate.mutable_pose_fcs()->set_x(data.frontcamera_candidates[idx].x);
                    candidate.mutable_pose_fcs()->set_y(data.frontcamera_candidates[idx].y);
                    candidate.mutable_pose_fcs()->set_z(data.frontcamera_candidates[idx].z);
                    candidate.set_confidence(data.frontcamera_candidates[idx].confidence);
                    candidate.set_sigma(data.frontcamera_candidates[idx].sigma);
                    google::protobuf::Timestamp timestamp = google::protobuf::util::TimeUtil::MillisecondsToTimestamp(data.frontcamera_candidates[idx].ts*1000.0);
                    candidate.mutable_timestamp()->CopyFrom(timestamp);
                    candidate.set_source(data.frontcamera_candidates[idx].source);

                    input.mutable_frontcamera_balls()->Add()->CopyFrom(candidate);
                }

                error_value = m.tick(timestamp, input, params, state, output, diagnostics);
                MRA_LOG_DEBUG("diagnostics: %s", MRA::convert_proto_to_json_str(diagnostics).c_str());
                MRA_LOG_DEBUG("state: %s", MRA::convert_proto_to_json_str(state).c_str());
                // Asserts for turn from middle to left position
                EXPECT_EQ(error_value, 0);
            }
        }
    }

    return output;
}

// Test with ball moving from left to right behind the robot. Runs for a few ticks and the final speed is verified.
TEST(RobotsportsLocalBallTest, ball_min_y_left_to_right)
{
    MRA_TRACE_TEST_FUNCTION();
    auto traject = BallTrajectGenerator();
    traject.set_ball_traject(-6.0, -4.0, 2.0, 0);
    traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    traject.set_omni_camera(6.0, 0.2, 15);
    traject.set_front_camera(13.0, 110.0, 0.2, 25);
    double traject_dist = 0.05;

    auto last_output = execute_ball_traject_test(traject, traject_dist);
    EXPECT_NEAR(last_output.ball().pos_vel_fcs().velocity().x(), 2.0, 0.001); // check if final speed is reached: x direction
    EXPECT_NEAR(last_output.ball().pos_vel_fcs().velocity().y(), 0.0, 0.001); // check if final speed is reached: y direction
}

// Test with ball moving from right to left behind the robot over longer distance.
// This test is used to check if no effects are present related to direction and orientation together with similar tests
TEST(RobotsportsLocalBallTest, ball_min_y_right_to_left)
{
    MRA_TRACE_TEST_FUNCTION();
    auto traject = BallTrajectGenerator();
    traject.set_ball_traject(6.0, -4.0, -2.0, 0);
    traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    traject.set_omni_camera(6.0, 0.2, 15);
    traject.set_front_camera(13.0, 110.0, 0.2, 25);
    double traject_dist = 12.0;

    auto last_output = execute_ball_traject_test(traject, traject_dist);
}

// Test with ball moving from right to left behind the robot over longer distance.
// This test is used to check if no effects are present related to direction and orientation together with similar tests
TEST(RobotsportsLocalBallTest, ball_plus_y_right_to_left)
{
    MRA_TRACE_TEST_FUNCTION();
    auto traject = BallTrajectGenerator();
    traject.set_ball_traject(6.0, -4.0, 2.0, 0);
    traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    traject.set_omni_camera(6.0, 0.2, 15);
    traject.set_front_camera(13.0, 110.0, 0.2, 25);
    double traject_dist = 12.0;

    auto last_output = execute_ball_traject_test(traject, traject_dist);
}

// Test with ball moving from left to right behind the robot over longer distance while robot is rotating in positive direction.
// This test is used to check if no effects are present related to direction and orientation together with similar tests
TEST(RobotsportsLocalBallTest, ball_min_y_left_to_right_robot_rotating_plus)
{
    MRA_TRACE_TEST_FUNCTION();
    auto traject = BallTrajectGenerator();
    traject.set_ball_traject(-6.0, -4.0, 2.0, 0);
    traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, 0.25);
    traject.set_omni_camera(6.0, 0.2, 15);
    traject.set_front_camera(13.0, 110.0, 0.2, 25);
    double traject_dist = 1.0;

    auto last_output = execute_ball_traject_test(traject, traject_dist);
}

// Test with ball moving from left to right behind the robot over longer distance while robot is rotating in negative direction.
// This test is used to check if no effects are present related to direction and orientation together with similar tests
TEST(RobotsportsLocalBallTest, ball_min_y_left_to_right_robot_rotating_min)
{
    MRA_TRACE_TEST_FUNCTION();
    auto traject = BallTrajectGenerator();
    traject.set_ball_traject(-6.0, -4.0, 2.0, 0);
    traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, -0.25);
    traject.set_omni_camera(6.0, 0.2, 15);
    traject.set_front_camera(13.0, 110.0, 0.2, 25);

    double traject_dist = 1.0;

    auto last_output = execute_ball_traject_test(traject, traject_dist);
}


// Test with ball moving from back to front side at left side of the robot.
// This test is used to check if no effects are present related to direction and orientation together with similar tests
TEST(RobotsportsLocalBallTest, ball_min_y_to_plus_y_left)
{
    MRA_TRACE_TEST_FUNCTION();
    auto traject = BallTrajectGenerator();
    traject.set_ball_traject(-6.0, -4.0, 0.0, +2.0);
    traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    traject.set_omni_camera(6.0, 0.2, 15);
    traject.set_front_camera(13.0, 110.0, 0.2, 25);

    double traject_dist = 1.0;

    auto last_output = execute_ball_traject_test(traject, traject_dist);
}


// Basic tick shall run OK and return error_value 0.
TEST(RobotsportsLocalBallTest, basicTick)
{
    // Arrange
    auto m = RobotsportsLocalBall::RobotsportsLocalBall();

    // Act
    int error_value = m.tick();

    // Assert
    EXPECT_EQ(error_value, 0);
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

