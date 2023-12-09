// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests


// Include testframework
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_factory.hpp"
using namespace ::testing;

#include "angles.hpp" // MRA::Geometry

// System under test:
#include "RobotsportsObstacleTracking.hpp"
using namespace MRA;


class SimulatedObstacleCandidate {
public:
    double x;
    double y;
    double z;
    double confidence;
    double sigma;
    double ts;
    int source;
};

class ObstacleTrajectData
{
public:
    double rel_time;
    double obstacle_x;
    double obstacle_y;
    double robot_x;
    double robot_y;
    double robot_rz;
    std::vector<SimulatedObstacleCandidate> omni_candidates;
    std::vector<SimulatedObstacleCandidate> frontcamera_candidates;
};

class ObstacleTrajectGenerator
{
public:
    ObstacleTrajectGenerator() {
//        std;:random_device rd;
//        std::mt19937 gen(rd());
//        normal_distribution<double>(0.0, sigmga);

    }

    void set_obstacle_traject(double obstacle_x, double obstacle_y, double obstacle_vx, double obstacle_vy) {
        m_obstacle_x = obstacle_x;
        m_obstacle_y = obstacle_y;
        m_obstacle_vx = obstacle_vx;
        m_obstacle_vy = obstacle_vy;

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
        double obstacle_vel = hypot(m_obstacle_vx, m_obstacle_vy);
        double tot_time =  distance / obstacle_vel;
        m_nr_samples = ceil(tot_time / sim_sample_time) + 1; // add extra sample for initial position

        return m_nr_samples;
    }

    bool get_sample_data(unsigned sample_idx, ObstacleTrajectData& r_sample_data)
    {
        bool object_detected = false;
        r_sample_data.rel_time = sample_idx * sim_sample_time;
        r_sample_data.obstacle_x = m_obstacle_x + r_sample_data.rel_time * m_obstacle_vx;
        r_sample_data.obstacle_y = m_obstacle_y + r_sample_data.rel_time * m_obstacle_vy;
        r_sample_data.robot_x = m_robot_x + r_sample_data.rel_time * m_robot_vx;
        r_sample_data.robot_y = m_robot_y + r_sample_data.rel_time * m_robot_vy;
        r_sample_data.robot_rz = m_robot_rz + r_sample_data.rel_time * m_robot_vrz;

        double angle_robot_to_obstacle = MRA::Geometry::wrap_pi( r_sample_data.robot_rz +
                                        MRA::Geometry::calc_facing_angle_fcs(r_sample_data.robot_x, r_sample_data.robot_y, r_sample_data.obstacle_x, r_sample_data.obstacle_y));

        r_sample_data.omni_candidates.clear();
        r_sample_data.frontcamera_candidates.clear();
        auto dist_robot_to_obstacle = hypot(fabs(r_sample_data.obstacle_x - r_sample_data.robot_x), fabs(r_sample_data.obstacle_y - r_sample_data.robot_y));
        if ((dist_robot_to_obstacle < m_sv_range) && fabs(angle_robot_to_obstacle) <= (0.5*m_sv_angle)) {
            // obstacle with frontcamera range and within the frontcamera vision angle
            object_detected = true;

            SimulatedObstacleCandidate bf;
            bf.x = r_sample_data.obstacle_x;
            bf.y = r_sample_data.obstacle_y;
            bf.z = 0.1;
            bf.confidence = 0.8;
            bf.sigma = m_sv_sigma;
            bf.ts = r_sample_data.rel_time;
            bf.source = 1;
            r_sample_data.frontcamera_candidates.push_back(bf);

        }

        if (dist_robot_to_obstacle < m_ov_range) {
            // obstacle with omnivision range
            object_detected = true;
            SimulatedObstacleCandidate bf;
            bf.x = r_sample_data.obstacle_x;
            bf.y = r_sample_data.obstacle_y;
            bf.z = 0.1;
            bf.confidence = 0.8;
            bf.sigma = m_sv_sigma;
            bf.ts = r_sample_data.rel_time;
            r_sample_data.omni_candidates.push_back(bf);
        }
        return object_detected;
    }


    private:
        double m_obstacle_x;
        double m_obstacle_y;
        double m_obstacle_vx;
        double m_obstacle_vy;
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



RobotsportsObstacleTracking::Output execute_obstacle_traject_test(ObstacleTrajectGenerator traject_generator, double distance)
{
    auto m = RobotsportsObstacleTracking::RobotsportsObstacleTracking();
    auto output = RobotsportsObstacleTracking::Output();
    auto state = RobotsportsObstacleTracking::State();
    auto local = RobotsportsObstacleTracking::Local();
    auto params = m.defaultParams();
    int n_samples = traject_generator.generate(distance);
    int error_value = 0;
    for (auto sample = 0; sample < n_samples; sample++)
    {
        ObstacleTrajectData data;
        if  (traject_generator.get_sample_data(sample, data)) {
            auto input = RobotsportsObstacleTracking::Input();
            input.Clear();
            input.mutable_obstacle_candidates()->Clear();
            MRA_LOG_DEBUG("test_input {\"time\": %6.4f, \"obstacle_x\": %6.4f, \"obstacle_y\": %6.4f, \"robot_x\": %6.4f, \"robot_y\": %6.4f, \"robot_rz\": %6.4f}",
                          data.rel_time, data.obstacle_x, data.obstacle_y, data.robot_x, data.robot_y, data.robot_rz);

            google::protobuf::Timestamp timestamp = google::protobuf::util::TimeUtil::MillisecondsToTimestamp(data.rel_time * 1000);
            if (data.omni_candidates.size() > 0) {

                for (unsigned idx = 0; idx < data.omni_candidates.size(); idx++) {
                    auto  candidate = MRA::RobotsportsObstacleTracking::ObstacleCandidate();
                    candidate.mutable_pose_fcs()->set_x(data.omni_candidates[idx].x);
                    candidate.mutable_pose_fcs()->set_y(data.omni_candidates[idx].y);
                    candidate.mutable_pose_fcs()->set_z(data.omni_candidates[idx].z);
                    candidate.set_confidence(data.omni_candidates[idx].confidence);
                    google::protobuf::Timestamp timestamp = google::protobuf::util::TimeUtil::MillisecondsToTimestamp(data.omni_candidates[idx].ts*1000.0);
                    candidate.mutable_timestamp()->CopyFrom(timestamp);

                    input.mutable_obstacle_candidates()->Add()->CopyFrom(candidate);
                }

                error_value = m.tick(timestamp, input, params, state, output, local);
                MRA_LOG_DEBUG("diagnostics: %s", MRA::convert_proto_to_json_str(local).c_str());
                MRA_LOG_DEBUG("state: %s", MRA::convert_proto_to_json_str(state).c_str());
                // Asserts for turn from middle to left position
                EXPECT_EQ(error_value, 0);
            }
        }
    }

    return output;
}

// Test with obstacle moving from left to right behind the robot. Runs for a few ticks and the final speed is verified.
TEST(RobotsportsObstacleTrackingTest, obstacle_min_y_left_to_right)
{
    MRA_TRACE_TEST_FUNCTION();
    auto traject = ObstacleTrajectGenerator();
    traject.set_obstacle_traject(-5.0, -4.0, 2.0, 0);
    traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    traject.set_omni_camera(8.0, 0.2, 15);
    traject.set_front_camera(13.0, 110.0, 0.2, 25);
    double traject_dist = 0.5;

    auto last_output = execute_obstacle_traject_test(traject, traject_dist);
//    EXPECT_NEAR(last_output.ball().pos_vel_fcs().velocity().x(), 2.0, 0.001); // check if final speed is reached: x direction
//    EXPECT_NEAR(last_output.ball().pos_vel_fcs().velocity().y(), 0.0, 0.001); // check if final speed is reached: y direction
}



// Basic tick shall run OK and return error_value 0.
TEST(RobotsportsObstacleTrackingTest, basicTick)
{
    // Arrange
    auto m = RobotsportsObstacleTracking::RobotsportsObstacleTracking();

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

