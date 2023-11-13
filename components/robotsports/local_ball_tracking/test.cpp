// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests

#include <cstdint>

#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_factory.hpp"
#include "logging.hpp"
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <vector>

#include "angles.hpp"

using namespace ::testing;

// System under test:
#include "RobotsportsLocalBallTracking.hpp"
using namespace MRA;
using namespace std;

class BallTrajectData
{
public:
	double rel_time;
	double ball_x;
	double ball_y;
	double robot_x;
	double robot_y;
	double robot_rz;
	std::vector<MRA::RobotsportsLocalBallTracking::BallCandidate> omni_candidates;
	std::vector<MRA::RobotsportsLocalBallTracking::BallCandidate> frontcamera_candidates;
};

class BallTrajectGenerator
{
public:
	BallTrajectGenerator() {

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
		    MRA_LOG_ERROR("currenlty only supporting of same sample rate for vision devices");
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

		auto dist_robot_to_ball = hypot(fabs(r_sample_data.ball_x - r_sample_data.robot_x), fabs(r_sample_data.ball_y - r_sample_data.robot_y));
		if ((dist_robot_to_ball < m_sv_range) && fabs(angle_robot_to_ball) <= (0.5*m_sv_angle)) {
			// ball with frontcamera range and within the frontcamera vision angle
			object_detected = true;
			auto bf = MRA::RobotsportsLocalBallTracking::BallCandidate();
			bf.mutable_measured_pose_fcs()->set_x(r_sample_data.ball_x);
			bf.mutable_measured_pose_fcs()->set_y(r_sample_data.ball_y);
			bf.mutable_measured_pose_fcs()->set_z(0.1);
			bf.set_confidence(0.8);
			bf.set_sigma(m_sv_sigma);
			bf.mutable_timestamp()->CopyFrom(google::protobuf::util::TimeUtil::MillisecondsToTimestamp(r_sample_data.rel_time * 1000.0));
		    r_sample_data.frontcamera_candidates.push_back(bf);
		}
		if (dist_robot_to_ball < m_ov_range) {
			// ball with omnivision range
			object_detected = true;
			auto bf = MRA::RobotsportsLocalBallTracking::BallCandidate();
			bf.mutable_measured_pose_fcs()->set_x(r_sample_data.ball_x);
			bf.mutable_measured_pose_fcs()->set_y(r_sample_data.ball_y);
			bf.mutable_measured_pose_fcs()->set_z(0.1);
			bf.set_confidence(0.8);
			bf.set_sigma(m_ov_sigma);
			bf.mutable_timestamp()->CopyFrom(google::protobuf::util::TimeUtil::MillisecondsToTimestamp(r_sample_data.rel_time * 1000.0));;
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
};

static void config_MRA_logger(std::string component)
{
    auto cfg = MRA::Logging::control::getConfiguration(); // return type: Logging.proto
    cfg.set_folder("/home/jurge/log");
    cfg.mutable_general()->set_component(component.c_str());
    cfg.mutable_general()->set_level(MRA::Datatypes::LogLevel::INFO);
    cfg.mutable_general()->set_dumpticks(false);
    cfg.mutable_general()->set_maxlinesize(1000);
    cfg.mutable_general()->set_maxfilesizemb(10.0);
    cfg.mutable_general()->set_pattern("[%Y-%m-%d %H:%M:%S.%f] [%n] [%^%l%$] %v");
    cfg.mutable_general()->set_hotflush(true);
    cfg.mutable_general()->set_enabled(true);
    MRA::Logging::control::setConfiguration(cfg);
}

RobotsportsLocalBallTracking::Output execute_ball_traject_test(BallTrajectGenerator traject_generator, double distance)
{
	auto m = RobotsportsLocalBallTracking::RobotsportsLocalBallTracking();
    auto output = RobotsportsLocalBallTracking::Output();
    auto state = RobotsportsLocalBallTracking::State();
    auto local = RobotsportsLocalBallTracking::Local();
    auto params = m.defaultParams();
	const TestInfo* test_info_ = ::testing::UnitTest::GetInstance()->current_test_info();
	std::string testname = test_info_->name();
    std::string testsuitename = test_info_->test_case_name();
    MRA_TRACE_TEST_FUNCTION(); // enable tracing
    MRA_LOG_INFO("> %s::%s", testsuitename.c_str(), testname.c_str());
	int n_samples = traject_generator.generate(distance);
	int error_value = 0;
	for (auto sample = 0; sample < n_samples; sample++)
	{
		BallTrajectData data;
		if  (traject_generator.get_sample_data(sample, data)) {
		    auto input = RobotsportsLocalBallTracking::Input();
		    input.Clear();
		    input.mutable_omnivision_balls()->Clear();
		    MRA_LOG_INFO("time {\"time\": %6.4f}", data.rel_time);
		    google::protobuf::Timestamp timestamp = google::protobuf::util::TimeUtil::MillisecondsToTimestamp(data.rel_time * 1000);
		    if (data.omni_candidates.size() > 0 || (data.frontcamera_candidates.size() > 0)) {
		    	input.mutable_omnivision_balls()->Clear();
		    	input.mutable_frontcamera_balls()->Clear();
		    	for (unsigned of = 0; of < data.omni_candidates.size(); of++) {
				    input.mutable_omnivision_balls()->Add()->CopyFrom(data.omni_candidates[of]);
		    	}
		    	for (unsigned of = 0; of < data.frontcamera_candidates.size(); of++) {
				    input.mutable_frontcamera_balls()->Add()->CopyFrom(data.frontcamera_candidates[of]);
		    	}

		        error_value = m.tick(timestamp, input, params, state, output, local);
			    MRA_LOG_INFO("diagnostics: %s", MRA::convert_proto_to_json_str(local).c_str());
                MRA_LOG_INFO("state: %s", MRA::convert_proto_to_json_str(state).c_str());
			    // Asserts for turn from middle to left position
			    EXPECT_EQ(error_value, 0);
		    }
		}
	}

    MRA_LOG_INFO("< %s::%s", testsuitename.c_str(), testname.c_str());
    return output;
}

// Test shall run OK and return error_value 0.
TEST(RobotsportsLocalBallTrackingTest, ball_min_y_left_to_right)
{
	auto traject = BallTrajectGenerator();
	traject.set_ball_traject(-6.0, -4.0, 2.0, 0);
	traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	traject.set_omni_camera(6.0, 0.2, 15);
	traject.set_front_camera(13.0, 110.0, 0.2, 25);
	double traject_dist = 0.05;

	std::string testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    std::string testsuitename = ::testing::UnitTest::GetInstance()->current_test_info()->test_case_name();
    config_MRA_logger(testsuitename + "_" + testname);
	auto last_output = execute_ball_traject_test(traject, traject_dist);
    EXPECT_NEAR(last_output.ball().pos_vel_fcs().velocity().x(), 2.0, 0.001); // check if final speed is reached: x direction
    EXPECT_NEAR(last_output.ball().pos_vel_fcs().velocity().y(), 0.0, 0.001); // check if final speed is reached: y direction
}

TEST(RobotsportsLocalBallTrackingTest, ball_min_y_right_to_left)
{
	auto traject = BallTrajectGenerator();
	traject.set_ball_traject(6.0, -4.0, -2.0, 0);
	traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	traject.set_omni_camera(6.0, 0.2, 15);
	traject.set_front_camera(13.0, 110.0, 0.2, 25);
    double traject_dist = 12.0;

	std::string testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    std::string testsuitename = ::testing::UnitTest::GetInstance()->current_test_info()->test_case_name();
    config_MRA_logger(testsuitename + "_" + testname);
    auto last_output = execute_ball_traject_test(traject, traject_dist);
}
TEST(RobotsportsLocalBallTrackingTest, ball_plus_y_right_to_left)
{
    auto traject = BallTrajectGenerator();
    traject.set_ball_traject(6.0, -4.0, 2.0, 0);
    traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    traject.set_omni_camera(6.0, 0.2, 15);
    traject.set_front_camera(13.0, 110.0, 0.2, 25);
    double traject_dist = 12.0;

    std::string testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    std::string testsuitename = ::testing::UnitTest::GetInstance()->current_test_info()->test_case_name();
    config_MRA_logger(testsuitename + "_" + testname);
    auto last_output = execute_ball_traject_test(traject, traject_dist);
}

// Test shall run OK and return error_value 0.
TEST(RobotsportsLocalBallTrackingTest, ball_min_y_left_to_right_robot_rotating_plus)
{
	auto traject = BallTrajectGenerator();
	traject.set_ball_traject(-6.0, -4.0, 2.0, 0);
	traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, 0.25);
	traject.set_omni_camera(6.0, 0.2, 15);
	traject.set_front_camera(13.0, 110.0, 0.2, 25);
    double traject_dist = 1.0;

    std::string testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    std::string testsuitename = ::testing::UnitTest::GetInstance()->current_test_info()->test_case_name();
    config_MRA_logger(testsuitename + "_" + testname);
    auto last_output = execute_ball_traject_test(traject, traject_dist);
}

// Test shall run OK and return error_value 0.
TEST(RobotsportsLocalBallTrackingTest, ball_min_y_left_to_right_robot_rotating_min)
{
	auto traject = BallTrajectGenerator();
	traject.set_ball_traject(-6.0, -4.0, 2.0, 0);
	traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, -0.25);
	traject.set_omni_camera(6.0, 0.2, 15);
	traject.set_front_camera(13.0, 110.0, 0.2, 25);

    double traject_dist = 1.0;

    std::string testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    std::string testsuitename = ::testing::UnitTest::GetInstance()->current_test_info()->test_case_name();
    config_MRA_logger(testsuitename + "_" + testname);
    auto last_output = execute_ball_traject_test(traject, traject_dist);
}


// Test shall run OK and return error_value 0.
TEST(RobotsportsLocalBallTrackingTest, ball_min_y_to_plus_y_left)
{
	auto traject = BallTrajectGenerator();
	traject.set_ball_traject(-6.0, -4.0, 0.0, +2.0);
	traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	traject.set_omni_camera(6.0, 0.2, 15);
	traject.set_front_camera(13.0, 110.0, 0.2, 25);

    double traject_dist = 1.0;

    std::string testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    std::string testsuitename = ::testing::UnitTest::GetInstance()->current_test_info()->test_case_name();
    config_MRA_logger(testsuitename + "_" + testname);
    auto last_output = execute_ball_traject_test(traject, traject_dist);
}


// Basic tick shall run OK and return error_value 0.
TEST(RobotsportsLocalBallTrackingTest, basicTick)
{
	std::string testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    config_MRA_logger(testname);
    MRA_LOG_INFO("> %s", testname.c_str());
    MRA_TRACE_TEST_FUNCTION();

    // Arrange
    auto m = RobotsportsLocalBallTracking::RobotsportsLocalBallTracking();

    // Act
    int error_value = m.tick();

    // Assert
    EXPECT_EQ(error_value, 0);
    MRA_LOG_INFO("< %s", testname.c_str());
}

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
	auto cfg = MRA::Logging::control::getConfiguration(); // return type: Logging.proto
	cfg.mutable_general()->set_enabled(false);
	MRA::Logging::control::setConfiguration(cfg);

    return r;
}

