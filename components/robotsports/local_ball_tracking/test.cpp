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
	std::vector<MRA::RobotsportsLocalBallTracking::BallFeature> omni_features;
	std::vector<MRA::RobotsportsLocalBallTracking::BallFeature> stereo_features;
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
	void set_stereo_camera(double range, double angle, double sigma, double sample_time_ms) {
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
			// ball with stereo-vision range and within the stereo vision angle
			object_detected = true;
			auto bf = MRA::RobotsportsLocalBallTracking::BallFeature();
			bf.set_x(r_sample_data.ball_x);
			bf.set_y(r_sample_data.ball_y);
			bf.set_z(0.1);
			bf.set_confidence(0.8);
			bf.set_sigma(m_sv_sigma);
			bf.set_dist(dist_robot_to_ball);
			bf.set_timestamp(r_sample_data.rel_time);
		    r_sample_data.stereo_features.push_back(bf);
		}
		if (dist_robot_to_ball < m_ov_range) {
			// ball with omnivision range
			object_detected = true;
			auto bf = MRA::RobotsportsLocalBallTracking::BallFeature();
			bf.set_x(r_sample_data.ball_x);
			bf.set_y(r_sample_data.ball_y);
			bf.set_z(0.1);
			bf.set_confidence(0.8);
			bf.set_sigma(m_ov_sigma);
			bf.set_dist(dist_robot_to_ball);
			bf.set_timestamp(r_sample_data.rel_time);
		    r_sample_data.omni_features.push_back(bf);
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
    cfg.mutable_general()->set_level(MRA::Datatypes::LogLevel::DEBUG);
    cfg.mutable_general()->set_dumpticks(false);
    cfg.mutable_general()->set_maxlinesize(1000);
    cfg.mutable_general()->set_maxfilesizemb(10.0);
    cfg.mutable_general()->set_pattern("[%Y-%m-%d %H:%M:%S.%f] [%n] [%^%l%$] %v");
    cfg.mutable_general()->set_hotflush(true);
    cfg.mutable_general()->set_enabled(true);
    MRA::Logging::control::setConfiguration(cfg);
}

void execute_ball_traject_test(BallTrajectGenerator traject_generator, double distance)
{
	auto m = RobotsportsLocalBallTracking::RobotsportsLocalBallTracking();
    auto output = RobotsportsLocalBallTracking::Output();
    auto state = RobotsportsLocalBallTracking::State();
    auto local = RobotsportsLocalBallTracking::Local();
    auto params = m.defaultParams();
    google::protobuf::Timestamp timestamp = google::protobuf::util::TimeUtil::GetCurrentTime(); // arbitrary
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
		    input.set_ts(data.rel_time);
		    if (data.omni_features.size() > 0 || (data.stereo_features.size() > 0)) {
		    	input.mutable_omnivision_balls()->Clear();
		    	input.mutable_stereovision_balls()->Clear();
		    	for (unsigned of = 0; of < data.omni_features.size(); of++) {
				    input.mutable_omnivision_balls()->Add()->CopyFrom(data.omni_features[of]);
		    	}
		    	for (unsigned of = 0; of < data.stereo_features.size(); of++) {
				    input.mutable_stereovision_balls()->Add()->CopyFrom(data.stereo_features[of]);
		    	}
			    error_value = m.tick(timestamp, input, params, state, output, local);
			    std::string localStr = MRA::convert_proto_to_json_str(local);
			    MRA_LOG_INFO("diagnostics: %s", localStr.c_str());
			    // Asserts for turn from middle to left position
			    EXPECT_EQ(error_value, 0);
		    }
		}
	}

    MRA_LOG_INFO("< %s::%s", testsuitename.c_str(), testname.c_str());

}

// Test shall run OK and return error_value 0.
TEST(RobotsportsLocalBallTrackingTest, ball_min_y_left_to_right)
{
	auto traject = BallTrajectGenerator();
	traject.set_ball_traject(-6.0, -4.0, 2.0, 0);
	traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	traject.set_omni_camera(6.0, 0.2, 15);
	traject.set_stereo_camera(13.0, 110.0, 0.2, 25);

	std::string testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
   // std::string testsuitename = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
	std::string testsuitename = ::testing::UnitTest::GetInstance()->current_test_info()->test_case_name();
    config_MRA_logger(testsuitename + "_" + testname);
	execute_ball_traject_test(traject, 0.5);
}

#if 0
TEST(RobotsportsLocalBallTrackingTest, ball_min_y_right_to_left)
{
	auto traject = BallTrajectGenerator();
	traject.set_ball_traject(6.0, -4.0, -2.0, 0);
	traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	traject.set_omni_camera(6.0, 0.2, 15);
	traject.set_stereo_camera(13.0, 110.0, 0.2, 25);

	std::string testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
	std::string testsuitename = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    config_MRA_logger(testsuitename + "_" + testname);
	execute_ball_traject_test(traject, 12.0);
}

// Test shall run OK and return error_value 0.
TEST(RobotsportsLocalBallTrackingTest, ball_min_y_left_to_right_robot_rotating_plus)
{
	auto traject = BallTrajectGenerator();
	traject.set_ball_traject(-6.0, -4.0, 2.0, 0);
	traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, 0.25);
	traject.set_omni_camera(6.0, 0.2, 15);
	traject.set_stereo_camera(13.0, 110.0, 0.2, 25);

	std::string testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
	std::string testsuitename = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    config_MRA_logger(testsuitename + "_" + testname);
	execute_ball_traject_test(traject, 12.0);
}

// Test shall run OK and return error_value 0.
TEST(RobotsportsLocalBallTrackingTest, ball_min_y_left_to_right_robot_rotating_min)
{
	auto traject = BallTrajectGenerator();
	traject.set_ball_traject(-6.0, -4.0, 2.0, 0);
	traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, -0.25);
	traject.set_omni_camera(6.0, 0.2, 15);
	traject.set_stereo_camera(13.0, 110.0, 0.2, 25);

	std::string testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
	std::string testsuitename = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    config_MRA_logger(testsuitename + "_" + testname);
	execute_ball_traject_test(traject, 12.0);
}

TEST(RobotsportsLocalBallTrackingTest, ball_plus_y_right_to_left)
{
	auto traject = BallTrajectGenerator();
	traject.set_ball_traject(6.0, 4.0, -2.0, 0);
	traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	traject.set_omni_camera(6.0, 0.2, 15);
	traject.set_stereo_camera(13.0, 110.0, 0.2, 25);

	std::string testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
	std::string testsuitename = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    config_MRA_logger(testsuitename + "_" + testname);
	execute_ball_traject_test(traject, 12.0);
}


// Test shall run OK and return error_value 0.
TEST(RobotsportsLocalBallTrackingTest, ball_min_y_to_plus_y_left)
{
	auto traject = BallTrajectGenerator();
	traject.set_ball_traject(-6.0, -4.0, 0.0, +2.0);
	traject.set_robot_traject(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	traject.set_omni_camera(6.0, 0.2, 15);
	traject.set_stereo_camera(13.0, 110.0, 0.2, 25);

	std::string testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
	std::string testsuitename = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    config_MRA_logger(testsuitename + "_" + testname);
	execute_ball_traject_test(traject, 12.0);
}


//// Basic tick shall run OK and return error_value 0.
//TEST(RobotsportsLocalBallTrackingTest, basicTick)
//{
//	std::string testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
//    config_MRA_logger(testname);
//    MRA_LOG_INFO("> %s", testname.c_str());
//    MRA_TRACE_TEST_FUNCTION();
//
//    // Arrange
//    auto m = RobotsportsLocalBallTracking::RobotsportsLocalBallTracking();
//
//    // Act
//    int error_value = m.tick();
//
//    // Assert
//    EXPECT_EQ(error_value, 0);
//    MRA_LOG_INFO("< %s", testname.c_str());
//}
//
//// Test shall run OK and return error_value 0.
//TEST(RobotsportsLocalBallTrackingTest, non_moving_ball_in_back_of_robot)
//{
//	std::string testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
//    config_MRA_logger(testname);
//    MRA_TRACE_TEST_FUNCTION();
//    MRA_LOG_INFO("> %s", testname.c_str());
//    auto m = RobotsportsLocalBallTracking::RobotsportsLocalBallTracking();
//    auto input = RobotsportsLocalBallTracking::Input();
//    auto output = RobotsportsLocalBallTracking::Output();
//    auto state = RobotsportsLocalBallTracking::State();
//    auto local = RobotsportsLocalBallTracking::Local();
//    auto params = m.defaultParams();
//    google::protobuf::Timestamp timestamp = google::protobuf::util::TimeUtil::GetCurrentTime(); // arbitrary
//
//
//    input.set_ts(1.0);
//	auto bf = MRA::RobotsportsLocalBallTracking::BallFeature();
//	bf.set_x(1.0);
//	bf.set_y(2.0);
//	bf.set_z(0.1);
//	bf.set_confidence(0.8);
//	bf.set_sigma(0.2);
//	bf.set_dist(hypot(bf.x(), bf.y()));
//    input.mutable_omnivision_balls()->Add()->CopyFrom(bf);;
//
//    // start in middle, expect turn to left
//    std::string stateInStr = MRA::convert_proto_to_json_str(state);
//    int error_value = m.tick(timestamp, input, params, state, output, local);
//
//	// second tick
//    input.set_ts(1.1);
//    input.mutable_omnivision_balls(0)->set_x(1.0);
//    input.mutable_omnivision_balls(0)->set_y(2.0);
//    input.mutable_omnivision_balls(0)->set_z(0.1);
//    input.mutable_omnivision_balls(0)->set_confidence(0.8);
//    input.mutable_omnivision_balls(0)->set_sigma(0.2);
//    input.mutable_omnivision_balls(0)->set_dist(hypot(bf.x(), bf.y()));
//
//    // start in middle, expect turn to left
//    stateInStr = MRA::convert_proto_to_json_str(state);
//    error_value = m.tick(timestamp, input, params, state, output, local);
//
//    // Asserts for turn from middle to left position
//    EXPECT_EQ(error_value, 0);
//    MRA_LOG_INFO("< %s", testname.c_str());
//}

#endif

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
	auto cfg = MRA::Logging::control::getConfiguration(); // return type: Logging.proto
	cfg.mutable_general()->set_enabled(false);
	MRA::Logging::control::setConfiguration(cfg);

    return r;
}

