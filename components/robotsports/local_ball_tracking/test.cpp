// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests

#include <cstdint>

#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_factory.hpp"
#include "logging.hpp"
#include <unistd.h>
using namespace ::testing;

// System under test:
#include "RobotsportsLocalBallTracking.hpp"
using namespace MRA;

static void config_MRA_logger()
{
//    auto cfg = MRA::Logging::control::getConfiguration(); // return type: Logging.proto
//    cfg.mutable_general()->set_enabled(false);
//    cfg.set_folder("/home/jurge/robotsports/build");
//    cfg.mutable_general()->set_component("MRA-test");
//    cfg.mutable_general()->set_level(MRA::Datatypes::LogLevel::INFO);
//    cfg.mutable_general()->set_enabled(true);
//    cfg.mutable_general()->set_dumpticks(false);
//    cfg.mutable_general()->set_maxlinesize(1000);
//    cfg.mutable_general()->set_maxfilesizemb(10.0);
//    cfg.mutable_general()->set_pattern("[%Y-%m-%d %H:%M:%S.%f] [%n] [%^%l%$] %v");
//    cfg.mutable_general()->set_hotflush(true);
//    MRA::Logging::control::setConfiguration(cfg);
}

// Basic tick shall run OK and return error_value 0.
TEST(RobotsportsLocalBallTrackingTest, basicTick)
{
    // Arrange
    auto m = RobotsportsLocalBallTracking::RobotsportsLocalBallTracking();

    config_MRA_logger();
    // Act
    int error_value = m.tick();

    // Assert
    EXPECT_EQ(error_value, 0);
}

// Test shall run OK and return error_value 0.
TEST(RobotsportsLocalBallTrackingTest, non_moving_ball_in_back_of_robot)
{
    auto m = RobotsportsLocalBallTracking::RobotsportsLocalBallTracking();
    auto input = RobotsportsLocalBallTracking::Input();
    auto output = RobotsportsLocalBallTracking::Output();
    auto state = RobotsportsLocalBallTracking::State();
    auto local = RobotsportsLocalBallTracking::Local();
    auto params = m.defaultParams();
    google::protobuf::Timestamp timestamp = google::protobuf::util::TimeUtil::GetCurrentTime(); // arbitrary


    input.set_ts(1.0);
	auto bf = MRA::RobotsportsLocalBallTracking::BallFeature();
	bf.set_x(1.0);
	bf.set_y(2.0);
	bf.set_z(0.1);
	bf.set_confidence(0.8);
	bf.set_sigma(0.2);
	bf.set_dist(hypot(bf.x(), bf.y()));
    input.mutable_omnivision_balls()->Add()->CopyFrom(bf);;

    config_MRA_logger();

    // start in middle, expect turn to left
    std::string stateInStr = MRA::convert_proto_to_json_str(state);
    int error_value = m.tick(timestamp, input, params, state, output, local);

//    std::string inputStr = MRA::convert_proto_to_json_str(input);
//    std::string paramsStr = MRA::convert_proto_to_json_str(params);
//    std::string stateOutStr = MRA::convert_proto_to_json_str(state);
//    std::string outputStr = MRA::convert_proto_to_json_str(output);
//    std::string localStr = MRA::convert_proto_to_json_str(local);
//    printf("timestamp: %s\n", google::protobuf::util::TimeUtil::ToString(timestamp).c_str());
//    printf("input: %s\n", inputStr.c_str());
//    printf("params: %s\n", paramsStr.c_str());
//    printf("state(out): %s\n", stateOutStr.c_str());
//    printf("local(in): %s\n", localStr.c_str());
//    printf("output: %s\n", outputStr.c_str());

	// second tick
    input.set_ts(1.1);
    input.mutable_omnivision_balls(0)->set_x(1.0);
    input.mutable_omnivision_balls(0)->set_y(2.0);
    input.mutable_omnivision_balls(0)->set_z(0.1);
    input.mutable_omnivision_balls(0)->set_confidence(0.8);
    input.mutable_omnivision_balls(0)->set_sigma(0.2);
    input.mutable_omnivision_balls(0)->set_dist(hypot(bf.x(), bf.y()));

    // start in middle, expect turn to left
    stateInStr = MRA::convert_proto_to_json_str(state);
    error_value = m.tick(timestamp, input, params, state, output, local);
//    inputStr = MRA::convert_proto_to_json_str(input);
//    paramsStr = MRA::convert_proto_to_json_str(params);
//    stateOutStr = MRA::convert_proto_to_json_str(state);
//    outputStr = MRA::convert_proto_to_json_str(output);
//    localStr = MRA::convert_proto_to_json_str(local);
//    printf("timestamp: %s\n", google::protobuf::util::TimeUtil::ToString(timestamp).c_str());
//    printf("input: %s\n", inputStr.c_str());
//    printf("params: %s\n", paramsStr.c_str());
//    printf("state(in): %s\n", stateInStr.c_str());
//    printf("state(out): %s\n", stateOutStr.c_str());
//    printf("local(in): %s\n", localStr.c_str());
//    printf("output: %s\n", outputStr.c_str());



    // Asserts for turn from middle to left position
    EXPECT_EQ(error_value, 0);
}

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

