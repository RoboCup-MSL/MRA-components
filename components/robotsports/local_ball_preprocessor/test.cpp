// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests


// Include testframework
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_factory.hpp"
using namespace ::testing;

// System under test:
#include "RobotsportsLocalBallPreprocessor.hpp"
using namespace MRA;

// Basic tick shall run OK and return error_value 0.
TEST(RobotsportsLocalBallPreprocessorTest, basicTick)
{
    // Arrange
    auto m = RobotsportsLocalBallPreprocessor::RobotsportsLocalBallPreprocessor();

    // Act
    int error_value = m.tick();

    // Assert
    EXPECT_EQ(error_value, 0);
}


// Basic tick shall run OK and return error_value 0.
TEST(RobotsportsLocalBallPreprocessorTest, FourMeasurements)
{
    // Arrange
    auto m = RobotsportsLocalBallPreprocessor::RobotsportsLocalBallPreprocessor();
    auto input = RobotsportsLocalBallPreprocessor::Input();
    auto output = RobotsportsLocalBallPreprocessor::Output();
    auto local = RobotsportsLocalBallPreprocessor::LocalType();
    auto state = RobotsportsLocalBallPreprocessor::StateType();
    auto params = m.defaultParams();

    google::protobuf::Timestamp timestamp = google::protobuf::util::TimeUtil::MillisecondsToTimestamp(1000);


    auto front_candidate_1 = MRA::Datatypes::BallCandidate();
    front_candidate_1.mutable_measured_pose_fcs()->set_x(1.0);
    front_candidate_1.mutable_measured_pose_fcs()->set_y(1.0);
    front_candidate_1.mutable_measured_pose_fcs()->set_z(0.1);
    front_candidate_1.set_confidence(0.8);
    front_candidate_1.set_sigma(0.6);
    front_candidate_1.mutable_timestamp()->CopyFrom(timestamp);
    input.mutable_frontcamera_balls()->Add()->CopyFrom(front_candidate_1);

    auto front_candidate_2 = MRA::Datatypes::BallCandidate();
    front_candidate_2.mutable_measured_pose_fcs()->set_x(1.2);
    front_candidate_2.mutable_measured_pose_fcs()->set_y(8.0);
    front_candidate_2.mutable_measured_pose_fcs()->set_z(0.1);
    front_candidate_2.set_confidence(0.1);
    front_candidate_2.set_sigma(0.6);
    front_candidate_2.mutable_timestamp()->CopyFrom(timestamp);
    input.mutable_frontcamera_balls()->Add()->CopyFrom(front_candidate_2);


    auto omnivision_candidate_1 = MRA::Datatypes::BallCandidate();
    omnivision_candidate_1.mutable_measured_pose_fcs()->set_x(1.0);
    omnivision_candidate_1.mutable_measured_pose_fcs()->set_y(1.0);
    omnivision_candidate_1.mutable_measured_pose_fcs()->set_z(0.1);
    omnivision_candidate_1.set_confidence(0.8);
    omnivision_candidate_1.set_sigma(0.46);
    omnivision_candidate_1.mutable_timestamp()->CopyFrom(timestamp);
    input.mutable_omnivision_balls()->Add()->CopyFrom(omnivision_candidate_1);

    auto omnivision_candidate_2 = MRA::Datatypes::BallCandidate();
    omnivision_candidate_2.mutable_measured_pose_fcs()->set_x(1.2);
    omnivision_candidate_2.mutable_measured_pose_fcs()->set_y(8.0);
    omnivision_candidate_2.mutable_measured_pose_fcs()->set_z(0.1);
    omnivision_candidate_2.set_confidence(0.1);
    omnivision_candidate_2.set_sigma(0.4);
    omnivision_candidate_2.mutable_timestamp()->CopyFrom(timestamp);
    input.mutable_omnivision_balls()->Add()->CopyFrom(omnivision_candidate_2);


    // Act
    int error_value = m.tick(timestamp, input, params, state, output, local);

    // Assert
    EXPECT_EQ(error_value, 0);
}




int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

