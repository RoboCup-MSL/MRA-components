// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests

#include <cstdint>

#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_factory.hpp"
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdexcept>
#include <random>

#include "angles.hpp"

using namespace ::testing;

// System under test:
#include "RobotsportsLocalBallTracking.hpp"
#include "logging.hpp"
using namespace MRA;
using namespace std;

// Basic tick shall run OK and return error_value 0.
TEST(RobotsportsLocalBallTrackingTest, basicTick)
{
    MRA_TRACE_TEST_FUNCTION();
    std::string testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();

    // Arrange
    auto m = RobotsportsLocalBallTracking::RobotsportsLocalBallTracking();

    // Act
    int error_value = m.tick();

    // Assert
    EXPECT_EQ(error_value, 0);
}


void execute_ball_traject_from_csv_file(std::string csv_filename, double confidence, double sigma)
{
    auto m = RobotsportsLocalBallTracking::RobotsportsLocalBallTracking();
    auto output = RobotsportsLocalBallTracking::Output();
    auto state = RobotsportsLocalBallTracking::State();
    auto local = RobotsportsLocalBallTracking::Local();
    auto params = m.defaultParams();
    int error_value = 0;

    ifstream inputFile(csv_filename);
    if (not inputFile.is_open()) {
       std::cerr << "inputfile could not be opened: " << csv_filename << std::endl;
       return;
    }

    std::vector<std::vector<double>> input_data;
    string line;
    // skip first line
    getline(inputFile, line);

    while (getline(inputFile, line)) {
      std::stringstream ss(line);
      double value1, value2, value3;

      if (not (ss >> value1 >> value2 >> value3)) {
          std::cerr << "error reading line: " << line << std::endl;
          continue;
      }
      input_data.push_back({value1, value2, value3});
    }
    inputFile.close();

    // process the readed data
    for (const auto& row : input_data)
    {
        auto input = RobotsportsLocalBallTracking::Input();
        input.Clear();
        google::protobuf::Timestamp timestamp = google::protobuf::util::TimeUtil::MillisecondsToTimestamp(row[0] * 1000);
        auto  candidate = MRA::Datatypes::BallCandidate();
        candidate.mutable_pose_fcs()->set_x(row[1]);
        candidate.mutable_pose_fcs()->set_y(row[2]);
        candidate.set_confidence(confidence);
        candidate.set_sigma(sigma);
        candidate.mutable_timestamp()->CopyFrom(timestamp);
        candidate.set_source(1);
        input.mutable_candidates()->Add()->CopyFrom(candidate);


        error_value = m.tick(timestamp, input, params, state, output, local);
        MRA_LOG_DEBUG("diagnostics: %s", MRA::convert_proto_to_json_str(local).c_str());
        MRA_LOG_DEBUG("state: %s", MRA::convert_proto_to_json_str(state).c_str());
        // Asserts for turn from middle to left position
        EXPECT_EQ(error_value, 0);
    }
}

TEST(RobotsportsLocalBallTrackingTest, omnivisionDataTest)
{
    MRA_TRACE_TEST_FUNCTION();
    execute_ball_traject_from_csv_file("./testdata/robotsports_omnivision.csv", 0.8, 0.5);
}

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();

    return r;
}

