#include <gtest/gtest.h>
#include "LocalizationFusion.hpp"
#include "WorldModelNode.hpp"
#include "WorldModelConvert.hpp"
#include <chrono>
#include <cmath>

using namespace falcons;

class LocalizationFusionTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        localization = std::make_unique<LocalizationFusion>();
    }

    std::unique_ptr<LocalizationFusion> localization;
};

TEST_F(LocalizationFusionTest, InitialState)
{
    RobotPose pose = localization->getCurrentPose();

    // Should start at origin with low confidence
    EXPECT_DOUBLE_EQ(pose.pose.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.pose.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.pose.theta, 0.0);
    EXPECT_LT(pose.confidence, 0.5);  // Low initial confidence
}

TEST_F(LocalizationFusionTest, OdometryIntegration)
{
    // Create test input with forward motion
    auto timestamp = std::chrono::system_clock::now();
    OdometryData odometry(timestamp, Velocity2D(1.0, 0.0, 0.0));  // 1 m/s forward
    std::vector<VisionLandmark> landmarks;  // No landmarks

    LocalizationInput input(timestamp, landmarks, odometry);

    // Process multiple times to simulate motion
    for (int i = 0; i < 5; ++i)
    {
        timestamp += std::chrono::milliseconds(100);  // 0.1 second intervals
        input.timestamp = timestamp;
        input.odometry.timestamp = timestamp;

        localization->tick(input);
    }

    RobotPose pose = localization->getCurrentPose();

    // Should have moved forward (approximately 0.5 meters in 0.5 seconds)
    EXPECT_GT(pose.pose.x, 0.3);
    EXPECT_LT(pose.pose.x, 0.7);
    EXPECT_DOUBLE_EQ(pose.pose.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.pose.theta, 0.0);
}

TEST_F(LocalizationFusionTest, VisionConfidenceBoost)
{
    auto timestamp = std::chrono::system_clock::now();
    OdometryData odometry(timestamp, Velocity2D(0.0, 0.0, 0.0));  // Stationary

    // Create vision landmarks with high confidence
    std::vector<VisionLandmark> landmarks;
    landmarks.emplace_back(VisionLandmark::Type::GOALPOST, 0.0, 5.0, 0.9);
    landmarks.emplace_back(VisionLandmark::Type::GOALPOST, M_PI, 5.0, 0.9);

    LocalizationInput input(timestamp, landmarks, odometry);

    RobotPose initial_pose = localization->getCurrentPose();
    localization->tick(input);
    RobotPose updated_pose = localization->getCurrentPose();

    // Confidence should increase when good landmarks are visible
    EXPECT_GT(updated_pose.confidence, initial_pose.confidence);
}

class WorldModelNodeTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        world_model = std::make_unique<WorldModelNode>();
    }

    std::unique_ptr<WorldModelNode> world_model;
};

TEST_F(WorldModelNodeTest, InitialWorldState)
{
    WorldModelState state = world_model->getWorldState();

    // Should have a valid robot pose
    EXPECT_DOUBLE_EQ(state.robot_pose.pose.x, 0.0);
    EXPECT_DOUBLE_EQ(state.robot_pose.pose.y, 0.0);
    EXPECT_DOUBLE_EQ(state.robot_pose.pose.theta, 0.0);
}

TEST_F(WorldModelNodeTest, ProcessFeedback)
{
    auto timestamp = std::chrono::system_clock::now();
    OdometryData odometry(timestamp, Velocity2D(1.0, 0.0, 0.0));

    world_model->processFeedback(odometry);

    WorldModelState state = world_model->getWorldState();

    // Should have updated timestamp and velocity
    EXPECT_EQ(state.timestamp, timestamp);
    EXPECT_NEAR(state.robot_pose.velocity.vx, 1.0, 1e-6);
}

TEST_F(WorldModelNodeTest, ProcessVision)
{
    auto timestamp = std::chrono::system_clock::now();
    std::vector<VisionLandmark> landmarks;
    landmarks.emplace_back(VisionLandmark::Type::GOALPOST, 0.0, 5.0, 0.8);

    // Need to provide odometry first for tick to process
    OdometryData odometry(timestamp, Velocity2D(0.0, 0.0, 0.0));
    world_model->processFeedback(odometry);

    RobotPose initial_pose = world_model->getWorldState().robot_pose;

    world_model->processVision(landmarks, timestamp);

    WorldModelState state = world_model->getWorldState();

    // Confidence should have improved with vision data
    EXPECT_GE(state.robot_pose.confidence, initial_pose.confidence);
}

class WorldModelConvertTest : public ::testing::Test
{
};

TEST_F(WorldModelConvertTest, VisionObjectConversion)
{
    // Create a ROS2 VisionObjects message
    mra_common_msgs::msg::VisionObjects ros_msg;
    ros_msg.robot = 1;
    ros_msg.frame = 100;

    mra_common_msgs::msg::VisionObject obj;
    obj.type = "goalpost";
    obj.azimuth = 0.5;
    obj.distance = 3.0;
    obj.confidence = 0.9;
    ros_msg.objects.push_back(obj);

    // Convert to internal type
    std::vector<VisionLandmark> landmarks = WorldModelConvert::fromVisionObjects(ros_msg);

    ASSERT_EQ(landmarks.size(), 1);
    EXPECT_EQ(landmarks[0].type, VisionLandmark::Type::GOALPOST);
    EXPECT_DOUBLE_EQ(landmarks[0].azimuth, 0.5);
    EXPECT_DOUBLE_EQ(landmarks[0].distance, 3.0);
    EXPECT_NEAR(landmarks[0].confidence, 0.9, 1e-6);  // Use NEAR for floating point
}

TEST_F(WorldModelConvertTest, FeedbackConversion)
{
    // Create a ROS2 Feedback message
    falcons_msgs::msg::Feedback ros_msg;
    ros_msg.velocity.linear.x = 2.0;
    ros_msg.velocity.linear.y = 1.0;
    ros_msg.velocity.angular.z = 0.5;

    // Convert to internal type
    OdometryData odometry = WorldModelConvert::fromFeedback(ros_msg);

    EXPECT_DOUBLE_EQ(odometry.velocity.vx, 2.0);
    EXPECT_DOUBLE_EQ(odometry.velocity.vy, 1.0);
    EXPECT_DOUBLE_EQ(odometry.velocity.vtheta, 0.5);
}

TEST_F(WorldModelConvertTest, WorldStateConversion)
{
    // Create internal WorldModelState
    auto timestamp = std::chrono::system_clock::now();
    RobotPose robot_pose(timestamp, Pose2D(1.0, 2.0, 0.5), Velocity2D(0.5, 0.0, 0.1), 0.8);
    WorldModelState state(timestamp, robot_pose);

    // Convert to ROS2 message
    mra_common_msgs::msg::WorldState ros_msg = WorldModelConvert::toWorldState(state);

    EXPECT_DOUBLE_EQ(ros_msg.robot.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(ros_msg.robot.pose.position.y, 2.0);
    EXPECT_DOUBLE_EQ(ros_msg.robot.velocity.linear.x, 0.5);
    EXPECT_DOUBLE_EQ(ros_msg.robot.velocity.angular.z, 0.1);
    EXPECT_TRUE(ros_msg.robot.active);
    EXPECT_FALSE(ros_msg.robot.human);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
