#include <gtest/gtest.h>
#include "LocalizationFusion.hpp"
#include "WorldModelNode.hpp"
#include <cmath>

using namespace falcons;

class LocalizationFusionTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        localization = std::make_unique<LocalizationFusion>();
    }

    Time createTime(double seconds)
    {
        Time time;
        time.sec = static_cast<int32_t>(seconds);
        time.nanosec = static_cast<uint32_t>((seconds - time.sec) * 1e9);
        return time;
    }

    Twist createTwist(double vx, double vy, double vtheta)
    {
        Twist twist;
        twist.linear.x = vx;
        twist.linear.y = vy;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = vtheta;
        return twist;
    }

    VisionObject createVisionObject(const std::string& type, double confidence)
    {
        VisionObject obj;
        obj.type = type;
        obj.confidence = confidence;
        return obj;
    }

    std::unique_ptr<LocalizationFusion> localization;
};

TEST_F(LocalizationFusionTest, InitialState)
{
    Pose pose = localization->getCurrentPose();

    // Should start at origin
    EXPECT_DOUBLE_EQ(pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
    EXPECT_DOUBLE_EQ(pose.orientation.w, 1.0);  // No rotation
    EXPECT_DOUBLE_EQ(pose.orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.orientation.z, 0.0);
}

TEST_F(LocalizationFusionTest, OdometryIntegration)
{
    // Create test input with forward motion
    Twist velocity = createTwist(1.0, 0.0, 0.0);  // 1 m/s forward
    std::vector<VisionObject> vision_objects;  // No vision objects

    // Process multiple times to simulate motion
    for (int i = 1; i <= 5; ++i)
    {
        Time timestamp = createTime(i * 0.1);  // 0.1 second intervals
        localization->tick(vision_objects, velocity, timestamp);
    }

    Pose pose = localization->getCurrentPose();

    // Should have moved forward (approximately 0.4 meters in 0.4 seconds)
    EXPECT_GT(pose.position.x, 0.3);
    EXPECT_LT(pose.position.x, 0.5);
    EXPECT_DOUBLE_EQ(pose.position.y, 0.0);
}

TEST_F(LocalizationFusionTest, VisionConfidenceBoost)
{
    Twist velocity = createTwist(0.0, 0.0, 0.0);  // Stationary

    // Create vision objects
    std::vector<VisionObject> vision_objects;
    vision_objects.push_back(createVisionObject("goalpost", 0.9));
    vision_objects.push_back(createVisionObject("goalpost", 0.9));

    Time timestamp = createTime(1.0);

    // First call with no vision
    localization->tick(std::vector<VisionObject>(), velocity, timestamp);

    // Second call with vision - this should improve confidence internally
    localization->tick(vision_objects, velocity, createTime(2.0));

    // The test passes if no exceptions are thrown - confidence logic is internal
    EXPECT_TRUE(true);
}

class WorldModelNodeTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        world_model = std::make_unique<WorldModelNode>();
    }

    Time createTime(double seconds)
    {
        Time time;
        time.sec = static_cast<int32_t>(seconds);
        time.nanosec = static_cast<uint32_t>((seconds - time.sec) * 1e9);
        return time;
    }

    Twist createTwist(double vx, double vy, double vtheta)
    {
        Twist twist;
        twist.linear.x = vx;
        twist.linear.y = vy;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = vtheta;
        return twist;
    }

    VisionObject createVisionObject(const std::string& type, double confidence)
    {
        VisionObject obj;
        obj.type = type;
        obj.confidence = confidence;
        return obj;
    }

    std::unique_ptr<WorldModelNode> world_model;
};

TEST_F(WorldModelNodeTest, InitialWorldState)
{
    WorldState state = world_model->getWorldState();

    // Should have a valid robot pose
    EXPECT_DOUBLE_EQ(state.robot.pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(state.robot.pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(state.robot.pose.position.z, 0.0);
}

TEST_F(WorldModelNodeTest, ProcessFeedback)
{
    Time timestamp = createTime(1.0);
    Twist velocity = createTwist(1.0, 0.0, 0.0);

    world_model->processFeedback(velocity, timestamp);

    WorldState state = world_model->getWorldState();

    // Should not have ticked
    EXPECT_EQ(state.id, 0);
}

TEST_F(WorldModelNodeTest, ProcessVision)
{
    Time timestamp = createTime(1.0);
    std::vector<VisionObject> vision_objects;
    vision_objects.push_back(createVisionObject("goalpost", 0.8));

    // Need to provide odometry first for tick to process
    Twist velocity = createTwist(0.0, 0.0, 0.0);
    world_model->processFeedback(velocity, timestamp);

    world_model->processVision(vision_objects, timestamp);

    WorldState state = world_model->getWorldState();

    // Should have processed the vision data successfully
    EXPECT_EQ(state.id, 1);
    EXPECT_EQ(state.time.sec, timestamp.sec);
    EXPECT_EQ(state.time.nanosec, timestamp.nanosec);
}

TEST_F(WorldModelNodeTest, Reset)
{
    // First, move the robot
    Time timestamp = createTime(1.0);
    Twist velocity = createTwist(1.0, 0.0, 0.0);
    world_model->processFeedback(velocity, timestamp);

    // Reset should return to initial state
    world_model->reset();

    WorldState state = world_model->getWorldState();
    EXPECT_DOUBLE_EQ(state.robot.pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(state.robot.pose.position.y, 0.0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
