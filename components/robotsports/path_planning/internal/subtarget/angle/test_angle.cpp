#include <gtest/gtest.h>
#include "dribble_module.hpp" // Include the header for the dribble function

// Test fixture for DribbleData to set up common test data
struct DribbleTest : public ::testing::Test {
    DribbleData d;
};

// Test case for non-zero velocity
TEST_F(DribbleTest, TestNonZeroVelocity) {
    // Test when velocity is non-zero (norm > 1e-6)
    d.setpoint.v << 0.5, 0.5, 0; // Non-zero velocity in the first two elements
    d.setpoint.p << 0, 0, 0;     // Just a placeholder for this test

    double expected_angle = std::atan2(-d.setpoint.v[0], d.setpoint.v[1]);
    ASSERT_NEAR(dribble(d), expected_angle, 1e-6); // Using ASSERT_NEAR for float comparison
}

// Test case for zero velocity
TEST_F(DribbleTest, TestZeroVelocity) {
    // Test when velocity is zero (norm <= 1e-6)
    d.setpoint.v << 0, 0, 0;      // Zero velocity
    d.setpoint.p << 0, 0, 1.5;    // Use 1.5 for p[2] to test default behavior (angle)

    double expected_angle = d.setpoint.p[2]; // Expected to take p[2] as the angle
    ASSERT_EQ(dribble(d), expected_angle);   // Exact equality for direct assignment
}

// Main function to run the tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}