// Include testframework
#include "gtest/gtest.h"
#include "gmock/gmock.h"
using namespace ::testing;

// System under test:
#include "intersect.hpp"
using namespace MRA::Geometry;

TEST(MRAGeometryIntersectTest, IntersectSinglePoint)
{
    // Arrange
    MRA::Geometry::Point p1{0, 0};
    MRA::Geometry::Point p2{1, 1};
    MRA::Geometry::Point q1{0, 1};
    MRA::Geometry::Point q2{1, 0};
    MRA::Geometry::Point intersection;

    // Act
    int result = intersect(p1, p2, q1, q2, false, &intersection);

    // Assert
    EXPECT_EQ(result, 1);
    EXPECT_EQ(intersection.x, 0.5);
    EXPECT_EQ(intersection.y, 0.5);
}

TEST(MRAGeometryIntersectTest, IntersectNoPoint)
{
    // Arrange
    MRA::Geometry::Point p1{0, 0};
    MRA::Geometry::Point p2{1, 1};
    MRA::Geometry::Point q1{0, 2};
    MRA::Geometry::Point q2{1, 3};
    MRA::Geometry::Point intersection;

    // Act
    int result = intersect(p1, p2, q1, q2, false, &intersection);

    // Assert
    EXPECT_EQ(result, 0);
}

TEST(MRAGeometryIntersectTest, IntersectPartialOverlap)
{
    // Arrange
    MRA::Geometry::Point p1{0, 0};
    MRA::Geometry::Point p2{2, 4};
    MRA::Geometry::Point q1{1, 2};
    MRA::Geometry::Point q2{3, 6};
    MRA::Geometry::Point intersection;

    // Act
    int result = intersect(p1, p2, q1, q2, false, &intersection);

    // Assert
    EXPECT_EQ(result, 2);
}

TEST(MRAGeometryIntersectTest, DegenerateLines)
{
    // Arrange
    MRA::Geometry::Point p1{0, 0};
    MRA::Geometry::Point p2{0, 0};
    MRA::Geometry::Point q1{0, 0};
    MRA::Geometry::Point q2{0, 0};
    MRA::Geometry::Point intersection;

    // Act
    int result = intersect(p1, p2, q1, q2, false, &intersection);

    // Assert
    EXPECT_EQ(result, 3);
}

TEST(MRAGeometryIntersectTest, IntersectFullLines)
{
    // Arrange
    MRA::Geometry::Point p1{0, 0};
    MRA::Geometry::Point p2{1, 1};
    MRA::Geometry::Point q1{3, 1};
    MRA::Geometry::Point q2{4, 0};
    MRA::Geometry::Point intersection;

    // Act
    int result = intersect(p1, p2, q1, q2, true, &intersection);

    // Assert
    EXPECT_EQ(result, 1);
    EXPECT_EQ(intersection.x, 2.0);
    EXPECT_EQ(intersection.y, 2.0);
}

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

