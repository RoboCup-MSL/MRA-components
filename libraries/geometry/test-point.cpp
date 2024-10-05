// Include testframework
#include "gtest/gtest.h"
#include "gmock/gmock.h"
using namespace ::testing;

// System under test:
#include "point.hpp"
using namespace MRA::Geometry;


TEST(MRAGeometryPointTest, constructorNoArgs)
{
    // Arrange
    auto p = Point();

    // Act

    // Assert
    EXPECT_EQ(p.x, 0.0);
    EXPECT_EQ(p.y, 0.0);
}

TEST(MRAGeometryPointTest, constructorSixArgs)
{
    // Arrange
    auto p = Point(1.0, 2.0);

    // Act

    // Assert
    EXPECT_EQ(p.x, 1.0);
    EXPECT_EQ(p.y, 2.0);
}

TEST(MRAGeometryPointTest, constructorProtobufComplete)
{
    // Arrange
    auto pd = MRA::Datatypes::Point();
    pd.set_x(1.0);
    pd.set_y(2.0);

    // Act
    auto p = Point(pd);

    // Assert
    EXPECT_EQ(p.x, 1.0);
    EXPECT_EQ(p.y, 2.0);
}

TEST(MRAGeometryPointTest, constructorProtobufIncomplete)
{
    // Arrange
    auto pd = MRA::Datatypes::Point();
    pd.set_y(2.0);

    // Act
    auto p = Point(pd);

    // Assert
    EXPECT_EQ(p.x, 0.0);
    EXPECT_EQ(p.y, 2.0);
}

TEST(MRAGeometryPointTest, constructorNoWrapping)
{
    // Arrange
    auto p = Point(111.0, 222.0);

    // Act

    // Assert
    EXPECT_EQ(p.x, 111.0);
    EXPECT_EQ(p.y, 222.0);
}

TEST(MRAGeometryPointTest, size)
{
    // Arrange
    // (make use of tje well-known pythagorean triple)
    auto p = Point(-3.0, 4.0);

    // Act
    float v = p.size();

    // Assert
    EXPECT_EQ(v, 5.0);
}

TEST(MRAGeometryPointTest, operatorPlus)
{
    // Arrange
    auto p1 = Point(1.0, 2.0);
    auto p2 = Point(1.0, 2.0);

    // Act
    auto p = p1 + p2;

    // Assert
    EXPECT_EQ(p.x, 2.0);
    EXPECT_EQ(p.y, 4.0);
}

TEST(MRAGeometryPointTest, operatorPlusAssign)
{
    // Arrange
    auto p = Point(1.0, 2.0);
    auto p2 = Point(1.0, 2.0);

    // Act
    p += p2;

    // Assert
    EXPECT_EQ(p.x, 2.0);
    EXPECT_EQ(p.y, 4.0);
}

TEST(MRAGeometryPointTest, operatorMinus)
{
    // Arrange
    auto p1 = Point(4.0, 6.0);
    auto p2 = Point(1.0, 2.0);

    // Act
    auto p = p1 - p2;

    // Assert
    EXPECT_EQ(p.x, 3.0);
    EXPECT_EQ(p.y, 4.0);
}

TEST(MRAGeometryPointTest, operatorMinusAssign)
{
    // Arrange
    auto p = Point(4.0, 6.0);
    auto p2 = Point(1.0, 2.0);

    // Act
    p -= p2;

    // Assert
    EXPECT_EQ(p.x, 3.0);
    EXPECT_EQ(p.y, 4.0);
}

TEST(MRAGeometryPointTest, operatorTimesScalar)
{
    // Arrange
    auto p1 = Point(1.0, 2.0);

    // Act
    auto p = p1 * 2;

    // Assert
    EXPECT_EQ(p.x, 2.0);
    EXPECT_EQ(p.y, 4.0);
}

TEST(MRAGeometryPointTest, operatorTimesScalarAssign)
{
    // Arrange
    auto p = Point(1.0, 2.0);

    // Act
    p *= 2;

    // Assert
    EXPECT_EQ(p.x, 2.0);
    EXPECT_EQ(p.y, 4.0);
}

TEST(MRAGeometryPointTest, operatorDivideScalar)
{
    // Arrange
    auto p1 = Point(2.0, 4.0);

    // Act
    auto p = p1 / 2;

    // Assert
    EXPECT_EQ(p.x, 1.0);
    EXPECT_EQ(p.y, 2.0);
}

TEST(MRAGeometryPointTest, operatorDivideScalarAssign)
{
    // Arrange
    auto p = Point(2.0, 4.0);

    // Act
    p /= 2;

    // Assert
    EXPECT_EQ(p.x, 1.0);
    EXPECT_EQ(p.y, 2.0);
}

TEST(MRAGeometryPointTest, equals)
{
    // Arrange
    auto point1 = Point(2.0, 4.0);
    auto point2 = Point(-1.0, 8.0);

    auto p1_eq_to_p2 = point1.equals(point2);
    EXPECT_EQ(p1_eq_to_p2, false);

    auto p1_eq_to_p2_large_tol = point1.equals(point2, 1e12);
    EXPECT_EQ(p1_eq_to_p2_large_tol, true);


    auto p1_eq_to_p1 = point1.equals(point1);
    EXPECT_EQ(p1_eq_to_p1, true);
}

TEST(MRAGeometryPointTest, distanceTo)
{
    // Arrange
    auto point1 = Point(2.0, 4.0);
    auto point2 = Point(-1.0, 8.0);
    // Act
    auto dist = point1.distanceTo(point2);

    // Assert
    EXPECT_EQ(dist, 5.0);
}


TEST(MRAGeometryPointTest, toString)
{
    // Arrange
    auto p = Point(1.99999, 4.001);

    // Act
    auto str = p.toString();

    // Assert
    EXPECT_EQ(str, "x: 2.00 y: 4.00");
}


TEST(MRAGeometryPointTest, inproduct)
{
    // Arrange
    auto p1 = Point(-3, -4);
    auto p2 = Point(3, 4);

    // Act
    auto inp = p1.inproduct(p2);

    // Assert
    EXPECT_EQ(inp, -25.0);
}

TEST(MRAGeometryPointTest, angle)
{
    // Arrange
    auto p1 = Point(0, 0);
    auto p2 = Point(1.0, 0);

    // Act
    auto angle = p1.angle(p2);

    // Assert
    EXPECT_NEAR(angle, M_PI, 1e-6);
}

TEST(MRAGeometryPointTest, normalize)
{
    // Arrange
    auto p = Point(5.0, 5.0);

    // Act
    p.normalize();

    // Assert
    EXPECT_NEAR(p.x, 0.5*sqrt(2.0), 1e-9);
    EXPECT_NEAR(p.y, 0.5*sqrt(2.0), 1e-9);
}

TEST(MRAGeometryPointTest, normalizeOfPointWithMagnitudeZero)
{
    // Arrange
    auto p = Point(0.0, 0.0);

    // Act
    ASSERT_THROW(p.normalize(), std::range_error);
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

