// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests


// Include testframework
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_factory.hpp"
using namespace ::testing;

// Other includes
#include <fstream>
#include <opencv2/opencv.hpp>
#include "opencv_utils.hpp"

// System under test:
#include "FalconsLocalizationVision.hpp"
#include "fit.hpp" // internal actually
#include "floor.hpp" // internal actually
#include "solver.hpp" // internal actually
using namespace MRA;

// Basic tick shall run OK and return error_value 0.
TEST(FalconsLocalizationVisionTest, basicTick)
{
    MRA_TRACE_TEST_FUNCTION();
    // Arrange
    auto m = FalconsLocalizationVision::FalconsLocalizationVision();

    // Act
    int error_value = m.tick();

    // Assert
    EXPECT_EQ(error_value, 0);
}

// Reference floor generation
TEST(FalconsLocalizationVisionTest, referenceFloor)
{
    MRA_TRACE_TEST_FUNCTION();
    // The floor is stored in state.
    // A pretty version with grid lines and input pixels (if present, not in this test)
    // is stored in local, but only if debug is enabled.
    // To visually inspect it, set the following boolean and run afterwards:
    //    plot.py -c /tmp/floorDump.bin
    std::string dumpFileName = "/tmp/floorDump.bin";
    bool exportForPlot = true;

    // Arrange
    auto m = FalconsLocalizationVision::FalconsLocalizationVision();
    auto input = FalconsLocalizationVision::Input();
    auto output = FalconsLocalizationVision::Output();
    auto state = FalconsLocalizationVision::State();
    auto local = FalconsLocalizationVision::Local();
    auto params = m.defaultParams(); // official MSL field definition, should not change too often ;)
    // use high resolution
    params.mutable_solver()->set_pixelspermeter(80);
    // disable blur
    params.mutable_solver()->mutable_blur()->set_factor(0.0);
    // optional debug mode
    params.set_debug(exportForPlot);

    // Act
    int error_value = m.tick(input, params, state, output, local);

    // Inspect the resulting floor
    cv::Mat referenceFloor;
    MRA::OpenCVUtils::deserializeCvMat(state.referencefloor(), referenceFloor);
    int pixel_count = cv::countNonZero(referenceFloor);

    // debug dump?
    if (exportForPlot)
    {
        std::ofstream dump(dumpFileName);
        local.floor().SerializeToOstream(&dump);
        dump.close();
        std::cout << "referenceFloor written to dump file " << dumpFileName << std::endl;
    }

    // Assert
    EXPECT_EQ(error_value, 0);
    EXPECT_EQ(pixel_count, 117051);
}

// Template for testing calculation/scoring function - 0.0 is perfect, 1.0 is worst
double FalconsLocalizationVisionTestCalc(std::vector<cv::Point2f> const &points, double x, double y, double rz)
{
    int n = points.size();
    MRA_TRACE_FUNCTION_INPUTS(n, x, y, rz);
    // Arrange
    auto m = FalconsLocalizationVision::FalconsLocalizationVision();
    auto params = m.defaultParams();
    FalconsLocalizationVision::Solver solver;
    solver.configure(params);
    cv::Mat referenceFloor = solver.createReferenceFloorMat(false);
    float ppm = params.solver().pixelspermeter();
    FalconsLocalizationVision::FitFunction fit(referenceFloor, points, ppm);
    // Act
    double pose[3] = {x, y, rz};
    double score = fit.calc(pose);
    MRA_TRACE_FUNCTION_OUTPUT(score);
    return score;
};

std::vector<cv::Point2f> makeFieldPoints(double szx, double szy)
{
    MRA_TRACE_FUNCTION_INPUTS(szx, szy);
    std::vector<cv::Point2f> result;
    double x = 0.5 * szx - 0.05; // inwards a bit, to correct for linewidth
    double y = 0.5 * szy - 0.05; // inwards a bit, to correct for linewidth
    result.push_back(cv::Point2f(0.0, 0.0));
    result.push_back(cv::Point2f(x, 0.0));
    result.push_back(cv::Point2f(-x, 0.0));
    result.push_back(cv::Point2f(0.0, y));
    result.push_back(cv::Point2f(0.0, -y));
    int n = result.size();
    MRA_TRACE_FUNCTION_OUTPUTS(n);
    return result;
}

std::vector<cv::Point2f> makeCirclePoints(double radius, bool diagonal)
{
    MRA_TRACE_FUNCTION_INPUTS(radius, diagonal);
    std::vector<cv::Point2f> result;
    double r = radius - 0.05; // inwards a bit, to correct for linewidth
    result.push_back(cv::Point2f(r, 0.0));
    result.push_back(cv::Point2f(-r, 0.0));
    result.push_back(cv::Point2f(0.0, r));
    result.push_back(cv::Point2f(0.0, -r));
    if (diagonal) {
        double s = sqrt(radius) - 0.05; // inwards a bit, to correct for linewidth
        result.push_back(cv::Point2f(s, s));
        result.push_back(cv::Point2f(s, -s));
        result.push_back(cv::Point2f(-s, s));
        result.push_back(cv::Point2f(-s, -s));
    }
    int n = result.size();
    MRA_TRACE_FUNCTION_OUTPUTS(n);
    return result;
}

TEST(FalconsLocalizationVisionTest, calc000small)
{
    MRA_TRACE_TEST_FUNCTION();
    // Arrange
    std::vector<cv::Point2f> rcsLinePoints = makeFieldPoints(12.0, 18.0); // legacy "small" field

    // Act
    double score = FalconsLocalizationVisionTestCalc(rcsLinePoints, 0.0, 0.0, 0.0);

    // Assert
    EXPECT_EQ(score, 0.4); // 3 of 5 points hit, 60 %score, where 0.0 is perfect (minimization calc)
}

TEST(FalconsLocalizationVisionTest, calc000default)
{
    MRA_TRACE_TEST_FUNCTION();
    // Arrange
    std::vector<cv::Point2f> rcsLinePoints = makeFieldPoints(14.0, 22.0); // default large field

    // Act
    double score = FalconsLocalizationVisionTestCalc(rcsLinePoints, 0.0, 0.0, 0.0);

    // Assert
    EXPECT_EQ(score, 0.0); // 100 %score, 0.0 is perfect
}

// As calc000default, but with rotation rz=pi, should lead to same result because of field symmetry
TEST(FalconsLocalizationVisionTest, calc00Rz)
{
    MRA_TRACE_TEST_FUNCTION();
    // Arrange
    std::vector<cv::Point2f> rcsLinePoints = makeFieldPoints(14.0, 22.0); // default large field

    // Act
    double score = FalconsLocalizationVisionTestCalc(rcsLinePoints, 0.0, 0.0, M_PI);

    // Assert
    EXPECT_EQ(score, 0.0); // 100 %score, 0.0 is perfect
}

// As calc000default, but with rotation rz=0.5*pi, should lead to reduced fit result
TEST(FalconsLocalizationVisionTest, calc00Rz2)
{
    MRA_TRACE_TEST_FUNCTION();
    // Arrange
    std::vector<cv::Point2f> rcsLinePoints = makeFieldPoints(14.0, 22.0); // default large field

    // Act
    double score = FalconsLocalizationVisionTestCalc(rcsLinePoints, 0.0, 0.0, 0.5*M_PI);

    // Assert
    EXPECT_EQ(score, 0.8); // 1 of 5 points hit (only center point), 20 %score
}

TEST(FalconsLocalizationVisionTest, calc000circle)
{
    MRA_TRACE_TEST_FUNCTION();
    // Arrange
    std::vector<cv::Point2f> rcsLinePoints = makeCirclePoints(2.0, true);

    // Act
    double score = FalconsLocalizationVisionTestCalc(rcsLinePoints, 0.0, 0.0, 0.0);

    // Assert
    EXPECT_EQ(score, 0.0);
}

TEST(FalconsLocalizationVisionTest, calcOffset1Circle)
{
    MRA_TRACE_TEST_FUNCTION();
    // Arrange
    std::vector<cv::Point2f> rcsLinePoints = makeCirclePoints(2.0, true);

    // Act
    double score = FalconsLocalizationVisionTestCalc(rcsLinePoints, 1.0, 0.0, 0.0);

    // Assert
    EXPECT_EQ(score, 0.75); // 6/8 miss
}

TEST(FalconsLocalizationVisionTest, calcOffset2Circle)
{
    MRA_TRACE_TEST_FUNCTION();
    // Arrange
    std::vector<cv::Point2f> rcsLinePoints = makeCirclePoints(2.0, true);

    // Act
    double score = FalconsLocalizationVisionTestCalc(rcsLinePoints, 0.0, 2.0, 0.0);

    // Assert
    EXPECT_EQ(score, 0.875); // 7/8 miss
}

/*// Test core fit scoring function: fitting a reference field with itself should yield a perfect result
TEST(FalconsLocalizationVisionTest, perfectFit)
{
    // Arrange
    auto m = FalconsLocalizationVision::FalconsLocalizationVision();
    auto input = FalconsLocalizationVision::Input();
    auto output = FalconsLocalizationVision::Output();
    auto state = FalconsLocalizationVision::State();
    auto local = FalconsLocalizationVision::Local();
    auto params = m.defaultParams();

    // Act - part 1: get reference floor
    int error_value = m.tick(input, params, state, output, local);
    EXPECT_EQ(error_value, 0);
    cv::Mat referenceFloor;
    MRA::OpenCVUtils::deserializeCvMat(state.referencefloor(), referenceFloor);

    // Act - part 2: score reference floor against itself
    MRA::FalconsLocalizationVision::FitFunction fit(referenceFloor, referenceFloor, params.solver().pixelspermeter());
    double overlapScore = fit.calcOverlap(referenceFloor, referenceFloor);

    // Assert
    EXPECT_EQ(overlapScore, 1.0);
}*/

// Test core fit scoring function: choose some pixels which match perfect field definition
// This is data driven -> json test case
TEST(FalconsLocalizationVisionTest, jsonTest1PerfectFit)
{
    MRA_TRACE_TEST_FUNCTION();
    auto output = TestFactory::run_testvector<FalconsLocalizationVision::FalconsLocalizationVision>(std::string("components/falcons/localization_vision/testdata/test1_perfect_fit.json"));
}
TEST(FalconsLocalizationVisionTest, jsonTest2ShiftXY)
{
    MRA_TRACE_TEST_FUNCTION();
    auto output = TestFactory::run_testvector<FalconsLocalizationVision::FalconsLocalizationVision>(std::string("components/falcons/localization_vision/testdata/test2_shift_xy.json"));
}

// Integration bugfixes
TEST(FalconsLocalizationVisionTest, jsonTest3GrabsR5BadInit)
{
    MRA_TRACE_TEST_FUNCTION();
    auto output = TestFactory::run_testvector<FalconsLocalizationVision::FalconsLocalizationVision>(std::string("components/falcons/localization_vision/testdata/test3_grabs_r5_20191219_210335_bad_init.json"));
}

int main(int argc, char **argv)
{
    // Verify that the version of the library that we linked against is
    // compatible with the version of the headers we compiled against.
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

