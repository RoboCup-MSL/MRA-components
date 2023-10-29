#ifndef _MRA_FALCONS_LOCALIZATION_VISION_FIT_HPP
#define _MRA_FALCONS_LOCALIZATION_VISION_FIT_HPP

#include <opencv2/core/optim.hpp>
#include "geometry.hpp"
#include "FalconsLocalizationVision_datatypes.hpp"
#include "tracker.hpp"


namespace MRA::FalconsLocalizationVision
{


struct FitResult
{
    bool valid = false;
    float score = 0.0;
    MRA::Geometry::Pose pose;
    std::vector<MRA::Geometry::Pose> path;
    bool operator<(FitResult const &other) { return score < other.score; }
}; // struct FitResult


class FitFunction: public cv::MinProblemSolver::Function
{
public:
    FitFunction(cv::Mat const &referenceFloor, std::vector<cv::Point2f> const &rcsLinePoints, float ppm);
	double calc(const double *x) const; // this is the main scoring function to be minimized, x is a tuple (x,y,rz)
    int getDims() const { return 3; }

    // helpers, public for testing purposes and diagnostics
    double calcOverlap(cv::Mat const &m1, cv::Mat const &m2) const;
    cv::Mat transform3dof(cv::Mat const &m, double x, double y, double rz) const; // TODO remove??
    std::vector<cv::Point2f> transformPoints(const std::vector<cv::Point2f> &points, cv::Mat tmat = cv::Mat::eye(3, 3, CV_64FC1)) const;
    cv::Mat transformationMatrixRCS2FCS(double x, double y, double rz) const;
    std::vector<MRA::Geometry::Pose> &getPath();

private:
    cv::Mat _referenceFloor;
    std::vector<cv::Point2f> _rcsLinePoints;
    double _rcsLinePointsPixelCount = 1.0; // for score normalization
    float _ppm; // needed to optimize in FCS instead of pixels
    cv::Mat transformationMatrixFCS2PCS() const;
    mutable std::vector<MRA::Geometry::Pose> _fitpath;
}; // class FitFunction


class FitCore
{
public:
    FitCore() {};
    ~FitCore() {};

    SolverParams settings;
    void configure(SolverParams const &config) { settings.CopyFrom(config); }

    FitResult run(
        cv::Mat const &referenceFloor,      // params translated once (at first tick) to reference floor to fit against, white pixels, potentially blurred
        std::vector<cv::Point2f> const &rcsLinePoints,
        MRA::Geometry::Pose const &guess,   // initial guess for the algorithm, note that the simplex is constructed AROUND it, so somewhere a shift might be needed
        MRA::Geometry::Pose const &step);   // initial step: search region

}; // class FitCore


class FitAlgorithm
{
public:
    FitAlgorithm() {};
    ~FitAlgorithm() {};

    SolverParams settings;
    void configure(SolverParams const &config) { settings.CopyFrom(config); _fitCore.configure(config); }

    void run(
        cv::Mat const &referenceFloor,      // params translated once (at first tick) to reference floor to fit against, white pixels, potentially blurred
        std::vector<cv::Point2f> const &rcsLinePoints,
        std::vector<Tracker> &trackers);    // list of trackers/attempts to run, multithreaded if so configured

private:
    FitCore _fitCore;

}; // class FitAlgorithm


} // namespace MRA::FalconsLocalizationVision

#endif

