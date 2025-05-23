// internal
#include "fit.hpp"
#include "threadpool.hpp"

// MRA libraries
#include "opencv_utils.hpp"
#include "logging.hpp"

using namespace MRA::FalconsLocalizationVision;

const double RAD2DEG = 180.0 / M_PI;


// Function to be executed by each thread
void fitThreadFunction(FitCore& _fitCore, cv::Mat const &referenceFloor, std::vector<cv::Point2f> const &rcsLinePoints, Tracker& tr) {
    FitResult fr = _fitCore.run(referenceFloor, rcsLinePoints, tr.guess, tr.step);
    tr.fitResult = fr.pose;
    tr.fitValid = fr.valid;
    tr.fitScore = fr.score;
    tr.fitPath = fr.path;
}


void FitAlgorithm::run(cv::Mat const &referenceFloor, std::vector<cv::Point2f> const &rcsLinePoints, std::vector<Tracker> &trackers)
{
    int num_threads = std::min(1, settings.numextrathreads());
#ifdef MRA_LOGGING_ENABLED
    int num_trackers_before = trackers.size();
    MRA_TRACE_FUNCTION_INPUTS(num_trackers_before, num_threads);
#endif

    // multithreaded execution (main thread idle)
    {
        ThreadPool threads(num_threads);
        for (auto &tr : trackers)
        {
            threads.enqueue(fitThreadFunction, std::ref(_fitCore), std::cref(referenceFloor), std::cref(rcsLinePoints), std::ref(tr));
        }
        // wait via ~ThreadPool
    }

#ifdef MRA_LOGGING_ENABLED
    int num_trackers_after = trackers.size();
    MRA_TRACE_FUNCTION_OUTPUTS(num_trackers_after);
#endif
}

FitResult FitCore::run(cv::Mat const &referenceFloor, std::vector<cv::Point2f> const &rcsLinePoints, MRA::Geometry::Pose const &guess, MRA::Geometry::Pose const &step)
{
    MRA::Datatypes::Pose guess_ = guess;
    MRA::Datatypes::Pose step_ = step;
#ifdef MRA_LOGGING_ENABLED
    int numpoints = rcsLinePoints.size();
    MRA_TRACE_FUNCTION_INPUTS(numpoints, guess_, step_);
#endif
    FitResult result;

    // sanity checks
    if (step.x < 0.01 or step.y < 0.01 or step.rz < 0.01)
    {
        throw std::runtime_error("bad step values");
    }

    // create solver
    auto cvSolver = cv::DownhillSolver::create();

    // configure solver
    cv::Ptr<FitFunction> f = new FitFunction(referenceFloor, rcsLinePoints, settings.pixelspermeter(), settings.linepoints().penaltyoutsidefield());
    cvSolver->setFunction(f);
    cv::Mat stepVec = (cv::Mat_<double>(3, 1) << step.x, step.y, step.rz);
    cvSolver->setInitStep(stepVec);
    cv::TermCriteria criteria = cvSolver->getTermCriteria();
    criteria.maxCount = settings.maxcount();
    criteria.epsilon = settings.epsilon();
    cvSolver->setTermCriteria(criteria);

    // set initial guess
    cv::Mat vec = (cv::Mat_<double>(1, 3) << guess.x, guess.y, guess.rz);

    // run solver, result will be in 'vec'
    result.score = cvSolver->minimize(vec);

    // store result
    float scoreThreshold = settings.scoring().thresholdvalid();
    result.valid = result.score <= scoreThreshold;
    result.pose.x = (vec.at<double>(0, 0));
    result.pose.y = (vec.at<double>(0, 1));
    result.pose.rz = (vec.at<double>(0, 2));
    result.path = f->getPath();

    // tracing
    MRA::Datatypes::Pose result_pose = result.pose;
    MRA_TRACE_FUNCTION_OUTPUTS(result.valid, result.score, result_pose);
    return result;
}

FitFunction::FitFunction(cv::Mat const &referenceFloor, std::vector<cv::Point2f> const &rcsLinePoints, float ppm, float penaltyOutsideField)
{
    MRA_TRACE_FUNCTION();
    _ppm = ppm;
    _penaltyOutsideField = penaltyOutsideField;
    _referenceFloor = referenceFloor;
    _rcsLinePoints = rcsLinePoints;
    // count pixels for normalization, prevent division by zero when no linepoints present (yet)
    _rcsLinePointsPixelCount = rcsLinePoints.size();
    MRA_TRACE_FUNCTION_OUTPUT(_rcsLinePointsPixelCount);
}

double FitFunction::calcOverlap(cv::Mat const &m1, cv::Mat const &m2) const
{
    MRA_TRACE_FUNCTION();
    // calculate the intersection of the white pixels using bitwise AND operation
    cv::Mat overlapMask;
    cv::bitwise_and(m1, m2, overlapMask);
    // calculate score, normalize on the value stored at construction time
    // (which runtime is passed as m2 after transformation)
    double result = static_cast<double>(countNonZero(overlapMask)) / _rcsLinePointsPixelCount;
    // clip score & confidence into [0.0, 1.0]
    result = std::max(0.0, std::min(1.0, result));
    MRA_TRACE_FUNCTION_OUTPUT(result);
    return result;
}

cv::Mat FitFunction::transform3dof(cv::Mat const &m, double x, double y, double rz) const
{
    MRA_TRACE_FUNCTION();
    cv::Mat result;
    // create a transformation matrix
    cv::Mat transformationMatrix = cv::getRotationMatrix2D(cv::Point2f(0.5 * m.cols, 0.5 * m.rows), rz * RAD2DEG, 1.0);
    transformationMatrix.at<double>(0, 2) += y * _ppm; // flip xy, cv::Mat is landscape mode
    transformationMatrix.at<double>(1, 2) += x * _ppm; // flip xy, cv::Mat is landscape mode
    // apply the transformation
    cv::warpAffine(m, result, transformationMatrix, m.size());
    return result;
}

std::vector<cv::Point2f> FitFunction::transformPoints(const std::vector<cv::Point2f> &points, cv::Mat const tmat) const
{
#ifdef MRA_LOGGING_ENABLED
    int n = points.size();
    MRA_TRACE_FUNCTION_INPUTS(n, tmat);
#endif

    // Nothing to do?
    if (points.size() == 0)
    {
        return points;
    }

    // Transformation matrices
    // RCS: robot coordinate system
    // FCS: field coordinate system
    // PCS: pixel coordinate system, applies to (reference) floor cv::Mat

    cv::Mat transformationMatrix33 = transformationMatrixFCS2PCS() * tmat;
    cv::Mat transformationMatrix32 = transformationMatrix33(cv::Rect(0, 0, 3, 2));

    // Transform
    std::vector<cv::Point2f> transformedPoints;
    cv::transform(points, transformedPoints, transformationMatrix32);

    MRA_TRACE_FUNCTION_OUTPUTS(transformationMatrix33, transformationMatrix32); // not really an output, but ok
    return transformedPoints;
}

std::vector<MRA::Geometry::Pose> &FitFunction::getPath()
{
    return _fitpath;
}

double FitFunction::calc(const double *v) const
{
    double x = v[0];
    double y = v[1];
    double rz = v[2];
    MRA_TRACE_FUNCTION_INPUTS(x, y, rz);
    double score = 0.0;
    int fieldPointsAmount = 0;
    int fieldPointsOutsideField = 0;
    std::vector<cv::Point2f> transformed = transformPoints(_rcsLinePoints, transformationMatrixRCS2FCS(x, y, rz));
    for (size_t i = 0; i < _rcsLinePoints.size(); ++i)
    {
        int pixelX = static_cast<int>(transformed[i].x);
        int pixelY = static_cast<int>(transformed[i].y);
        float s = 0.0;
        // Check if the pixel is within the image bounds
        if (pixelX >= 0 && pixelX < _referenceFloor.cols && pixelY >= 0 && pixelY < _referenceFloor.rows)
        {
            // Look up pixel intensity in _referenceFloor and accumulate the score
            s = static_cast<float>(_referenceFloor.at<uchar>(pixelY, pixelX)) / 255.0;
            score += s; // max 1.0 per pixel
            fieldPointsAmount++;
        }
        else
        {
            fieldPointsOutsideField++;
        }
        MRA_LOG_DEBUG("calc %3d   rx=%8.3f  ry=%8.3f  px=%4d py=%4d  s=%6.2f", (int)i, _rcsLinePoints[i].x, _rcsLinePoints[i].y, (int)(pixelX), (int)(pixelY), s);
    }
    _fitpath.push_back(MRA::Geometry::Pose(x, y, rz));
    // normalization to 0..1 where 0 is good (minimization)
    double result = 1.0 - score / _rcsLinePointsPixelCount;

    // make the score slightly worse for each pixels that is outside the field (to prevent a good score while most pixels are outside the field)
    // a good lock has a score of less then 0.050
    // a bad lock has a score of higher then 0.175
    // 30 points outside the field should result result in a worse score
    // so set 30 points outside the field to a score increase of 0.15, that means each point adds 0.005 to the score
    // TODO: double check this with the goaly
    result += fieldPointsOutsideField * _penaltyOutsideField;

    MRA_TRACE_FUNCTION_OUTPUTS(result, fieldPointsAmount, fieldPointsOutsideField);
    return result;
}

cv::Mat FitFunction::transformationMatrixRCS2FCS(double x, double y, double rz) const
{
    MRA_TRACE_FUNCTION_INPUTS(x, y, rz);

    // see also MRA::geometry::Position::transformRcsToFcs
    // and Floor::pointFcsToPixel

    cv::Mat result = cv::Mat::eye(3, 3, CV_64FC1);
    cv::getRotationMatrix2D(cv::Point2f(x, y), -rz * RAD2DEG, 1.0).copyTo(result.rowRange(0, 2).colRange(0, 3));
    result.at<double>(0, 2) = x;
    result.at<double>(1, 2) = y;

    MRA_TRACE_FUNCTION_OUTPUTS(result);
    return result;
}

cv::Mat FitFunction::transformationMatrixFCS2PCS() const
{
    MRA_TRACE_FUNCTION();

    cv::Mat result = cv::Mat::zeros(3, 3, CV_64FC1);
    result.at<double>(1, 0) = _ppm; // flip xy and scale
    result.at<double>(0, 1) = _ppm;
    result.at<double>(0, 2) = 0.5 * _referenceFloor.cols; // put origin at center (offbyone?)
    result.at<double>(1, 2) = 0.5 * _referenceFloor.rows;

    MRA_TRACE_FUNCTION_OUTPUTS(result);
    return result;
}

