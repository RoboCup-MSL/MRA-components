#include "solver.hpp"
#include "guessing.hpp"

// MRA libraries
#include "geometry.hpp"
#include "opencv_utils.hpp"
#include "logging.hpp"


using namespace MRA::FalconsLocalizationVision;


Solver::Solver()
{
}

Solver::~Solver()
{
}

void Solver::configure(Params const &p)
{
    MRA_TRACE_FUNCTION_INPUTS(p);
    _params = p;
    // check for missing required parameters
    checkParamsValid();
    // configure helper classes
    _floor.configure(_params);
    _fitAlgorithm.configure(_params.solver());
    // trigger re-init
    _reinit = true;
}

void Solver::checkParamsValid() const
{
    MRA_TRACE_FUNCTION();
    // check field model params
    if (_params.model().a() < 1)
    {
        throw std::runtime_error("invalid configuration: field model parameter A (field length in y) too small (got " + std::to_string(_params.model().a()) + ")");
    }
    if (_params.model().b() < 1)
    {
        throw std::runtime_error("invalid configuration: field model parameter B (field width in x) too small (got " + std::to_string(_params.model().b()) + ")");
    }
    if (_params.model().k() < 0.01)
    {
        throw std::runtime_error("invalid configuration: field model parameter K (line width) too small (got " + std::to_string(_params.model().k()) + ")");
    }
    // check solver params
    auto solverParams = _params.solver();
    if (solverParams.maxcount() < 1)
    {
        throw std::runtime_error("invalid configuration: maxCount should be a positive number (got " + std::to_string(solverParams.maxcount()) + ")");
    }
    if (solverParams.actionradius().x() == 0.0)
    {
        throw std::runtime_error("invalid configuration: actionRadius.x too small (got " + std::to_string(solverParams.actionradius().x()) + ")");
    }
    if (solverParams.actionradius().y() == 0.0)
    {
        throw std::runtime_error("invalid configuration: actionRadius.y too small (got " + std::to_string(solverParams.actionradius().y()) + ")");
    }
    if (solverParams.actionradius().rz() == 0.0)
    {
        throw std::runtime_error("invalid configuration: actionRadius.rz too small (got " + std::to_string(solverParams.actionradius().rz()) + ")");
    }
    if (solverParams.linepoints().fit().radiusconstant() < 0.01)
    {
        throw std::runtime_error("invalid configuration: linePoints.fit.radiusConstant too small (got " + std::to_string(solverParams.linepoints().fit().radiusconstant()) + ")");
    }
}

void Solver::setState(State const &s)
{
    MRA_TRACE_FUNCTION_INPUTS(s);
    _state = s;
}

void Solver::setInput(Input const &in)
{
    MRA_TRACE_FUNCTION_INPUTS(in);
    _input = in;
}

cv::Mat Solver::createReferenceFloorMat(bool withBlur) const
{
    MRA_TRACE_FUNCTION_INPUTS(withBlur);
    cv::Mat result;

    // given the configured field (letter model and optional custom shapes),
    // create a CV::mat representation, using a blur factor
    // serialize and store the mat in state, as this should not be recalculated each tick
    // some external python plot tool should be able to plot it

    // determine set of shapes
    std::vector<MRA::Datatypes::Shape> shapes(_params.shapes().begin(), _params.shapes().end());
    if (_params.has_model())
    {
        _floor.letterModelToShapes(_params.model(), shapes); // TODO: move this to MRA::libraries? could be useful elsewhere
    }

    // create cv::Mat such that field is rotated screen-friendly: more columns than rows
    result = _floor.createMat();
    _floor.shapesToCvMat(shapes, result);

    // apply blur
    if (withBlur)
    {
        float blurFactor = _params.solver().blur().factor();
        int blurMaxDepth = _params.solver().blur().maxdepth();
        uchar blurMinValue = _params.solver().blur().minvalue();
        result = _floor.applyBlur(result, blurFactor, blurMaxDepth, blurMinValue);
    }

    return result;
}

void Solver::reinitialize()
{
    MRA_TRACE_FUNCTION();
    // first check the flag
    if (!_reinit) return;

    // check if (re)calculation is needed based on state
    // if not (cache_hit), then deserialize from state and done,
    // if yes then calculate using params
    std::string stateParamsStr, paramsStr;
    _state.params().SerializeToString(&stateParamsStr);
    _params.SerializeToString(&paramsStr);
    bool cache_hit = (stateParamsStr == paramsStr) && _state.has_referencefloor();
    if (cache_hit)
    {
        MRA::OpenCVUtils::deserializeCvMat(_state.referencefloor(), _referenceFloorMat);
        return;
    }

    MRA_LOG_DEBUG("cache miss, creating reference floor");

    // calculate reference floor and store in state as protobuf CvMatProto object for next iteration (via state)
    _referenceFloorMat = createReferenceFloorMat(true);
    MRA::OpenCVUtils::serializeCvMat(_referenceFloorMat, *_state.mutable_referencefloor());

    // store params into state
    _state.mutable_params()->CopyFrom(_params);
}

std::vector<cv::Point2f> Solver::createLinePoints() const
{
    MRA_TRACE_FUNCTION();
    std::vector<cv::Point2f> result;
    for (const Landmark& landmark : _input.landmarks())
    {
        float x = landmark.x();
        float y = landmark.y();
        cv::Point2f lp(x, y);
        result.push_back(lp);
    }
    int n = result.size();
    MRA_TRACE_FUNCTION_OUTPUT(n);
    return result;
}

std::vector<Tracker> Solver::createTrackers() const
{
    MRA_TRACE_FUNCTION();
    std::vector<Tracker> result;

    // load from state
    for (auto const &st: _state.trackers())
    {
        result.push_back(Tracker(_params, st));
    }

    // state trackers are sorted by descending quality, so the first tracker is expected to be the best one
    // given input guess is intended to finetune, taking action radius into account
    // offset the input guess to ensure that it is included,
    // since by default the generated simplex would not hit it
    //     if the step is configured to be (sx,sy,srz) and initial guess is zero
    //     then the initial simplex evaluates at the following simplex:
    //         (-0.5*sx, -0.5*sy, -0.5*srz)
    //         ( 0.5*sx,       0,        0)
    //         (      0,  0.5*sy,        0)
    //         (      0,       0,  0.5*srz)
    if (result.size() == 0)
    {
        result.push_back(Tracker(_params, TrackerState()));
    }
    Tracker &tracker = result.at(0);
    tracker.guess = _input.guess();
    tracker.guess.rz -= 0.5 * tracker.step.rz;

    // run the guesser to add more attempts/trackers
    Guesser g(_params);
    bool initial = (_state.tick() == 0);
    g.run(result, initial);

    return result;
}

void Solver::runFitUpdateTrackers()
{
    MRA_TRACE_FUNCTION();
    // run the fit algorithm (multithreaded, one per tracker) and update trackers
    _fitAlgorithm.run(_referenceFloorMat, _linePoints, _trackers);

    // set _fitResult
    _fitResult.valid = false;
    if (_trackers.size())
    {
        auto tr = _trackers.at(0);
        _fitResult.valid = tr.fitValid;
        _fitResult.pose = tr.fitResult;
        _fitResult.score = tr.fitScore;
        _fitResult.path = tr.fitPath;
        if (_fitResult.valid)
        {
            Candidate c;
            c.mutable_pose()->CopyFrom((MRA::Datatypes::Pose)_fitResult.pose);
            c.set_confidence(_fitResult.score);
            *_output.add_candidates() = c;
        }
    }
}

void Solver::cleanupBadTrackers()
{
    MRA_TRACE_FUNCTION();
}

// TODO move to opencv_utils?
// combine non-black pixels from m_other into m
void combineImages(cv::Mat m, cv::Mat m_other)
{
    MRA_TRACE_FUNCTION();
    CV_Assert(m.size() == m_other.size() && m.type() == CV_8UC3 && m_other.type() == CV_8UC3);
    cv::Mat mask;
    cv::cvtColor(m_other, mask, cv::COLOR_BGR2GRAY);
    cv::threshold(mask, mask, 1, 255, cv::THRESH_BINARY);
    cv::Mat tmp;
    m_other.copyTo(tmp, mask);
    m.setTo(cv::Scalar(0, 0, 0), mask);
    m += tmp;
}

cv::Mat Solver::createDiagnosticsMat() const
{
    MRA_TRACE_FUNCTION();
    // reference floor either with or without blur
    bool withBlur = true;
    cv::Mat referenceFloorMat = withBlur ? _referenceFloorMat : createReferenceFloorMat();

    // create diagnostics floor, upscale to colors
    cv::Mat result;
    cv::cvtColor(referenceFloorMat, result, cv::COLOR_GRAY2BGR);

    // add linepoints with blue/cyan color
    float ppm = _params.solver().pixelspermeter();
    FitFunction ff(referenceFloorMat, _linePoints, ppm);
    std::vector<cv::Point2f> transformed = ff.transformPoints(_linePoints, ff.transformationMatrixRCS2FCS(_fitResult.pose.x, _fitResult.pose.y, _fitResult.pose.rz));
    MRA_LOG_DEBUG("number of transformed points: %d", (int)transformed.size());
    for (const auto &point : transformed) {
        cv::circle(result, cv::Point(point.x, point.y), ppm * _params.solver().linepoints().plot().radius(), cv::Scalar(255, 0, 0), -1);
    }

    // add fit path
    MRA_LOG_DEBUG("number of points in fit path: %d", (int)_fitResult.path.size());
    std::vector<cv::Point2f> pathPoints;
    std::transform(_fitResult.path.begin(), _fitResult.path.end(), std::back_inserter(pathPoints),
                   [](const MRA::Geometry::Pose& obj) { return cv::Point2f(obj.x, obj.y); });
    transformed = ff.transformPoints(pathPoints);
    auto pathColor = _params.solver().pathpoints().color();
    for (const auto &point : transformed) {
        cv::circle(result, cv::Point(point.x, point.y), ppm * _params.solver().pathpoints().radius(), cv::Scalar(pathColor.b(), pathColor.g(), pathColor.r()), -1);
    }

    // show robot as red circle with orientation line
    auto color = cv::Scalar(0, 0, 255);
    int linewidth = 4;
    MRA::Geometry::Position robotPoint(_fitResult.pose);
    MRA::Geometry::Position directionOffset = MRA::Geometry::Position(0.0, 0.4, 0.0); // point towards y direction (where the robot is aiming at)
    MRA::Geometry::Position directionPoint = directionOffset.transformRcsToFcs(robotPoint);
	cv::circle(result, _floor.pointFcsToPixel(robotPoint), ppm * 0.28, color, linewidth);
	cv::circle(result, _floor.pointFcsToPixel(directionPoint), ppm * 0.04, color, linewidth);
	cv::line(result, _floor.pointFcsToPixel(robotPoint), _floor.pointFcsToPixel(directionPoint), color, linewidth);

    // add green grid lines on top (all 1 pixel, so we can clearly see how the field lines are positioned)
    _floor.addGridLines(result, 1.0, cv::Scalar(0, 100, 0)); // 1meter grid: very faint
    _floor.addGridLines(result, 2.0, cv::Scalar(0, 255, 0)); // 2meter grid: bright, more prominent

    return result;
}

void Solver::dumpDiagnosticsMat()
{
    MRA_TRACE_FUNCTION();
    MRA::OpenCVUtils::serializeCvMat(createDiagnosticsMat(), *_diag.mutable_floor());
}

void Solver::manualMode()
{
    MRA_TRACE_FUNCTION();
    float ppm = _params.solver().pixelspermeter();
    // run the core calc() function
    FalconsLocalizationVision::FitFunction fit(_referenceFloorMat, _linePoints, ppm);
    double pose[3] = {_params.solver().manual().pose().x(), _params.solver().manual().pose().y(), _params.solver().manual().pose().rz()};
    double score = fit.calc(pose);
    // copy pose into _fitResult so local.floor will be properly created
    _fitResult.pose.x = pose[0];
    _fitResult.pose.y = pose[1];
    _fitResult.pose.rz = pose[2];
    _fitResult.path = fit.getPath();
    // set output
    Candidate c;
    c.mutable_pose()->CopyFrom((MRA::Datatypes::Pose)_fitResult.pose);
    c.set_confidence(score);
    *_output.add_candidates() = c;
}

int Solver::run()
{
    int tick = _state.tick();
    MRA_TRACE_FUNCTION_INPUTS(tick);
    // try to keep the design as simple as possible: minimize state, trackers over time (that is for worldModel to handle)
    // initially (and maybe also occasionally?) we should perhaps do some kind of grid search
    // that is handled in the FitAlgorithm, also optional multithreading and guessing / search space partitioning

    // the FitCore is a single fit operation (which uses opencv Downhill Simplex solver):
    // fit given white pixels and initial guess to the reference field

    // create or get the cached reference floor
    reinitialize();

    // create a floor (linePoints RCS, robot at (0,0,0)) for input linepoints
    _linePoints = createLinePoints();

    // check for any linepoints
    // (having none at all is very unusual for a real robot, but not so much in test suite)
    if (_linePoints.size())
    {
        // manual mode?
        if (_params.solver().manual().enabled())
        {
            manualMode();
        }
        else
        {
            // regular mode, based on trackers
            // setup trackers: existing from state and new from guessing configuration
            _trackers = createTrackers();

            // run the fit algorithm (multithreaded), update trackers, update _fitResult
            runFitUpdateTrackers();
        }
    }

    // create and optionally dump of diagnostics data for plotting
    dumpDiagnosticsMat();

    // prepare for next tick
    _state.set_tick(1 + _state.tick());

    // set best fit result as output, or return error code
    return 0;
}

Output const &Solver::getOutput() const
{
    return _output;
}

Local const &Solver::getDiagnostics() const
{
    return _diag;
}

State const &Solver::getState() const
{
    return _state;
}

