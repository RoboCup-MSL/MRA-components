#include "solver.hpp"
#include "guessing.hpp"

// MRA libraries
#include "geometry.hpp"
#include "opencv_utils.hpp"

using namespace MRA::FalconsLocalizationVision;


Solver::Solver()
{
}

Solver::~Solver()
{
}

void Solver::configure(Params const &p)
{
    _params = p;
    // check for missing required parameters
    checkParamsValid();
    // configure helper classes
    _floor.configure(_params);
    _fitAlgorithm.configure(_params.solver());
}

void Solver::checkParamsValid() const
{
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
    _state = s;
}

void Solver::setInput(Input const &in)
{
    _input = in;
}

cv::Mat Solver::createReferenceFloorMat(float blurFactor) const
{
    cv::Mat result;

    // check if (re)calculation is needed
    if (_state.has_referencefloor())
    {
        MRA::OpenCVUtils::deserializeCvMat(_state.referencefloor(), result); // if not fast enough, then cache internally
        return result;
    }

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
    _floor.shapesToCvMat(shapes, blurFactor, result);

    return result;
}

void Solver::updateReferenceFloorMat()
{
    if (!_state.has_referencefloor())
    {
        // store result as protobuf CvMatProto object for next iteration (via state)
        MRA::OpenCVUtils::serializeCvMat(_referenceFloorMat, *_state.mutable_referencefloor());
    }
}

cv::Mat Solver::createLinePointsMat(float overruleRadius) const
{
    // create a floor (linePoints RCS, robot at (0,0,0)) for input linepoints
    cv::Mat result = _floor.createMat();
    std::vector<Landmark> linePoints(_input.landmarks().begin(), _input.landmarks().end());
    _floor.linePointsToCvMat(linePoints, result, overruleRadius);
    return result;
}

std::vector<Tracker> Solver::createTrackers() const
{
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
    // run the fit algorithm (multithreaded, one per tracker) and update trackers
    _fitAlgorithm.run(_referenceFloorMat, _linePointsMat, _trackers);

    // set _fitResult
    _fitResult.valid = false;
    if (_trackers.size())
    {
        auto tr = _trackers.at(0);
        _fitResult.valid = tr.fitValid;
        _fitResult.pose = tr.fitResult;
        _fitResult.score = tr.fitScore;
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
}

// TODO move to opencv_utils?
// combine non-black pixels from m_other into m
void combineImages(cv::Mat m, cv::Mat m_other)
{
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
    // reference floor either with or without blur
    bool withBlur = true;
    cv::Mat referenceFloorMat = withBlur ? _referenceFloorMat : createReferenceFloorMat();

    // create diagnostics floor, upscale to colors
    cv::Mat result;
    cv::cvtColor(referenceFloorMat, result, cv::COLOR_GRAY2BGR);

    // add linepoints with blue/cyan color
    // _linePointsMat has pixels for the fit, let's construct a new one for visualization using overruleRadius
    cv::Mat linePointsMat = createLinePointsMat(_params.solver().linepoints().plot().radius());
    float ppm = _params.solver().pixelspermeter();
    FitFunction ff(referenceFloorMat, linePointsMat, ppm);
    cv::Mat transformedLinePoints = ff.transform3dof(linePointsMat, _fitResult.pose.x, _fitResult.pose.y, _fitResult.pose.rz);
    cv::Mat transformedLinePointsColor;
    cv::cvtColor(transformedLinePoints, transformedLinePointsColor, cv::COLOR_GRAY2BGR);
    cv::Mat whiteMask;
    cv::compare(transformedLinePointsColor, cv::Scalar(255, 255, 255), whiteMask, cv::CMP_EQ);
    transformedLinePointsColor.setTo(cv::Scalar(255, 0, 0), whiteMask);
    combineImages(result, transformedLinePointsColor);

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
    MRA::OpenCVUtils::serializeCvMat(createDiagnosticsMat(), *_diag.mutable_floor());
}

int Solver::run()
{
    // try to keep the design as simple as possible: minimize state, trackers over time (that is for worldModel to handle)
    // initially (and maybe also occasionally?) we should perhaps do some kind of grid search
    // that is handled in the FitAlgorithm, also optional multithreading and guessing / search space partitioning

    // the FitCore is a single fit operation (which uses opencv Downhill Simplex solver):
    // fit given white pixels and initial guess to the reference field

    // create or get the cached reference floor
    _referenceFloorMat = createReferenceFloorMat(_params.solver().blurfactor());
    updateReferenceFloorMat();

    // create a floor (linePoints RCS, robot at (0,0,0)) for input linepoints
    // TODO: factor out LinePoints object with operations (the mat transform implementation seems too slow)
    _linePointsMat = createLinePointsMat();

    // setup trackers: existing from state and new from guessing configuration
    _trackers = createTrackers();

    // run the fit algorithm (multithreaded), update trackers, update _fitResult
    runFitUpdateTrackers();

    // optional dump of diagnostics data for plotting
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
