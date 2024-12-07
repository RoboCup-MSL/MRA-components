#include "logging.hpp"
#include "guessing.hpp"

using namespace MRA::FalconsLocalizationVision;


Guesser::Guesser(Params const &params)
{
    _config = params.solver().guessing();
    _floorMaxX = 0.5 * params.model().b();
    _floorMaxY = 0.5 * params.model().a();
}

std::optional<MRA::Geometry::Point> Guesser::tryGuess(std::vector<MRA::Geometry::Point> const &pointsToAvoid) const
{
    MRA_TRACE_FUNCTION();
    std::optional<MRA::Geometry::Point> result;
    for (int iAttempt = 0; iAttempt < _config.random().maxtries(); ++iAttempt)
    {
        MRA::Geometry::Point candidate(
            (2.0 * rand() / RAND_MAX - 1.0) * _floorMaxX,
            (2.0 * rand() / RAND_MAX - 1.0) * _floorMaxY);
        bool tooClose = false;
        for (auto const &pt: pointsToAvoid)
        {
            tooClose |= ((candidate - pt).size() < _config.random().exclusionradius());
        }
        if (!tooClose)
        {
            return candidate;
        }
    }

    return result;
}

void Guesser::run(std::vector<Tracker> &trackers, Params const &params, bool initial) const
{
#ifdef MRA_LOGGING_ENABLED
    int num_trackers = trackers.size();
    MRA_TRACE_FUNCTION_INPUTS(num_trackers, initial);
#endif

    // if so configured, add new fit attempts (as trackers)
    // so that the fit algorithm can run them all

    // structural guesses
    auto gvec = _config.structural();
    std::transform(gvec.begin(), gvec.end(), std::back_inserter(trackers),
        [params](const MRA::Datatypes::Circle &c) { return Tracker(params, c); });

    // initial guesses only at the very first tick
    if (initial)
    {
        gvec = _config.initial();
        std::transform(gvec.begin(), gvec.end(), std::back_inserter(trackers),
            [params](const MRA::Datatypes::Circle &c) { return Tracker(params, c); });
    }

    // for random guessing, use avoidance areas
    std::vector<MRA::Geometry::Point> pointsToAvoid;
    for (auto const &tr: trackers)
    {
        MRA::Geometry::Point pt(tr.fitResult.x, tr.fitResult.y);
        pointsToAvoid.push_back(pt);
    }

    // add random guesses
    auto rgParams = _config.random();
    bool doRandom = rgParams.count() && rgParams.searchradius() && rgParams.maxtries();
    for (int iRandom = 0; iRandom < doRandom * rgParams.count(); ++iRandom)
    {
        // draw a random pos within the field range
        std::optional<MRA::Datatypes::Point> pt = tryGuess(pointsToAvoid);
        if (pt)
        {
            // store
            MRA::Datatypes::Circle c;
            c.mutable_center()->set_x(pt->x());
            c.mutable_center()->set_y(pt->y());
            c.set_radius(rgParams.searchradius());
            trackers.push_back(Tracker(params, c));
            pointsToAvoid.push_back(*pt);
        }
    }
}

