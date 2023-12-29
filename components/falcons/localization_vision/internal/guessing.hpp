#ifndef _MRA_FALCONS_LOCALIZATION_VISION_GUESSING_HPP
#define _MRA_FALCONS_LOCALIZATION_VISION_GUESSING_HPP

#include <optional>
#include "tracker.hpp"
#include "FalconsLocalizationVision_datatypes.hpp"


namespace MRA::FalconsLocalizationVision
{

class Guesser
{
public:
    Guesser(Params const &params);
    ~Guesser() {};

    void run(std::vector<Tracker> &trackers, Params const &params, bool initial = true) const;

private:
    GuessingParams _config;
    float _floorMaxX;
    float _floorMaxY;

    std::optional<MRA::Geometry::Point> tryGuess(std::vector<MRA::Geometry::Point> const &pointsToAvoid) const;

}; // class Guesser

} // namespace MRA::FalconsLocalizationVision

#endif

