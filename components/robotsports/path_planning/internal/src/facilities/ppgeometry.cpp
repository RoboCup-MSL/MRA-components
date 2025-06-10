/*
 * ppgeometry.cpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */

#include "ppgeometry.hpp"
#include <cmath>

MRA::Geometry::Position addRcsToFcs(MRA::Geometry::Position const &posRcs, MRA::Geometry::Position const &posFcs)
{
    // return posFcs with posRcs offset added
    // TODO (#14): awkward old MRA::Geometry::Position API ... we should actually improve the core MRA::Geometry::Position class ...
    MRA::Geometry::Position result = posRcs;
    result.transformRcsToFcs(posFcs);
    result.rz = posFcs.rz;
    return result;
}

MRA::Geometry::Position faceTowards(MRA::Geometry::Position const &current, double targetX, double targetY)
{
    MRA::Geometry::Position result = current;
    result.rz = std::atan2(targetY - current.y, targetX - current.x);
    return result;
}

