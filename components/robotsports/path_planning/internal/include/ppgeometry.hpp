/*
 * ppgeometry.hpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */

#ifndef PPGEOMETRY_HPP_
#define PPGEOMETRY_HPP_

#include "PathPlanning.hpp"

MRA::Geometry::Position addRcsToFcs(MRA::Geometry::Position const &posRcs, MRA::Geometry::Position const &posFcs);
MRA::Geometry::Position faceTowards(MRA::Geometry::Position const &current, double targetX, double targetY);

#endif

