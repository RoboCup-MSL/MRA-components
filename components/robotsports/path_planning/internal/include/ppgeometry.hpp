/*
 * ppgeometry.hpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */

#ifndef PPGEOMETRY_HPP_
#define PPGEOMETRY_HPP_

#include "PathPlanning.hpp"

Position2D addRcsToFcs(Position2D const &posRcs, Position2D const &posFcs);
Position2D faceTowards(Position2D const &current, float targetX, float targetY);

#endif

