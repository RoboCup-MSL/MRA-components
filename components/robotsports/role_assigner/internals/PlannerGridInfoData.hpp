/*
 * TeamPlannerGrid.h
 *
 *  Created on: Jan 3, 2016
 *      Author: jurge
 */

#ifndef PLANNERGRIDINFODATA_H_
#define PLANNERGRIDINFODATA_H_

#include <vector>
#include <string>
#include "geometry.hpp"

namespace MRA {

class PlannerGridGameData {
public:
	std::vector<MRA::Geometry::Pose> Team;  // me as first
	std::vector<MRA::Geometry::Pose> Opponents;
	MRA::Geometry::Pose ball;
};


class PlannerGridCell{
public:
	PlannerGridCell(unsigned id1, double x1,double y1, std::vector<double> v): id(id1), x(x1), y(y1), value(v) {
		//
	};
	unsigned id;
	double x;
	double y;
	std::vector<double> value;
};

class PlannerGridInfoData {
public:
	PlannerGridGameData gameData;
	std::vector<PlannerGridCell> cells;
	std::vector<std::string> name;
	std::vector<double> weight;

	std::string toString();
	void saveToFile(const std::string& filename);
	void readFromFile(const std::string& filename);
};

} /* namespace MRA */

#endif /* PLANNERGRIDINFODATA_H_ */