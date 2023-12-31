#include "planner_common.hpp"

#include <vector>
#include <iostream>
#include <iomanip>
#include <cmath>
#include "MathUtils.h"

using namespace std;
using namespace MRA;

double chance_of_intercept(const MRA::Geometry::Point& pass_begin_vec, const MRA::Geometry::Point& pass_end_vec, const vector<MRA::Geometry::Pose>& Opponents,
		double interceptionChanceStartDistance,
		double interceptionChanceIncreasePerMeter,
		double interceptionDistancePenaltyFactor)
{
	// Calculates the intercept by enemy threat
	double interceptionPenalty = 0;

	// check chance of intercept from begin to end position of the pass.
	// Check if in a trapezoid (smallest parallel side at begin of pass and largest parallel side at end of the pass.
	// Trapezoid is constructed with X meter from begin and X meter + pass-distance * increase_per_meter
	// Check for each opponent if the opponent is in the trapezoid. (divide trapezoid area in two triangle and
	// check if opponent is not in one of the triangles.
	// if opponent is  the trapezoid, the penalty is related to to distance of the pass-line

	double start_width = interceptionChanceStartDistance; // starting with for interception
	double increase_per_meter = interceptionChanceIncreasePerMeter;
	double dist_from_to = pass_end_vec.distanceTo(pass_begin_vec); // distance from pass begin to pass end point


	double angle_begin_to_end_pass = pass_end_vec.angle(pass_begin_vec); // angle from begin pass to end pass point
	double end_width = start_width + dist_from_to * increase_per_meter; // calculate size of largest parallel side of trapezoid

	// calculate largest parallel side of trapezoid
	double EndXp = pass_end_vec.x + (cos(angle_begin_to_end_pass + 0.5 * M_PI) * end_width);
	double EndYp = pass_end_vec.y + (sin(angle_begin_to_end_pass + 0.5 * M_PI) * end_width);
	double EndXm = pass_end_vec.x  + (cos(angle_begin_to_end_pass - 0.5 * M_PI) * end_width);
	double EndYm = pass_end_vec.y  + (sin(angle_begin_to_end_pass - 0.5 * M_PI) * end_width);


	// calculate smallest parallel side of trapezoid
	double BeginXp = pass_begin_vec.x + (cos(angle_begin_to_end_pass + 0.5 * M_PI) * start_width);
	double BeginYp = pass_begin_vec.y + (sin(angle_begin_to_end_pass + 0.5 * M_PI) * start_width);
	double BeginXm = pass_begin_vec.x  + (cos(angle_begin_to_end_pass - 0.5 * M_PI) * start_width);
	double BeginYm = pass_begin_vec.y  + (sin(angle_begin_to_end_pass - 0.5 * M_PI) * start_width);

	// loop for calculating angles between receiving positions and opponents
	for( unsigned int i = 0; i < Opponents.size(); i++){
	    MRA::Geometry::Point opponent = Opponents[i];
		// return true if point (x,y) is in polygon defined by the points  else return false
		if (inTriangle(BeginXp, BeginYp, EndXm, EndYm, BeginXm, BeginYm, opponent.x , opponent.y) ||
				inTriangle(EndXm, EndYm, BeginXp, BeginYp, EndXp, EndYp, opponent.x , opponent.y) )
		{
			// Opponent is in the Trapezoid
			// calculate intercept point of the perpendicular from Opponent to the pass-line (shortest line to pass line for the opponent)
		    MRA::Geometry::Point Per;
			intersectPerpendicular(Per.x, Per.y, pass_begin_vec.x, pass_begin_vec.y, pass_end_vec.x, pass_end_vec.y, opponent.x, opponent.y);
			// calculate distance from opponent intercept point (Vector Per) to pass begin point
			double distPer = pass_begin_vec.distanceTo(Per); //
			double ext_at_Per = start_width + distPer * increase_per_meter;
			double distOppToPer = opponent.distanceTo(Per); // distance opponent to  pass-line

			// penalty per opponent:
			// relative to distance to pass-line (divide by max distance to side of trapezoid at that distance
			double enemyPenalty = interceptionDistancePenaltyFactor * ((ext_at_Per-distOppToPer)/ext_at_Per);

			// sum all penalties, a point with a lot of intercept possibilities gets a high penalty
			interceptionPenalty = max(enemyPenalty, interceptionPenalty);
		}
	}

	return interceptionPenalty;
}
