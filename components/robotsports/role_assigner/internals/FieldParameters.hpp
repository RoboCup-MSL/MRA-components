/*
 * FieldParameters.hpp
 *
 *  Created on: Sep 26, 2024
 *      Author: jurge
 */

#ifndef COMPONENTS_ROBOTSPORTS_ROLE_ASSIGNER_INTERNALS_FIELDPARAMETERS_HPP_
#define COMPONENTS_ROBOTSPORTS_ROLE_ASSIGNER_INTERNALS_FIELDPARAMETERS_HPP_

class StandardLetterModel
{
public:
    double A = 22.0;  // [22.0]   field length including lines (y)
    double B = 14.0;  // [14.0]   field width including lines (x)
    double C = 6.9;   // [ 6.9]   penalty area width including lines (x)
    double D = 3.9;   // [ 3.9]   goal area width including lines (x)
    double E = 2.25;  // [ 2.25]  penalty area length including lines (y)
    double F = 0.75;  // [ 0.75]  goal area length including lines (y)
    double G = 0.75;  // [ 0.75]  corner circle radius including lines
    double H = 4.0;   // [ 4.0]   inner circle diameter including lines
    double I = 3.6;   // [ 3.6]   penalty mark distance (y) including line to mark center (?)
    double J = 0.15;  // [ 0.15]  penalty- and center mark diameter
    double K = 0.125; // [ 0.125] line width
    double L = 1.0;   // [ 1.0]   field border (x) (between outer line and black safety border)
    double M = 1.0;   // [ 1.0]   Technical Team Area width (x)
    double N = 7.5;   // [ 7.5]   Technical Team Area length (y) (between safety borders)
    double O = 1.0;   // [ 1.0]   Technical Team Area ramp length (y)
    double P = 0.5;   // [ 0.5]   Technical Team Area ramp width (x)
    double Q = 3.5;   // [ 3.5]   off-center distance to restart spots (x)
};

class FieldParameters {
public:
    StandardLetterModel SLM;
    bool penalty_area_present = true;
    bool technical_team_area_present = true;
    double goal_width = 2.0;
    double goal_length = 0.6;
};




#endif /* COMPONENTS_ROBOTSPORTS_ROLE_ASSIGNER_INTERNALS_FIELDPARAMETERS_HPP_ */
