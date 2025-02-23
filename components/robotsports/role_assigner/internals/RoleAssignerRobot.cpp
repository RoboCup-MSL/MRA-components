/**
 *  @file
 *  @brief   Class for team planning
 *  @curator Jürge van Eijck
 */

#include "RoleAssignerRobot.hpp"

#include <iomanip>
#include <sstream>
#include <iostream>
#include <cmath>
#include "RoleAssignerData.hpp"

using namespace std;
using namespace MRA;


    // bool active; // participating in the game (robot may be inactive when figuring out where it is)
    // bool human;
    // long trackingId;

std::string RoleAssignerRobot::toString() const {
    std::stringstream buffer;
    buffer << std::boolalpha
           << "robotId: " << this->robotId   << " player-type: " << PlayerTypeAsString(this->player_type) 
           <<  " active: " << this->active << endl
           << "Pos: " << this->position.toString() 
           << " Vel: " << this->velocity.toString() << endl
           << " control ball: " <<  this->controlBall << " pass is on its way: " << passBall << endl
           << "human: " << this->human   << " trackingId: " << this->trackingId << endl
           << "time_in_own_penalty_area:" << this->time_in_own_penalty_area 
           << " time_in_opponent_penalty_area: " << this->time_in_opponent_penalty_area << endl;
    return buffer.str();
}
    

// compare function to sort vector of the class on the member robotId
bool RoleAssignerRobot::CompareRobotId(const RoleAssignerRobot& r1, const RoleAssignerRobot& r2)
{
    return (r1.robotId < r2.robotId);
}
