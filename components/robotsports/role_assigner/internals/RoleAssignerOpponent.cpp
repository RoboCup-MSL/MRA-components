#include "RoleAssignerOpponent.hpp"

#include <iomanip>
#include <sstream>
#include <iostream>
using namespace MRA;


std::string MRA::RoleAssignerOpponent::toString() const {
    std::stringstream buffer;
    buffer << "trackingId: " << this->trackingId  << " assigned: " << this->assigned << std::endl
           << "Pos: " << this->position.toString() << " Vel: " << this->velocity.toString() << std::endl;
    return buffer.str();
}

