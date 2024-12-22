#include "EnvironmentParameters.hpp"

#include <sstream>

std::string MRA::EnvironmentParameters::toString() const {
    std::stringstream buffer;
    buffer << std::boolalpha;
    buffer << "StandardLetterModel: " << std::endl;
    buffer << "\tA = " << this->SLM.A << std::endl;
    buffer << "\tB = " << this->SLM.B << std::endl;
    buffer << "\tC = " << this->SLM.C << std::endl;
    buffer << "\tD = " << this->SLM.D << std::endl;
    buffer << "\tE = " << this->SLM.E << std::endl;
    buffer << "\tF = " << this->SLM.F << std::endl;
    buffer << "\tG = " << this->SLM.G << std::endl;
    buffer << "\tH = " << this->SLM.H << std::endl;
    buffer << "\tI = " << this->SLM.I << std::endl;
    buffer << "\tJ = " << this->SLM.J << std::endl;
    buffer << "\tK = " << this->SLM.K << std::endl;
    buffer << "\tL = " << this->SLM.L << std::endl;
    buffer << "\tM = " << this->SLM.M << std::endl;
    buffer << "\tN = " << this->SLM.N << std::endl;
    buffer << "\tO = " << this->SLM.O << std::endl;
    buffer << "\tP = " << this->SLM.P << std::endl;
    buffer << "\tQ = " << this->SLM.Q << std::endl;
    buffer << "penalty_area_present = " << this->penalty_area_present << std::endl;
    buffer << "technical_team_area_present = " << this->technical_team_area_present << std::endl;
    buffer << "goal_width = " << this->goal_width << std::endl;
    buffer << "goal_length = " << this->goal_length << std::endl;
    buffer << "parking_area_width = " << this->parking_area_width << std::endl;
    buffer << "parking_area_length = " << this->parking_area_length << std::endl;
    buffer << "parking_distance_between_robots = " << this->parking_distance_between_robots << std::endl;
    buffer << "parking_distance_to_line = " << this->parking_distance_to_line << std::endl;
    buffer << "robot_size = " << this->robot_size << std::endl;;
    buffer << "ball_radius = " << this->ball_radius << std::endl;
    buffer << "minimum_distance_to_goal_area = " << this->minimum_distance_to_goal_area << std::endl;
    return buffer.str();
}
