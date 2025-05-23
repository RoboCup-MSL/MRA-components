/*
 * VelocityControl.hpp
 * Started as copy of file with same name in the Falcons variant of Velocity Control
 */

#ifndef VELOCITYCONTROL_HPP_
#define VELOCITYCONTROL_HPP_


#include "VelocityControlData.hpp"
#include "VelocityControlAlgorithms.hpp"


namespace MRA::internal::RVC
{

class VelocityControl
{
public:
    VelocityControl();
    ~VelocityControl();

    void iterate();

public:
    // having these public is convenient for test suite
    VelocityControlData data{};

private:
    std::vector<std::shared_ptr<VelocityControlAlgorithm>> algorithms;
    void setup();
    void add_algorithm(std::shared_ptr<VelocityControlAlgorithm> alg, bool unskippable = false);
};

} // namespace MRA::internal::RVC

#endif

