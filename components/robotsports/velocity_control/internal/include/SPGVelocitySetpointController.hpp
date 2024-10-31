/*
 * SPGVelocitySetpointController.hpp
 * Started as copy of file with same name in the Falcons variant of Velocity Control
 */

#ifndef ROBOTSPORTS_VELOCITYCONTROL_SPGVELOCITYSETPOINTCONTROLLER_HPP_
#define ROBOTSPORTS_VELOCITYCONTROL_SPGVELOCITYSETPOINTCONTROLLER_HPP_

// own package
#include "VelocitySetpointControllers.hpp"

// MRA-libraries
#include "MRAbridge.hpp"

// external
//#include <ReflexxesAPI.h>
#include <ruckig/ruckig.hpp>

struct SpgLimits
{
    float vx;
    float vy;
    float vRz;
    float ax;
    float ay;
    float aRz;
    bool  hasJerkLimit;
    float jx;
    float jy;
    float jRz;
};

class SPGVelocitySetpointController : public AbstractVelocitySetpointController
{
public:
    SPGVelocitySetpointController();
    ~SPGVelocitySetpointController();
    bool calculate(VelocityControlData &data);

private:
    bool calculateSPG(VelocityControlData& data, SpgLimits const &spgLimits, Position2D& resultPosition, Velocity2D &resultVelocity);
    bool isDofAccelerating(const VelocityControlData &data, const Velocity2D& resultVelocity, int dof, float threshold);

    // Position SPG
    bool calculatePosXYRzPhaseSynchronized(VelocityControlData& data, SpgLimits const &spgLimits, Position2D& resultPosition, Velocity2D &resultVelocity);
    bool calculatePosXYPhaseSynchronized(VelocityControlData& data, SpgLimits const &spgLimits, Position2D& resultPosition, Velocity2D &resultVelocity);
    bool calculatePosRzNonSynchronized(VelocityControlData& data, SpgLimits const &spgLimits, Position2D& resultPosition, Velocity2D &resultVelocity);

    // Velocity SPG
    bool calculateVelXYRzPhaseSynchronized(VelocityControlData& data, SpgLimits const &spgLimits, Position2D& resultPosition, Velocity2D &resultVelocity);

    // internal data stored for open loop
    Position2D m_deltaPositionRCS;
    Velocity2D m_currentVelocityRCS;
    Velocity2D m_targetVelocityRCS;

};

#endif

