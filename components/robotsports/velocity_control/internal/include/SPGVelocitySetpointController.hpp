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

struct SpgLimits {
    float vx;
    float vy;
    float vRz;
    float ax;
    float ay;
    float aRz;
    bool hasJerkLimit;
    float jx;
    float jy;
    float jRz;
    float vxy;
    float axy;
};

class SPGVelocitySetpointController : public AbstractVelocitySetpointController {
  public:
    SPGVelocitySetpointController();
    ~SPGVelocitySetpointController();
    bool calculate(VelocityControlData &data);

  private:
    enum AXES {
      AXES_XY,
      AXES_X,
      AXES_Y,
      AXES_RZ,
      AXES_XYRZ
    };

    bool calculateSPG(const VelocityControlData& r_data, const SpgLimits& r_spgLimits, VelocityControlResult& result);

    // Position SPG
    bool calculatePosXYRzPhaseSynchronized(const VelocityControlData&  r_data, const SpgLimits& r_spgLimits, VelocityControlResult& r_result);
    bool calculatePosXYPhaseSynchronized(const VelocityControlData&  r_data, const SpgLimits& r_spgLimits, VelocityControlResult& r_result);
    bool calculatePosRzNonSynchronized(const VelocityControlData& r_data, const SpgLimits& r_spgLimits, VelocityControlResult& r_result);

    // Velocity SPG
    bool calculateVelXYRzPhaseSynchronized(const VelocityControlData&  r_data, const SpgLimits& r_spgLimits, VelocityControlResult& r_result);
    bool calculateVelXYPhaseSynchronized(const VelocityControlData&  r_data, const SpgLimits& r_spgLimits, VelocityControlResult& r_result);
    bool calculateVelSingleAxisNonSynchronized(enum SPGVelocitySetpointController::AXES axis, const VelocityControlData&  r_data,
    						 const SpgLimits& r_spgLimits, VelocityControlResult& r_result);

    // helper function for calculation using ruckig library
    template<size_t DOFs, template<class, size_t> class CustomVector>
    bool ruckig_calculate(const MRA::internal::RVC::VelocityControlData & r_data,
                          ruckig::InputParameter<DOFs, CustomVector>& r_input,
                          AXES axes,
                          MRA::internal::RVC::VelocityControlResult & r_result);


    // internal data stored for open loop
    Position2D m_deltaPositionRCS;
    Velocity2D m_currentVelocityRCS;
    Velocity2D m_targetVelocityRCS;
};

#endif
