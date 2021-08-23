#pragma once

#include <MOE/MahiOpenExo/Joint.hpp>
#include <Mahi/Com/MelShare.hpp>

namespace moe {

//==============================================================================
// CLASS DECLARATION
//==============================================================================

class JointVirtual : public Joint {
public:
    /// Constructor
    JointVirtual(const std::string &name,
                 std::array<double, 2> position_limits,
                 double velocity_limit,
                 double torque_limit,
                 mahi::robo::Limiter limiter,
                 std::shared_ptr<mahi::com::MelShare> ms_trq,
                 std::shared_ptr<mahi::com::MelShare> ms_pos,
                 const double rest_pos);
    
    /// Converts PositionSensor position to Joint position
    double get_position();

    /// Converts PositionSensor velocity to Joint velocity
    double get_velocity();

    /// Sets the joint torque to #new_torque
    void set_torque(double new_torque);

    /// Enables the joint's position sensor, velocity sensor, and actuator
    bool enable();

    /// Disables the joint's position sensor, velocity sensor, and actuator
    bool disable();

private:
    const double m_rest_pos; // value to send if there is no melshare available

    std::shared_ptr<mahi::com::MelShare> ms_torque; // melshare to send torque to simulation
    std::shared_ptr<mahi::com::MelShare> ms_posvel; // melshare to received position and velocity from simulation
};

} // namespace moe