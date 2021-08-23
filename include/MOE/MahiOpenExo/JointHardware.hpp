#pragma once

#include <MOE/MahiOpenExo/Joint.hpp>
#include <Mahi/Daq/Handle.hpp>

namespace moe {
//==============================================================================
// CLASS DECLARATION
//==============================================================================

class JointHardware : public Joint {
public:
    /// Constructor
    JointHardware(const std::string &name,
                  std::array<double, 2> position_limits,
                  double velocity_limit,
                  double torque_limit,
                  mahi::robo::Limiter limiter,
                  double actuator_transmission,
                  std::shared_ptr<mahi::daq::EncoderHandle> position_sensor,
                  double position_transmission,
                  const double &velocity_sensor,
                  double velocity_transmission,
                  double motor_kt,
                  double amp_gain,
                  mahi::daq::DOHandle motor_enable_handle,
                  mahi::daq::TTL motor_enable_value,
                  mahi::daq::AOHandle amp_write_handle);
    
    /// Converts PositionSensor position to Joint Position
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
    std::shared_ptr<mahi::daq::EncoderHandle> m_position_sensor; // pointer to the PositionSensor of this Joint
    const double &m_velocity_sensor; // Pointer to the VelocitySensor of this Joint

    double m_actuator_transmission; // transmission ratio describing the 
                                    // multiplicative gain in torque from Joint
                                    // space to Actuator space, actuator Torque = 
                                    // actuator transmission * joint torque
    double m_position_transmission; // gain in position from PositionSensor space to Joint space, joint position = position sensor transmission * sensed position
    double m_velocity_transmission; // gain in velocity from VelocitySensor space to Joint space, joint velocity = velocity sensor transmission * sensed velocity

    double m_motor_kt; // motor gain in T/A
    double m_amp_gain; // amplifier gain in A/V

    mahi::daq::DOHandle m_motor_enable_handle; // DO channel used to control if motor is enabled/disabled
    mahi::daq::TTL m_motor_enable_value; // Digital value to set for to enable motor through the amplifier
    mahi::daq::AOHandle m_amp_write_handle; // AO channel to write to for setting desired torque values
};

} // namespace moe