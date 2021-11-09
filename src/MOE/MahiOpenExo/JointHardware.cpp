#include <MOE/MahiOpenExo/JointHardware.hpp>
#include <Mahi/Util/Math/Functions.hpp>
#include <Mahi/Util/Logging/Log.hpp>
#include <Mahi/Com/MelShare.hpp>
#include <iostream>

using namespace mahi::util;

namespace moe {

JointHardware::JointHardware(const std::string &name,
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
             mahi::daq::AOHandle amp_write_handle):
    Joint(name,position_limits,velocity_limit,torque_limit,limiter),
    m_position_sensor(position_sensor),
    m_velocity_sensor(velocity_sensor),
    m_actuator_transmission(actuator_transmission),
    m_position_transmission(position_transmission),
    m_velocity_transmission(velocity_transmission),
    m_motor_kt(motor_kt),
    m_amp_gain(amp_gain),
    m_motor_enable_handle(motor_enable_handle),
    m_motor_enable_value(motor_enable_value),
    m_amp_write_handle(amp_write_handle),
    butt(2,hertz(20),hertz(1000))
    {

    }

bool JointHardware::enable() {
    m_limiter.reset();
    if (!m_motor_enable_handle.write_level(m_motor_enable_value)) return false;
    set_torque(0.0);
    m_enabled = true;
    return true;
}

bool JointHardware::disable() {
    set_torque(0.0);
    if (!m_motor_enable_handle.write_level(!m_motor_enable_value)) return false;
    m_enabled = false;
    return true;
}

double JointHardware::get_position() {
    return m_position_transmission*m_position_sensor->get_pos();
}

double JointHardware::get_velocity() {
    if (software_velocity){
        static double pos_last = get_position();
        auto curr_pos = get_position();
        double velocity_unfiltered = (curr_pos - pos_last)/0.001;
        pos_last = curr_pos;
        return butt.update(velocity_unfiltered);
    }
    return m_velocity_transmission*m_velocity_sensor;
}

void JointHardware::set_torque(double new_torque) {
    if (m_enabled){
        m_com_torque = new_torque;
        if (torque_limit_exceeded()) {
            LOG(Warning) << "Joint " << get_name() << " command torque saturated to " << m_torque_limit;
            m_torque = clamp(m_com_torque, m_torque_limit);
        }
        else{
            m_torque = m_com_torque;
        }
        double motor_torque = m_torque * m_actuator_transmission;             // (Nm) * (Nm / Nm) = Nm
        double command_current = motor_torque/m_motor_kt;                     // (Nm) / (Nm / A)  = A
        double command_voltage = command_current/m_amp_gain; // (A)  / (A / V)   = V
        m_amp_write_handle.set_volts(command_voltage);
        // std::cout << "m_torque: " << m_torque << ", ";
        // std::cout << "command_current: " << command_current << ", ";
        // std::cout << "command_voltage: " << command_voltage << std::endl;
    }
}
                
} // namespace moe