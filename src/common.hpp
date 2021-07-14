#pragma once

#include <Mahi/Util.hpp>

using mahi::util::PI;
using mahi::util::DEG2RAD;
using std::vector;
                                  //  Joint 0      Joint 1      Joint 2      Joint 3
vector<double>     gear_ratios = {   0.42/4.5, 0.4706/8.75,  0.4735/9.0, 0.2210/6.00}; // ratio from motor shaft rotation to joint rotation [unitless]
vector<double>    encoder_cprs = {      500*4,         500,         500,         500}; // counts per revolution of encoders on motors
vector<bool>       switch_dirs = {      false,       false,       false,       true}; // Decides whether or not to switch directions (applied to encoder)
vector<double>             Kts = {      0.127,      0.0603,      0.0603,      0.0538}; // Motor Kt vals [Nm/A]
vector<double>      amp_ratios = {        0.4,         0.2,         0.2,         0.2}; // amp per volt ratio set in the driver software [A/V]
vector<double> read_amp_ratios = {        1.0,         0.5,         0.5,         0.5}; // amp per volt ratio set in the driver software [A/V]
vector<double>             Kps = {      125.0,        25.0,        20.0,        20.0}; // Kp for pd control of joints
vector<double>             Kds = {       1.75,        1.15,         1.0,        0.25}; // Kd for pd control of joints
vector<double>   torque_limits = {        4.0,         4.0,         4.0,         4.0}; // Software Torque limits on each joint (Nm)
vector<double> velocity_limits = {180*DEG2RAD, 200*DEG2RAD, 200*DEG2RAD, 200*DEG2RAD}; // Velocity Limits on each motor (rad/s)
vector<size_t> encoder_channel = {          0,           1,           2,           3}; // Velocity Limits on each motor (rad/s)

int moe_n_dof = 4;

enum moe_joint_enum{
    elbow_fe,   // 0
    forearm_ps, // 1
    wrist_fe,   // 2
    wrist_ru    // 3
};

bool torque_limits_exceeded(vector<double> current_torques, vector<double> torque_limits){
    for (auto i = 0; i < torque_limits.size(); i++){
        if (abs(current_torques[i]) >= torque_limits[i]){
            LOG(mahi::util::Error) << "Torque limit on joint " << i << " of " << torque_limits[i] << " Nm exceeded with a value of " << current_torques[i] << ".";
            return true;
        }
    }
    return false;
}

bool velocity_limits_exceeded(vector<double> current_velocities, vector<double> velocity_limits){
    for (auto i = 0; i < velocity_limits.size(); i++){
        if (abs(current_velocities[i]) >= velocity_limits[i]){
            LOG(mahi::util::Error) << "Velocity limit on joint " << i << " of " << velocity_limits[i] << " rad/s exceeded with a value of " << current_velocities[i] << ".";
            return true;
        }
    }
    return false;
}