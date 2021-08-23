// Do we need to include license and authors up here?

// Based on MeiiParameters.hpp found in MEII
// taking most info from common.hpp found in src folder for now

#pragma once
#include <MAHI/Util/Math/Constants.hpp>
#include <Mahi/Util/Types.hpp>
#include <Mahi/Util/Timing/Time.hpp>
#include <array>

using mahi::util::PI;
using mahi::util::DEG2RAD;
using mahi::util::seconds;

namespace moe {

    /// Stores the constant parameters associated with the Mahi Open Exo
    struct MoeParameters{
        /// Default constructor.
        MoeParameters() :
                            // Joint 0      Joint 1       Joint 2      Joint 3      
        kt_{                    0.127,      0.0603,       0.0603,       0.0538},
        motor_cont_limits_{0, 0, 0, 0},
        motor_peak_limits_{0, 0, 0, 0},
        motor_i2t_times_{seconds(0.0), seconds(0.0), seconds(0.0), seconds(0.0)},
        eta_{                0.42/4.5, 0.4706/8.75,   0.4735/9.0,  0.2210/6.00},
        encoder_res_{           500*4,         500,          500,          500},
        pos_limits_min_{0, 0, 0, 0},
        pos_limits_max_{0, 0, 0, 0},
        vel_limits_{      180*DEG2RAD, 200*DEG2RAD,  200*DEG2RAD,  200*DEG2RAD},
        joint_torque_limits{      4.0,         4.0,          4.0,          4.0},
        kin_friction_{0, 0, 0, 0}
        { }

    /// motor torque constants [Nm/A]
    std::array<double, 4> kt_;
    /// motor continuous limits [A]
    std::array<double, 4> motor_cont_limits_;
    /// motor peak current limits [A]
    std::array<double, 4> motor_peak_limits_;
    /// motor i^2*t times [s]
    std::array<mahi::util::Time, 4> motor_i2t_times_;
    /// transmission ratios [inch/inch] or [m]
    std::array<double, 4> eta_;
    /// encoder resolutions [counts/rev]
    std::array<mahi::util::uint32, 4> encoder_res_;
    /// joint position limits minimum [rad]
    std::array<double, 4> pos_limits_min_;
    /// joint position limits maximum [rad]
    std::array<double, 4> pos_limits_max_;
    /// joint velocity limits [rad/s]
    std::array<double, 4> vel_limits_;
    /// joint torque limits [Nm]
    std::array<double, 4> joint_torque_limits;
    /// joint kinetic friction [Nm]
    std::array<double, 4> kin_friction_;
    };
}
