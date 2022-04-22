#pragma once
#include<include/MOE/MahiOpenExo/Dynamics/JointProperties.hpp>
#include <array>
namespace moe {
    struct MoeMassProperties
    {
        MoeMassProperties():
        //                            m,          Pcx,          Pcy,          Pcz,         Icxx,         Icyy,         Iczz,         Icxy,         Icxz,         Icyz
                      J0({          0.0,          0.0,          0.0,          0.0,          0.0,          0.0,          0.0,          0.0,          0.0,          0.0}),
                      J1({      1.52216,    0.0248667,    0.0334816,     0.113233,    0.0264029,     0.028015,     0.011706,  0.000133917,   0.00373713,   0.00550114}),
                      J2({     0.763788,    0.0111073,    0.0286123,   -0.0793294,   0.00846422,   0.00620754,   0.00450565,  -0.00023993,  -0.00072435,  -0.00152016}),
                      J3({     0.221559,   -0.0573976,    0.0225462,    0.0493419,   0.00114956,   0.00175658,   0.00113296, -0.000290004,  -0.00049729,  0.000309793}),
                 J0_main({      0.88611,   -0.0242374,   0.00733288,  -0.00109224,   0.00782321,   0.00750629,    0.0035827,  0.000313766,  0.000413803,  0.000206803}),
        J0_counterweight({      5.35239,  -0.00845038,     0.080158,     0.170913,     0.197678,      0.16367,    0.0477826,  -0.00362553,  -0.00773035,     0.073328}),
               J0_slider({     0.955742,     -0.10568,   -0.0917346,  -5.1449e-05,   0.00906562,    0.0114148,    0.0197328,    0.0091862,   3.5826e-06,  4.65669e-06})
            {}
        JointProperties J0;
        JointProperties J1;
        JointProperties J2;
        JointProperties J3;
        JointProperties J0_main;
        JointProperties J0_counterweight;
        JointProperties J0_slider;
        const double g = 9.807;
    };
}
