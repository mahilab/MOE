#pragma once
#include<MOE/MahiOpenExo/Dynamics/JointProperties.hpp>
#include <array>
namespace moe {
    struct MoeMassProperties
    {
        MoeMassProperties():
        //                            m,          Pcx,          Pcy,          Pcz,         Icxx,         Icyy,         Iczz,         Icxy,         Icxz,         Icyz
                      J0({          0.0,          0.0,          0.0,          0.0,          0.0,          0.0,          0.0,          0.0,          0.0,          0.0}),
                      J1({      1.52216,    0.0248667,    0.0334816,     0.113233,   0.00517995,    0.0075572,   0.00905835,   -0.0011334, -0.000548857, -0.000269683}),
                      J2({     0.763788,    0.0111073,    0.0286123,   -0.0793294,    0.0030323,   0.00130668,   0.00378613, -0.000482666, -5.13513e-05,  0.000213485}),
                      J3({     0.197497,   -0.0568326,    0.0232957,    0.0550034,  0.000418214,  0.000424516,  0.000268017, -7.03986e-06,  0.000124091,  5.16671e-05}),
                 J0_main({     0.964912,   -0.0243612,   0.00165483, -0.000772708,   0.00822809,   0.00703745,    0.0034252,  0.000479983,  0.000389853,  0.000194094}),
        J0_counterweight({      5.35239, -2.84006e-08,       0.0762,     0.188425,   0.00693784,   0.00693784,    0.0130096, -1.72914e-11,  2.63428e-09,  1.43658e-09}),
               J0_slider({     0.955742,     -0.10568,   -0.0917346,  -5.1449e-05,   0.00102281,  0.000740784,   0.00101596, -7.92816e-05, -1.61391e-06,  1.45908e-07})
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
