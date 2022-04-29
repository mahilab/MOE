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
                      J3({     0.221559,   -0.0573976,    0.0225462,    0.0493419,  0.000497518,  0.000487249,  0.000290413, -3.28529e-06,  0.000130188,  6.33151e-05}),
                 J0_main({      0.88611,   -0.0242374,   0.00733288,  -0.00109224,    0.0077745,   0.00698468,    0.0030145,  0.000471255,  0.000390345,    0.0002139}),
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
