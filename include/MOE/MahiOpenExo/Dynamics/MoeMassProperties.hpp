#pragma once
#include<include/MOE/MahiOpenExo/Dynamics/JointProperties.hpp>
#include<include/MOE/MahiOpenExo/Dynamics/UserParams.hpp>
#include <array>

namespace moe {
    struct MoeMassProperties
    {
        MoeMassProperties():
            J0({ m0, Pcx0, Pcy0, Pcz0, Icxx0, Icyy0, Iczz0, Icxy0, Icxz0, Icyz0}),
            J1({ m1, Pcx1, Pcy1, Pcz1, Icxx1, Icyy1, Iczz1, Icxy1, Icxz1, Icyz1}),
            J2({ m2, Pcx2, Pcy2, Pcz2, Icxx2, Icyy2, Iczz2, Icxy2, Icxz2, Icyz2}),
            J3({ m3, Pcx3, Pcy3, Pcz3, Icxx3, Icyy3, Iczz3, Icxy3, Icxz3, Icyz3}),
            J0_main({m0_main, Pcx0_main, Pcy0_main, Pcz0_main, Icxx0_main, Icyy0_main, Iczz0_main, Icxy0_main, Icxz0_main, Icyz0_main}),
            J0_counterweight({m0_counterweight, Pcx0_counterweight, Pcy0_counterweight, Pcz0_counterweight, Icxx0_counterweight, Icyy0_counterweight, Iczz0_counterweight, Icxy0_counterweight, Icxz0_counterweight, Icyz0_counterweight}),
            J0_slider({m0_slider, Pcx0_slider, Pcy0_slider, Pcz0_slider, Icxx0_slider, Icyy0_slider, Iczz0_slider, Icxy0_slider, Icxz0_slider, Icyz0_slider})
            {

            }
        JointProperties J0;
        JointProperties J1;
        JointProperties J2;
        JointProperties J3;
        JointProperties J0_main;
        JointProperties J0_counterweight;
        JointProperties J0_slider;


    };
}