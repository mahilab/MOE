#pragma once
#include <array>

namespace moe{
    struct JointProperties
    {
        double m;
        double Pcx;
        double Pcy;
        double Pcz;
        double Icxx;
        double Icyy;
        double Iczz;
        double Icxy;
        double Icxz;
        double Icyz;
    };
    
}