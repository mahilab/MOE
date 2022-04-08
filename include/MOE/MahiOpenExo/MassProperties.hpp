#pragma once
#include <array>

namespace moe{
    struct MassProperties
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