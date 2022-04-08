#pragma once
#include<MOE/MahiOpenExo/mass_properties.hpp>
#include<MOE/MahiOpenExo/MassProperties.hpp>
#include <array>
namespace moe {
    struct MoeMassProperties
    {
        MoeMassProperties():
            J0({ m0, Pcx0, Pcy0, Pcz0, Icxx0, Icyy0, Iczz0, Icxy0, Icxz0, Icyz0}),
            J1({ m1, Pcx1, Pcy1, Pcz1, Icxx1, Icyy1, Iczz1, Icxy1, Icxz1, Icyz1}),
            J2({ m2, Pcx2, Pcy2, Pcz2, Icxx2, Icyy2, Iczz2, Icxy2, Icxz2, Icyz2}),
            J3({ m3, Pcx3, Pcy3, Pcz3, Icxx3, Icyy3, Iczz3, Icxy3, Icxz3, Icyz3}),
            all_props({J0, J1, J2, J3})
            {

            }
        MassProperties J0;
        MassProperties J1;
        MassProperties J2;
        MassProperties J3;
        std::vector<MassProperties> all_props;
    };
    

}