#pragma once

#include <Mahi/Util/Math/Constants.hpp>
#include <string>
#include <vector>

namespace moe {

    //==============================================================================
    // FORWARD DECLARATIONS
    //==============================================================================

    class MahiOpenExoVirtual;

    //==============================================================================
    // CLASS DECLARATION
    //==============================================================================

    /// Encapsulates the hardware configuration for a MahiOpenExo
    class MoeConfigurationVirtual {
    
    public:

        /// Constructor for standard configuration
        MoeConfigurationVirtual(const std::vector<double> rest_positions = {-45*mahi::util::DEG2RAD, 0, 0, 0},
                                const std::vector<std::string> torque_ms_names = {"ms_torque_0",
                                                                                  "ms_torque_1",
                                                                                  "ms_torque_2",
                                                                                  "ms_torque_3"},
                                const std::vector<std::string> posvel_ms_names = {"ms_posvel_0",
                                                                                  "ms_posvel_1",
                                                                                  "ms_posvel_2",
                                                                                  "ms_posvel_3"}):
            m_rest_positions(rest_positions),
            m_torque_ms_names(torque_ms_names),
            m_posvel_ms_names(posvel_ms_names)
            {
            }

    private:

        friend class MahiOpenExoVirtual;

        const std::vector<double> m_rest_positions; // rest positions to use when there is no input from melshare
        const std::vector<std::string> m_torque_ms_names; // names for the torque melshares
        const std::vector<std::string> m_posvel_ms_names; // 
    };
} // namespace moe