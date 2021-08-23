#pragma once

#include <MOE/MahiOpenExo/MoeConfigurationVirtual.hpp>
#include <MOE/MahiOpenExo/Moe.hpp>
#include <Mahi/Util/Print.hpp>

namespace moe {
    /// Class for controlling the Mahi Open Exo
    class MahiOpenExoVirtual : public Moe {

    ///////////////////////// STANDARD CLASS FUNCTIONS AND PARAMS /////////////////////////

    public: 
        /// Constructor
        MahiOpenExoVirtual(MoeConfigurationVirtual configuration);

        MoeConfigurationVirtual config_vr; // moe configuration, conisting of daq, parameters, etc

    //////////////// OVERRIDING PURE VIRTUAL FUNCTIONS OF MOE ////////////////
    public:
        /// enables the daq
        bool daq_enable(){return true;};
        /// disables the daq
        bool daq_disable(){return true;};
        /// opens the daq
        bool daq_open(){return true;};
        /// close the daq
        bool daq_close(){return true;};
        /// starts the watchdog on the daq
        bool daq_watchdog_start(){return true;};
        /// kicks the watchdog on the daq
        bool daq_watchdog_kick(){return true;};
        /// reads all from the daq
        bool daq_read_all(){return true;};
        /// writes all from the daq
        bool daq_write_all(){return true;};
        /// sets encoders to input position (in counts)
        bool daq_encoder_write(int index, mahi::util::int32 encoder_offset){return true;};
    };
} // namespace moe