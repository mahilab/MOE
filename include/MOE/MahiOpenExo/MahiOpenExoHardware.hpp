#pragma once

#include <MOE/MahiOpenExo/MoeConfigurationHardware.hpp>
#include <MOE/MahiOpenExo/Moe.hpp>
#include <Mahi/Daq/Handle.hpp>

namespace moe{
    /// Class for controlling the Mahi Open Exo
    class MahiOpenExoHardware : public Moe {

    ///////////////////////// STANDARD CLASS FUNCTIONS AND PARAMS /////////////////////////

    public: 
        /// Constructor
        MahiOpenExoHardware(MoeConfigurationHardware configuration);

        MoeConfigurationHardware config_hw; // moe configuration, consisting of daq, parameters, etc

        std::vector<mahi::daq::EncoderHandle*> encoder_handles;

    //////////////// OVERRIDING PURE VIRTUAL FUNCTIONS OF MEII ////////////////

    public:
        /// enables the daq
        bool daq_enable(){return config_hw.m_daq.enable();};
        /// disables the daq
        bool daq_disable(){return config_hw.m_daq.disable();};
        /// opens the daq
        bool daq_open(){return config_hw.m_daq.open();};
        /// closes the daq
        bool daq_close(){return config_hw.m_daq.close();};
        /// starts the watchdog on the daq
        bool daq_watchdog_start(){return config_hw.m_daq.watchdog.start();};
        /// kicks the watchdog on the daq
        bool daq_watchdog_kick(){return config_hw.m_daq.watchdog.kick();};
        /// reads all from the daq
        bool daq_read_all(){return config_hw.m_daq.read_all();};
        /// writes all from the daq
        bool daq_write_all(){return config_hw.m_daq.write_all();};
        /// sets encoders to input position (in counts)
        bool daq_encoder_write(int index, mahi::util::int32 encoder_offset){return config_hw.m_daq.encoder.write(index, encoder_offset);};
    };
} // namespace moe