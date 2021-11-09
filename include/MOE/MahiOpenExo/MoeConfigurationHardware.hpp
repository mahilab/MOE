#pragma once

#include <Mahi/Daq/Quanser/Q8Usb.hpp>
#include <Mahi/Daq/Quanser/QuanserEncoder.hpp>
#include <Mahi/Daq/Types.hpp>
#include <Mahi/Daq/Watchdog.hpp>
#include <vector>

namespace moe {

    //==============================================================================
    // FORWARD DECLARATIONS
    //==============================================================================

    class MahiOpenExoHardware;

    //==============================================================================
    // CLASS DECLARATION
    //==============================================================================

    /// Encapsulates the hardware configuration for a MahiOpenExo
    class MoeConfigurationHardware {

    public:

        /// Constructor for standard configuration
        MoeConfigurationHardware(mahi::daq::Q8Usb&                     daq,
                                 const std::vector<mahi::daq::ChanNum> encoder_channels = {0,1,2,3},
                                 const std::vector<mahi::daq::ChanNum> enable_channels = {0,1,2,3},
                                 const std::vector<mahi::daq::ChanNum> current_write_channels = {0,1,2,3},
                                 const std::vector<mahi::daq::TTL>     enable_values = std::vector<mahi::daq::TTL>(4,mahi::daq::TTL_HIGH),
                                 const std::vector<double>             amp_gains = {0.4, 0.2, 0.2, 0.2}):
            m_daq(daq),
            m_encoder_channels(encoder_channels),
            m_enable_channels(enable_channels),
            m_current_write_channels(current_write_channels),
            m_enable_values(enable_values),
            m_amp_gains(amp_gains)
            {
            }
    
    private:

        friend class MahiOpenExoHardware;

        mahi::daq::Q8Usb&                     m_daq;                    // DAQ controlling the MahiOpenExo
        const std::vector<mahi::daq::ChanNum> m_encoder_channels;       // encoder channels that measure motor positions
        const std::vector<mahi::daq::ChanNum> m_enable_channels;        // DO channels that enable/disable motors
        const std::vector<mahi::daq::ChanNum> m_current_write_channels; // AI channels that write current to amps
        const std::vector<mahi::daq::TTL>     m_enable_values;          // enable values for the amplifiers to enable the motors
        const std::vector<double>              m_amp_gains;              // amplifier gain to convert volts to amps
    };
} // namespace moe