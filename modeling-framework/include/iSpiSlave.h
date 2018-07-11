#pragma once
#include <cstdint>
#include "SpiCommon.h"

struct SpiSlaveConfig {
    int mosi_pin_number;
    int miso_pin_number;
    int ss_pin_number;
    int sclk_pin_number;
    int supported_spi_modes;
    unsigned int max_frequency;
    BitOrder bit_order;
};

class iSpiSlaveV1 {
  public:
    virtual size_t Transmit(uint8_t *rx_buffer, uint8_t *tx_buffer, size_t length) = 0;
    virtual bool IsSsActive() = 0;
    virtual void Detach() = 0;
};

class iSpiSlaveV2: public iSpiSlaveV1 {
  public:
     virtual bool WaitForMasterTransmit() = 0;
};

typedef iSpiSlaveV2 iSpiSlave;
