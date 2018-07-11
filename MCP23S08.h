#pragma once
#include "ModelingFramework.h"
#include <map>

/* Registers */
#define IODIR_ADDR                  (0x00)
#define IPOL_ADDR                   (0x01)
#define GPINTEN_ADDR                (0x02)
#define DEFVAL_ADDR                 (0x03)
#define INTCON_ADDR                 (0x04)
#define IOCON_ADDR                  (0x05)
#define GPPU_ADDR                   (0x06)
#define INTF_ADDR                   (0x07)
#define INTCAP_ADDR                 (0x08)
#define GPIO_ADDR                   (0x09)
#define OLAT_ADDR                   (0x0A)

/* bit registers */
#define INTPOL_BIT                  2
#define ODR_BIT                     4
#define HAEN_BIT                    8
#define DISSLW_BIT                  16
#define SEQOP_BIT                   32

#define MCP_REGS                    11
#define OPCODE_PREFIX               0x08 /* 01000 */
#define READ_COMMAND                1

#define NUMBER_OF_PINS              8

enum Mode {sequential = 0, byte = 1};

class MCP23S08 : public ExternalPeripheral {
  public:
    MCP23S08();
    void Main() override;
    void Stop() override;

    MCP23S08(const MCP23S08&) = delete;
    MCP23S08& operator=(const MCP23S08&) = delete;

  private:
    void Reset();
    void HandleRead();
    void HandleWrite();
    void InitializePins();
    void WriteGPIO(const uint8_t data);
    void WriteOLAT(const uint8_t data);
    void WriteIPOL(const uint8_t data);
    void WriteIOCON(const uint8_t data);
    void WriteINTCAP();
    void ResetPinChanged(std::vector<WireLogicLevelEvent>& notifications);
    void GPIOPinChanged(std::vector<WireLogicLevelEvent>& notifications);
    void AddressPinsChanged(std::vector<WireLogicLevelEvent>& notifications);
    void SetPinsCallbacks();
    void SetDeviceAddress();
    void ThrowInterruprt();
    void SetINTOn();
    void SetINTOff();
    void SwitchINT();
    void WaitForInactiveSS();
    void UpdateGPIOReg(WireLogicLevelEvent notification);
    uint8_t WaitForData();
    uint8_t ReadGPIO();
    uint8_t ReadINTCAP();
    uint8_t ReadINTF();
    uint8_t ReadPinLevels();
    bool IsInterruptEnabled(uint8_t pin_bit);
    bool IsInputPin(uint8_t pin_bit);
    bool IsDEFVALCompare(uint8_t pin_bit);
    bool DEFVALEqual(uint8_t pin_bit, WireLogicLevelEvent notification);
    bool LastValEqual(uint8_t pin_bit, WireLogicLevelEvent notification);


    uint8_t registers_[MCP_REGS] = {0};
    uint8_t device_address_ {0};
    iSpiSlaveV2* spi_slave_ {};

    uint32_t pins_[NUMBER_OF_PINS] = {0};
    uint32_t a0_ {0};
    uint32_t a1_ {0};
    uint32_t reset_ {0};
    uint32_t int_ {0};

    Mode mode_ = sequential;

    bool hardware_addr_enabled_ = false;
    bool in_reset_mode = false;
    bool should_stop_ = false;
    bool int_active_high_ = true;

    std::map<uint32_t ,uint8_t > pins_to_index_;
};

DLL_EXPORT ExternalPeripheral *PeripheralFactory() {
    return new MCP23S08();
}
