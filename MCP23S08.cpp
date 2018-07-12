#include "MCP23S08.h"
#include <iostream>


MCP23S08::MCP23S08(): pins_to_index_() {

    SpiSlaveConfig spi_config;

    spi_config.mosi_pin_number = GetPinNumber("si");
    spi_config.miso_pin_number = GetPinNumber("so");
    spi_config.ss_pin_number = GetPinNumber("cs");
    spi_config.sclk_pin_number = GetPinNumber("sck");
    spi_config.supported_spi_modes = SPI_MODE_CPOL0_CPHA0;
    spi_config.max_frequency = 10000000;
    spi_config.bit_order = MSB_FIRST;

    spi_slave_ = CreateSpiSlave(spi_config);
    InitializePins();
    Reset();
}

void MCP23S08::InitializePins() {
    pins_[0] = GetPinNumber("M0");
    pins_[1] = GetPinNumber("M1");
    pins_[2] = GetPinNumber("M2");
    pins_[3] = GetPinNumber("M3");
    pins_[4] = GetPinNumber("M4");
    pins_[5] = GetPinNumber("M5");
    pins_[6] = GetPinNumber("M6");
    pins_[7] = GetPinNumber("M7");


    a0_    = GetPinNumber("A0");
    a1_    = GetPinNumber("A1");
    int_   = GetPinNumber("INT");
    reset_ = GetPinNumber("RESET");

    SetINTOff();

    pins_to_index_ = {{pins_[0], 0}, {pins_[1], 1}, {pins_[2], 2}, {pins_[3], 3},
                      {pins_[4], 4}, {pins_[5], 5}, {pins_[6], 6}, {pins_[7], 7}};

    SetPinsCallbacks();
}


void MCP23S08::AddressPinsChanged(std::vector<WireLogicLevelEvent>& notifications) {
    SetDeviceAddress();
}

void MCP23S08::ResetPinChanged(std::vector<WireLogicLevelEvent>& notifications) {

    WireLogicLevelEvent notification = notifications[0];

    if (notification.type == FALLING) {
        in_reset_mode = true;
        Reset();
    }
    else if (notification.type == RISING) {
        in_reset_mode = false;
    }
}

// Before throwing interrupt - read all pin levels and write them into INTCAP register
void MCP23S08::ThrowInterruprt() {
    WriteINTCAP();
    SetINTOn();
}

void MCP23S08::SetINTOff() {
    if (int_active_high_) {
        SetPinLevel(int_, false);
    }
    else {
        SetPinLevel(int_, true);
    }
    registers_[INTF_ADDR] = 0;
}


void MCP23S08::SetINTOn() {

    if (int_active_high_) {
        SetPinLevel(int_, true);
    }
    else {
        SetPinLevel(int_, false);
    }
}

bool MCP23S08::IsInterruptEnabled(uint8_t pin_bit) {
    return (registers_[GPINTEN_ADDR] & pin_bit) == 1;
}

bool MCP23S08::IsInputPin(uint8_t pin_bit) {
    return (registers_[IODIR_ADDR] & pin_bit) == 1;
}

bool MCP23S08::IsDEFVALCompare(uint8_t pin_bit) {
    return (registers_[INTCON_ADDR] & pin_bit) == 1;
}

// Compare between pin notification to the value of the associated bit in DEFVAL
// If equal - return true. Else - return false.
bool MCP23S08::DEFVALEqual(uint8_t pin_bit, WireLogicLevelEvent notification) {
    if ((((registers_[DEFVAL_ADDR] && pin_bit) != 0) && (notification.type = FALLING)) ||
            (((registers_[DEFVAL_ADDR] && pin_bit) == 0) && (notification.type = RISING))) {
        return false;
    }

    return true;
}

// Compare between pin notification to the previous value in the port
// If equal - return true. Else - return false.
bool MCP23S08::LastValEqual(uint8_t pin_bit, WireLogicLevelEvent notification) {

    bool gpio_on = ((registers_[GPIO_ADDR] ^ registers_[IPOL_ADDR]) & pin_bit);

    if ((gpio_on && notification.type == FALLING) ||
        (!gpio_on && notification.type == RISING)) {
        return false;
    }

    return true;
}


// Get notification of pin change and update the GPIO
void MCP23S08::UpdateGPIOReg(WireLogicLevelEvent notification) {

    uint8_t pin_bit = 0x01 << pins_to_index_[notification.wire_number];

    if(notification.type == RISING) {
        if((registers_[IPOL_ADDR] & pin_bit) == 0) { // No polarity on this pin
            registers_[GPIO_ADDR] |= pin_bit;
        }
        else {
            registers_[GPIO_ADDR] &= ~pin_bit;
        }
    }
    else { // FALLING
        if((registers_[IPOL_ADDR] & pin_bit) == 0) { // No polarity on this pin
            registers_[GPIO_ADDR] &= ~pin_bit;
        }
        else {
            registers_[GPIO_ADDR] |= pin_bit;
        }
    }

}


// Callback for GPIO change. Should raise interrupt to master for new input, if needed.
void MCP23S08::GPIOPinChanged(std::vector<WireLogicLevelEvent>& notifications) {

    uint8_t pin_index = 0;
    uint8_t interrupts = 0; // bitmask representing the interrupts of the pins

    for (WireLogicLevelEvent notification : notifications) {

        pin_index = pins_to_index_[notification.wire_number];
        uint8_t pin_bit = 0x01 << pin_index;
        // Raising interrupts only on input pins
        if (IsInputPin(pin_bit)) {
            UpdateGPIOReg(notification);

            if (IsInterruptEnabled(pin_bit)) {

                if (IsDEFVALCompare(pin_bit)) {
                    if (!DEFVALEqual(pin_bit, notification)) {
                        interrupts |= pin_bit;
                    }
                }

                    // Need to compare with last value of port
                else {
                    if (!LastValEqual(pin_bit, notification)) {
                        interrupts |= pin_bit;
                    }
                }
            }
        }

        registers_[INTF_ADDR] |= interrupts;

        if(interrupts) {
            ThrowInterruprt();
        }
    }
}


void MCP23S08::SetPinsCallbacks() {
    SetPinChangeLevelEventCallback(reset_, std::bind(&MCP23S08::ResetPinChanged,this,std::placeholders::_1));

    SetPinChangeLevelEventCallback(a0_, std::bind(&MCP23S08::AddressPinsChanged,this,std::placeholders::_1));
    SetPinChangeLevelEventCallback(a1_, std::bind(&MCP23S08::AddressPinsChanged,this,std::placeholders::_1));

    SetPinChangeLevelEventCallback(pins_[0], std::bind(&MCP23S08::GPIOPinChanged,this,std::placeholders::_1));
    SetPinChangeLevelEventCallback(pins_[1], std::bind(&MCP23S08::GPIOPinChanged,this,std::placeholders::_1));
    SetPinChangeLevelEventCallback(pins_[2], std::bind(&MCP23S08::GPIOPinChanged,this,std::placeholders::_1));
    SetPinChangeLevelEventCallback(pins_[3], std::bind(&MCP23S08::GPIOPinChanged,this,std::placeholders::_1));
    SetPinChangeLevelEventCallback(pins_[4], std::bind(&MCP23S08::GPIOPinChanged,this,std::placeholders::_1));
    SetPinChangeLevelEventCallback(pins_[5], std::bind(&MCP23S08::GPIOPinChanged,this,std::placeholders::_1));
    SetPinChangeLevelEventCallback(pins_[6], std::bind(&MCP23S08::GPIOPinChanged,this,std::placeholders::_1));
    SetPinChangeLevelEventCallback(pins_[7], std::bind(&MCP23S08::GPIOPinChanged,this,std::placeholders::_1));
}

uint8_t MCP23S08::WaitForData() {

    uint8_t byte_received = 0;
    uint8_t data = 0;

    while (byte_received == 0) {
        byte_received =  spi_slave_->Transmit(&data, nullptr, 1);
    }

    return data;
}

void MCP23S08::HandleRead() {

    uint8_t address = 0;
    uint8_t data = 0;
    uint8_t byte_received = spi_slave_->Transmit(&address, nullptr, 1);

    // Continue as long as CS is activated
    while (byte_received != 0) {
        bool ss_active = spi_slave_->WaitForMasterTransmit();

        if(!ss_active) {
            break;
        }

        switch (address) {
            case GPIO_ADDR:
                data = ReadGPIO();
                break;
            case INTF_ADDR:
                data = ReadINTF();
                break;
            case INTCAP_ADDR:
                data = ReadINTCAP();
                break;
            default:
                data = registers_[address];
        }

        // Send the value to the master
        byte_received = spi_slave_->Transmit(nullptr, &data, 1);

        // Increment address pointer in case of sequential mode
        if (mode_ == sequential) {
            address = (address + 1) % 10;
        }
    }
}

uint8_t MCP23S08::ReadINTCAP() {

    uint8_t value = registers_[INTCAP_ADDR];
    registers_[INTCAP_ADDR] = 0;
    // Clear Output Interrupt. Should be done on GPIO read or INTCAP read
    SetINTOff();

    return value;
}

// Reads all the 8 GPIO pins and return byte represents the pins that are set
// Each bit represents pin according to pin index
uint8_t MCP23S08::ReadPinLevels() {
    uint8_t pin_levels = 0;

    // Go over all the pins and set the relevant bit if the pin level is 1
    for(int i = 0; i < NUMBER_OF_PINS; i++) {
        if(GetPinLevel(pins_[i]) == 1) {
            pin_levels = pin_levels | (0x01 << i);
        }
    }

    return pin_levels;
}


// INTCAP register is read only register, and can be changed only from the hardware each time interrupt occurred
void MCP23S08::WriteINTCAP() {
    registers_[INTCAP_ADDR] = ReadPinLevels();
}


void MCP23S08::SetDeviceAddress() {
    bool a1_high = GetPinLevel(a1_);
    bool a0_high = GetPinLevel(a0_);
    device_address_ =  (a1_high << 1) | a0_high;
}


// Reading from GPIO register reads actual the port value of the input pins, and read from GPIO register the output pins
uint8_t MCP23S08::ReadGPIO() {

    const uint8_t input_pins = registers_[IODIR_ADDR];
    const uint8_t output_pins = ~input_pins;

    uint8_t gpio_input_pins = (ReadPinLevels() ^ registers_[IPOL_ADDR]) & input_pins;  // XOR with polarity register and mask with input pins
    uint8_t gpio_val = gpio_input_pins | (registers_[GPIO_ADDR] & output_pins); // Add output pins from GPIO register

    // Clear Output Interrupt. Should be done on GPIO read or INTCAP read
    SetINTOff();
    registers_[INTCAP_ADDR] = 0;

    return gpio_val;
}

uint8_t MCP23S08::ReadINTF() {

    uint8_t reg_val = registers_[INTF_ADDR];
    return reg_val;
}




// Writing to OLAT register modifies the pins configured as output
void MCP23S08::WriteOLAT(const uint8_t data) {

    registers_[OLAT_ADDR] = data;

    // Bitmask of all the pins defined as output
    const uint8_t output_pins = ~registers_[IODIR_ADDR];

    uint8_t output_pin = output_pins & 0x01;
    uint8_t pin_level = (data ^ registers_[IPOL_ADDR]) & 0x01;

    // Go over all the pins and set/unset their level if they defined as output pins, according to "data"
    for (int i = 0; i < NUMBER_OF_PINS; i++) {
        // Check if the curr pin is output pin
        if (output_pin) {
            SetPinLevel(pins_[i], pin_level);
        }
        // Go to next pin
        output_pin = (output_pins >> (i + 1)) & 0x01;
        pin_level = ((data ^ registers_[IPOL_ADDR]) >> (i + 1)) & 0x01;
    }
}

// Writing to GPIO modifies OLAT register
void MCP23S08::WriteGPIO(const uint8_t data) {
    registers_[GPIO_ADDR] = data;
    WriteOLAT(data);
}

// IPOL defines the polarity - each bit represents the associated pin.
// "1" - the pin is opposite to gpio register.
// "0" - the pin is the same as the gpio register
void MCP23S08::WriteIPOL(const uint8_t data) {
        registers_[IPOL_ADDR] = data;
        WriteGPIO(registers_[GPIO_ADDR]); // Write GPIO again to update the uutput pins according to the new polarity

}


void MCP23S08::SwitchINT() {
    if(GetPinLevel(int_) == 1) {
        SetPinLevel(int_, 0);
    }
    else {
        SetPinLevel(int_, 1);
    }
}

void MCP23S08::WriteIOCON(const uint8_t data) {

    // Handle enable hardware bit
    if (data & HAEN_BIT) {
        hardware_addr_enabled_ = true;
    }
    else {
        hardware_addr_enabled_ = false;
    }

    // Handle sequential mode
    if (data & SEQOP_BIT) {
        mode_ = byte;
    }
    else {
        mode_ = sequential;
    }

    // Handle polarity of INT
    if ((registers_[IOCON_ADDR] & INTPOL_BIT) != (data & INTPOL_BIT)) {
        SwitchINT();
        if (data & INTPOL_BIT) {
            int_active_high_ = true;
        } else {
            int_active_high_ = false;
        }
    }

    registers_[IOCON_ADDR] = data;
}

void MCP23S08::HandleWrite() {

    uint8_t dataToWrite = 0;
    uint8_t address = 0;

    if(in_reset_mode) {
        std::cout << "Error: can't write to registers while reset mode.\r\n" << std::endl;
    }

    // Get register address
    if (spi_slave_->Transmit(&address, nullptr, 1) == 0) {
        return;  // CS was deactivated
    }

    /* Write registers values to the master.
       Case of sequential mode - send the register values sequentially
       Case of byte mode - send the same register value */
    while (spi_slave_->Transmit(&dataToWrite, nullptr, 1) != 0) {
        switch (address) {
            case GPIO_ADDR:
                WriteGPIO(dataToWrite);
                break;
            case IPOL_ADDR:
                WriteIPOL(dataToWrite);
                break;
            case IOCON_ADDR:
                WriteIOCON(dataToWrite);
                break;
            case OLAT_ADDR:
                WriteOLAT(dataToWrite);
                break;
            case INTCAP_ADDR:
                std::cout << "INTCAP register is read only register - writing INTCAP is not allowed";
                break;
            case INTF_ADDR:
                std::cout << "INTF register is read only register - writing INTF is not allowed";
                break;
            default:
                registers_[address] = dataToWrite;
        }

        // Increment address pointer
        if (mode_ == sequential) {
            address = (address + 1) % 10;
        }
    }
}

void MCP23S08::WaitForInactiveSS() {
    uint8_t opcode = 0;
    while (spi_slave_->Transmit(&opcode, nullptr, 1) != 0) {}
}

void MCP23S08::Main() {

    while (!should_stop_) {
        uint8_t opcode = 0;
        uint8_t hardware_addr = 0;
        const uint8_t byte_received = spi_slave_->Transmit(&opcode, nullptr, 1);
        if (byte_received == 0) {
            continue;
        }

        uint8_t opcode_prefix = opcode >> 3; // opcode prefix is 5 first bits in the opcode

        // Opcode must start with fixed prefix
        if (opcode_prefix != OPCODE_PREFIX) {
            continue;
        }

        // A0 & A1 bits - bits 1 & 2 - defines the hardware address
        hardware_addr = ((opcode >> 1) & 0x03);

        // If hardware address is enabled - the command address must be equal to the device address
        if (!hardware_addr_enabled_ ||
                (hardware_addr_enabled_ && device_address_ == hardware_addr)) {
            // Read command - bit indexed 0 has value 1
            if (opcode & READ_COMMAND) {
                HandleRead();
            }
            // Write command - bit indexed 0 has value 0
            else {
                HandleWrite();
            }
        }
        else {
            WaitForInactiveSS();
        }
    }
}

void MCP23S08::Stop() {
    should_stop_ = true;
}

// Reset all registers of MCP23S08
// All regs reset values are zero except IODIR register, which should be 0xFF
void MCP23S08::Reset() {
   std::fill(registers_, registers_ + MCP_REGS, 0);
   registers_[IODIR_ADDR] = 0XFF;

   WriteGPIO(0); // For reset output pin levels
}