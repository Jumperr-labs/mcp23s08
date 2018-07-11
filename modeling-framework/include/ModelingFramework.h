#pragma once
#include <string>
#include <functional>
#include "iSpiSlave.h"
#include "iI2cSlave.h"
#include "WireManagerTypes.h"

#ifdef _WINDOWS
    #define DLL_EXPORT extern "C" __declspec(dllexport)
#else
    #define DLL_EXPORT extern "C"
#endif

/**
 * This function returns the pin number that the pin name is connected to according to board.json file
 * @param pin_name (as it appears on board.json file)
 * @return pin number
 */
int GetPinNumber(const std::string &pin_name);

/**
  * This function returns an instance of SPI slave
  * @param spi_config that configure the spi slave
  * @return SPI slave
*/
iSpiSlave* CreateSpiSlave(SpiSlaveConfig &spi_config);

/**
  * This function returns an instance of I2C slave
  * @return I2C slave
*/
iI2cSlave* CreateI2cSlave();

/**
 * This function returns the current level of a given pin name
 * @param pin_name (as it appears on board.json file), that his level returns
 * @return true, if pin level is high, otherwise false
 */
bool GetPinLevel(int pin_number);

/**
 * This function returns the current level of a given pin name
 * @param pin_name (as it appears on board.json file), that his level returns
 * @return true, if pin level is high, otherwise false
 */
bool GetPinLevel(const std::string &pin_name);

/**
 * This function sets the level of a given pin number
 * @param pin_number to set his level
 * @param pin_level true, in order to set the level to high, otherwise false
 */
void SetPinLevel(int pin_number, bool pin_level);

/**
 * This function returns the current level of a given pin name
 * @param pin_name (as it appears on board.json file), to set his level
 * @param pin_level true, in order to set the level to high, otherwise false
 */
void SetPinLevel(const std::string &pin_name, bool pin_level);

/* Peripheral Timed Callback */
typedef std::function<void ()> callback_t;

/**
 * This function calls the callback function after ns of emulation time
 * @param ns number of nanoseconds in emulation time that will pass until the callback will be called
 * @param callback function that will be called after times up (this function has no input and no outputs)
 * @param run_once true, if the function should be called more then once
 * @return time request id
 */
int AddTimedCallback(uint64_t ns, const callback_t &callback, bool run_once = false);

/***
 * This function cancel the callback.
 * @param id request id that returned from AddTimedCallback function
 */
void CancelTimedCallback(int id);

/**
 * This function updates time request, change the time until the callback function will called
 * @param id request id that returned from AddTimedCallback function
 * @param ns number of nanoseconds in emulation time that will pass until the callback will be called
 */
void UpdateTimedCallback(int id, uint64_t ns);

/**
 * This function returns next int32 value from a given data generator name
 * @param name data generator name, to read from
 * @return the value that was read from data generator
 */
int32_t GetNextInt32FromDataGenerator(std::string name);

/**
 * This function returns next double value from a given data generator name
 * @param name data generator name, to read from
 * @return the value that was read from data generator
 */
double GetNextDoubleFromDataGenerator(std::string name);

/**
 * This function returns next int32 value from a given data generator name
 * @param name data generator name, to read from
 * @return the value that was read from data generator
 */
int16_t GetNextInt16FromDataGenerator(std::string name);

/**
 * This function returns next uint8 value from a given data generator name
 * @param name data generator name, to read from
 * @return the value that was read from data generator
 */
uint8_t GetNextUInt8FromDataGenerator(std::string name);

/**
 * This function returns the last double value that was read from a given data generator name
 * @param name data generator name, to read from
 * @return the value that was cached from data generator
 */
double GetCachedValueFromDataGenerator(std::string name);

/**
 * This function sets multiple pins levels
 * @param pin_changes vector of pin change
 */
void SetMultiplePinsLevel(const pin_level_changes_t& pin_changes);

/**
 * This function set pin direction (input/output)
 * @param pin_id to change his direction
 * @param direction the direction of the pin
 * @param pullup set true, if the pin set to high, otherwise false
 */
void SetPinDirection(uint32_t pin_id,  pin_direction_t direction, bool pullup);

/**
 * This function set callback on pin level change event
 * @param pin_id that change in his level will call the the callback
 * @param pin_change_level_callback callback function to be called on change level event
 * @return callback id
 */
int SetPinChangeLevelEventCallback(uint32_t pin_id, const pin_change_level_callback_t &callback);


class ExternalPeripheral {
  public:
    virtual void Main() = 0;
    virtual void Stop() = 0;
};

typedef ExternalPeripheral* peripheral_factory_t();