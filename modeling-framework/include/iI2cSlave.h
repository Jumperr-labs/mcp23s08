#pragma once
#include <functional>

typedef bool ack;

class iI2cSlaveBaseReadStreamV1 {
  public:
    /// Sends an ACK and then reads a byte to \p rx_buffer.
    /// Repeats \p size times or until a STOP signal was received from the master.
    /// \param rx_buffer Buffer to hold data from the I2C master.
    /// \param size Maximum bytes to be read into \p rx_buffer.
    /// \return Number or bytes read into \p rx_buffer.
    ///         If less than \p size, the corresponding instance of iI2cSlaveBaseReadStream is no longer active and the write callback should return immediately.
    virtual size_t ReadAndSendAck(uint8_t *rx_buffer, size_t size) = 0;

    /// \return true if a stop signal was received during the current transaction, otherwise returns false.
    virtual bool ReceivedStop() = 0;
};

class iI2cSlaveBaseWriteStreamV1 {
  public:
    /// Sends data to the I2C master.
    /// \param tx_buffer A buffer that holds the data to be sent to the I2C master.
    /// \param size Maximum number of bytes to be sent.
    /// \return Number of bytes written to the I2C master.
    ///         If less than \p size, a NACK was received from the master.
    virtual size_t Write(uint8_t* tx_buffer, size_t size) = 0;

    /// \return true if a NACK was received during the current transaction, otherwise returns false.
    virtual bool ReceivedNack() = 0;
};

typedef iI2cSlaveBaseReadStreamV1 iI2cSlaveBaseReadStream;
typedef iI2cSlaveBaseWriteStreamV1 iI2cSlaveBaseWriteStream;

/// Called by an instance of iSpiSlave when a the I2C master writes to the slave.
/// Receives a iI2cSlaveBaseReadStream to read data from.
/// If this function returns without setting the last ACK/NACK, an NACK signal will be assumed and sent to the master for the remaining bytes.
typedef std::function<void(uint8_t, iI2cSlaveBaseReadStream&)> i2c_write_command_callback_t;

/// Called by an instance of iSpiSlave when a the I2C master reads from the slave.
/// Receives a iI2cSlaveBaseWriteStream.
typedef std::function<void(uint8_t, iI2cSlaveBaseWriteStream&)> i2c_read_command_callback_t;

struct iI2cMasterWriteCommandCallback {
    virtual void OnMasterWriteCommandCallback(uint8_t address, iI2cSlaveBaseReadStream& stream) = 0 ;
};

template<class T>
struct I2cMasterWriteCommandCallback : public iI2cMasterWriteCommandCallback{
    I2cMasterWriteCommandCallback(const T& callback) : callback_(callback) {}
    void OnMasterWriteCommandCallback(uint8_t address, iI2cSlaveBaseReadStream& stream) { callback_(address, stream); }
    T callback_{};
};

struct iI2cMasterReadCommandCallback {
    virtual void OnMasterReadCommandCallback(uint8_t address, iI2cSlaveBaseWriteStream& stream) = 0 ;
};

template<class T>
struct I2cMasterReadCommandCallback : public iI2cMasterReadCommandCallback {
    I2cMasterReadCommandCallback(const T& callback) : callback_(callback) {}
    void OnMasterReadCommandCallback(uint8_t address, iI2cSlaveBaseWriteStream& stream) { callback_(address, stream); }
    T callback_{};
};

class iI2cSlaveBaseV1 {
  public:
    /// Sets a callback to be called when the I2C master is attempting to write to the slave.
    /// \param callback
    template<class i2c_write_command_callback>
    void SetMasterWriteCommandCallback( const i2c_write_command_callback& callback) {
        I2cMasterWriteCommandCallback_ = new I2cMasterWriteCommandCallback<i2c_write_command_callback>(callback);
    }

    /// Sets a callback to be called when the I2C master is attempting to read from the slave.
    /// \param callback
    template<class i2c_read_command_callback>
    void SetMasterReadCommandCallback(const i2c_read_command_callback& callback) {
        I2cMasterReadCommandCallback_ = new I2cMasterReadCommandCallback<i2c_read_command_callback>(callback);;
    }

    virtual void SetSlaveAddress(uint8_t address, uint8_t address_mask) = 0;
    
    iI2cMasterWriteCommandCallback* I2cMasterWriteCommandCallback_{};
    iI2cMasterReadCommandCallback* I2cMasterReadCommandCallback_{};
};

typedef iI2cSlaveBaseV1 iI2cSlave;

