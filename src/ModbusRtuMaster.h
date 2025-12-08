#pragma once

#include "Arduino.h"

class ModbusRtuMaster {
public:
    enum Error {
        OK = 0,
        TIMEOUT,
        CRC_ERROR,
        EXCEPTION, ///< Modbus exception response
        BAD_RESPONSE
    };

    ModbusRtuMaster(Stream &serial, uint8_t slaveId);

    uint16_t getResponseBuffer(uint8_t index);
    void     clearResponseBuffer();
    //uint8_t  setTransmitBuffer(uint8_t, uint16_t);
    //void     clearTransmitBuffer();

    // Read holding registers (0x03)
    Error readHoldingRegisters(uint16_t startAddr, uint16_t count);

    // Write single register (0x06)
    Error writeSingleRegister(uint16_t regAddr, uint16_t value);

private:
    Stream   &m_serial;
    uint8_t   m_slaveId;

    static const uint32_t  m_timeoutMs = 200;
    static const uint8_t   m_maxBufferSize = 64;

    uint16_t  m_readAddress;
    uint16_t  m_readQty;
    uint16_t  m_responseBuffer[m_maxBufferSize];
//  uint8_t   m_responseBufferIndex;
//  uint16_t  m_responseBufferLength;
    uint16_t  m_writeAddress;
    uint16_t  m_writeQty;
    uint16_t  m_transmitBuffer[m_maxBufferSize];
//  uint8_t   m_transmitBufferIndex;
//  uint16_t  m_transmitBufferLength;

    enum ModbusFunction: uint8_t {
//      ReadCoils                  = 0x01,
//      ReadDiscreteInputs         = 0x02,
//      WriteSingleCoil            = 0x05,
//      WriteMultipleCoils         = 0x0F,
        ReadHoldingRegisters       = 0x03,
//      ReadInputRegisters         = 0x04,
        WriteSingleRegister        = 0x06,
//      WriteMultipleRegisters     = 0x10,
//      MaskWriteRegister          = 0x16,
//      ReadWriteMultipleRegisters = 0x17,
    };

    Error transact(ModbusFunction func);

    static uint16_t crc16(const uint8_t *data, size_t len);
};
