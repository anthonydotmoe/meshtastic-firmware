#include "ModbusRtuMaster.h"

#define lowWord(ww) ((uint16_t) ((ww) & 0xFFFF))
#define highWord(ww) ((uint16_t) ((ww) >> 16))
#define LONG(hi, lo) ((uint32_t) ((hi) << 16 | (lo)))

ModbusRtuMaster::ModbusRtuMaster(Stream &serial, uint8_t slaveId)
    : m_serial(serial), m_slaveId(slaveId)
{
}

/// @brief Retrieve data from response buffer.
/// @param index  index of response buffer array
/// @return value in position index of response buffer
uint16_t ModbusRtuMaster::getResponseBuffer(uint8_t index)
{
    if (index < m_maxBufferSize) {
        return m_responseBuffer[index];
    } else {
        return 0xFFFF;
    }
}

/// @brief Clear Modbus response buffer.
void ModbusRtuMaster::clearResponseBuffer()
{
    for (uint8_t i = 0; i < m_maxBufferSize; i++) {
        m_responseBuffer[i] = 0;
    }
}

/// @brief Modbus function 0x06 Write Single Register.
/// @param regAddr address of the holding register (0x0000..0xFFFF)
/// @param value value to be written to holding register (0x0000..0xFFFF)
/// @return ModbusRtuMaster::Error
ModbusRtuMaster::Error ModbusRtuMaster::writeSingleRegister(uint16_t regAddr, uint16_t value)
{
    m_writeAddress = regAddr;
    m_writeQty = 0;
    m_transmitBuffer[0] = value;
    return transact(WriteSingleRegister);
}

/// @brief Modbus function 0x03 Read Holding Registers.
/// @param startAddr address of the first holding register (0x0000..0xFFFF)
/// @param count quantity of holding registers to read (1..125, enforced by remote device)
/// @return ModbusRtuMaster::Error
ModbusRtuMaster::Error ModbusRtuMaster::readHoldingRegisters(uint16_t startAddr, uint16_t count)
{
    m_readAddress = startAddr;
    m_readQty = count;
    return transact(ReadHoldingRegisters);
}

// https://github.com/syvic/ModbusMaster
ModbusRtuMaster::Error ModbusRtuMaster::transact(ModbusFunction func)
{
    uint8_t modbusADU[256];
    uint8_t modbusADUSize = 0;
    uint16_t crc;
    uint32_t startTime;
    uint8_t i;
    uint8_t bytesLeft = 8;
    Error status = OK;

    // Assemble Modbus request ADU
    modbusADU[modbusADUSize++] = m_slaveId;
    modbusADU[modbusADUSize++] = static_cast<uint8_t>(func);

    switch (func) {
        case ReadHoldingRegisters:
            modbusADU[modbusADUSize++] = highByte(m_readAddress);
            modbusADU[modbusADUSize++] = lowByte(m_readAddress);
            modbusADU[modbusADUSize++] = highByte(m_readQty);
            modbusADU[modbusADUSize++] = lowByte(m_readQty);
            break;
    }

    switch (func) {
        case WriteSingleRegister:
            modbusADU[modbusADUSize++] = highByte(m_writeAddress);
            modbusADU[modbusADUSize++] = lowByte(m_writeAddress);
            break;
    }

    switch (func) {
        case WriteSingleRegister:
            modbusADU[modbusADUSize++] = highByte(m_transmitBuffer[0]);
            modbusADU[modbusADUSize++] = lowByte(m_transmitBuffer[0]);
            break;
    }

    // Append CRC
    crc = crc16(modbusADU, modbusADUSize);
    modbusADU[modbusADUSize++] = lowByte(crc);
    modbusADU[modbusADUSize++] = highByte(crc);
    modbusADU[modbusADUSize++] = 0;

    // Transmit request
    for (i = 0; i < modbusADUSize; i++) {
        m_serial.write(modbusADU[i]);
    }

    modbusADUSize = 0;
    m_serial.flush();

    // loop until we run out of time or bytes, or an error occurs
    startTime = millis();
    while (bytesLeft && (status == OK)) {
        if (m_serial.available()) {
            modbusADU[modbusADUSize++] = m_serial.read();
            bytesLeft--;
        /*
        } else if (m_idle) {
            idle();
        */
        }

        // evaluate slave ID, function code once enough bytes have been read
        if (modbusADUSize == 5) {
            // verify response is for correct Modbus slave
            if (modbusADU[0] != m_slaveId) {
                status = BAD_RESPONSE;
                break;
            }

            // verify response is for correct Modbus function code (mask exception bit 7)
            if ((modbusADU[1] & 0x7F) != func) {
                status = BAD_RESPONSE;
                break;
            }

            // check whether Modbus exception occurred; return Modbus Exception Code
            if (bitRead(modbusADU[1], 7)) {
                status = EXCEPTION;
                // TODO: maybe return exception code?? modbusADU[2]
                break;
            }

            // evaluate returned Modbus function code
            switch(modbusADU[1]) {
                case ReadHoldingRegisters:
                    bytesLeft = modbusADU[2];
                    break;

                case WriteSingleRegister:
                    bytesLeft = 3;
                    break;
            }
        }

        if (millis() > (startTime + m_timeoutMs)) {
            status = TIMEOUT;
        }
    }

    // verify response is large enough to inspect further
    if ((status == OK) && modbusADUSize >= 5)
    {
        // calculate CRC
        crc = crc16(modbusADU, modbusADUSize - 2);

        // verify CRC
        if (lowByte(crc) != modbusADU[modbusADUSize - 2] || highByte(crc) != modbusADU[modbusADUSize - 1]) {
            status = CRC_ERROR;
        }
    }

    // disassemble ADU into words
    if (status == OK)
    {
        // evaluate returned Modbus function code
        switch(modbusADU[1]) {
            case ReadHoldingRegisters:
                // load bytes into word; response bytes are ordered H, L, H, L, ...
                for (i = 0; i < (modbusADU[2] >> 1); i++) {
                    if (i < m_maxBufferSize) {
                        m_responseBuffer[i] = word(modbusADU[2 * i + 3], modbusADU[2 * i + 4]);
                    }

                    m_responseBufferLength = i;
                }
                break;
        }
    }

//  m_transmitBufferIndex = 0;
//  m_transmitBufferLength = 0;
    m_responseBufferIndex = 0;
    return status;
}

uint16_t ModbusRtuMaster::crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
