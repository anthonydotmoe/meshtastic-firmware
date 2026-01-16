#include "ModbusRtuMaster.h"
#include "DebugConfiguration.h"

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

static inline void append_u16(uint8_t *buf, uint8_t &len, uint16_t value)
{
    buf[len++] = highByte(value);
    buf[len++] = lowByte(value);
}

// https://github.com/syvic/ModbusMaster
ModbusRtuMaster::Error ModbusRtuMaster::transact(ModbusFunction func)
{
    uint8_t  adu[256];
    uint8_t  aduSize   = 0;
    uint16_t crc;
    uint32_t startTime;
    uint8_t  i;
    uint8_t  bytesLeft = 8;  // initial minimum to decide frame shape
    Error    status    = OK;

    // Header
    adu[aduSize++] = m_slaveId;
    adu[aduSize++] = static_cast<uint8_t>(func);

    // PDU
    switch (func) {
        case ReadHoldingRegisters:
            append_u16(adu, aduSize, m_readAddress);
            append_u16(adu, aduSize, m_readQty);
            break;

        case WriteSingleRegister:
            append_u16(adu, aduSize, m_writeAddress);
            append_u16(adu, aduSize, m_transmitBuffer[0]);
            break;
    }

    // CRC
    crc = crc16(adu, aduSize);
    adu[aduSize++] = lowByte(crc);
    adu[aduSize++] = highByte(crc);

    LOG_DEBUG("Modbus TX (func 0x%02X, len %u):", (uint8_t)func, aduSize);

    char hexbuf[256];
    char *w = hexbuf;

    for (uint8_t j = 0; j < aduSize; j++) {
        // Write "HH " into hexbuf
        int n = snprintf(w, sizeof(hexbuf) - (w - hexbuf), "%02X ", adu[j]);
        if (n <= 0) break;
        w += n;
    }

    // Null terminate
    *w = '\0';

    // Emit entire hex string
    LOG_DEBUG("%s", hexbuf);

    // Transmit
    for (i = 0; i < aduSize; i++) {
        m_serial.write(adu[i]);
    }
    m_serial.flush();

    // Receive
    aduSize   = 0;
    startTime = millis();

    while (bytesLeft && (status == OK) && (millis() - startTime) < m_timeoutMs) {
        if (m_serial.available()) {
            adu[aduSize++] = m_serial.read();
            bytesLeft--;

            // evaluate slave ID, function code once enough bytes have been read
            if (aduSize == 5) {
                // verify response is for correct Modbus slave
                if (adu[0] != m_slaveId) {
                    status = BAD_RESPONSE;
                    break;
                }

                // verify response is for correct Modbus function code (mask exception bit 7)
                if ((adu[1] & 0x7F) != static_cast<uint8_t>(func)) {
                    status = BAD_RESPONSE;
                    break;
                }

                // check whether Modbus exception occurred; return Modbus Exception Code
                if (bitRead(adu[1], 7)) {
                    status = EXCEPTION;
                    // TODO: maybe return exception code?? adu[2]
                    break;
                }

                // evaluate returned Modbus function code
                switch (adu[1]) {
                    case ReadHoldingRegisters:
                        bytesLeft = adu[2];  // byte count
                        break;

                    case WriteSingleRegister:
                        bytesLeft = 3;       // addrHi, addrLo, valHi, valLo -> 4 bytes,
                                             // but we already have func+id+first byte
                        break;
                }
            }
        }
        /*
        else if (m_idle) {
            idle();
        }
        */
    }

    if (status == OK && bytesLeft) {
        status = TIMEOUT;
    }

    // verify response is large enough to inspect further
    if ((status == OK) && aduSize >= 5) {
        // Log the received bytes
        if (aduSize > 0) {
            char rxbuf[256];
            char *w = rxbuf;

            for (uint8_t j = 0; j < aduSize; j++) {
                int n = snprintf(w, sizeof(rxbuf) - (w - rxbuf), "%02X ", adu[j]);
                if (n <= 0) break;
                w += n;
            }
            *w = '\0';

            LOG_DEBUG("Modbus RX (len %u): %s", aduSize, rxbuf);
        }

        // calculate CRC
        crc = crc16(adu, aduSize - 2);

        // verify CRC
        if (lowByte(crc) != adu[aduSize - 2] ||
            highByte(crc) != adu[aduSize - 1]) {
            status = CRC_ERROR;
        }
    }

    // disassemble ADU into words
    if (status == OK) {
        switch (adu[1]) {
            case ReadHoldingRegisters: {
                uint8_t  byteCount         = adu[2];
                uint16_t expectedByteCount = m_readQty * 2;
                if (byteCount != expectedByteCount) {
                    status = BAD_RESPONSE; // length mismatch
                    break;
                }

                uint8_t numRegs = byteCount / 2;
                uint8_t maxRegs = (numRegs <= m_maxBufferSize) ? numRegs : m_maxBufferSize;

                // load bytes into word; response bytes are ordered H, L, H, L, ...
                for (i = 0; i < maxRegs; i++) {
                    m_responseBuffer[i] = word(adu[2 * i + 3], adu[2 * i + 4]);
                }
                break;
            }

            case WriteSingleRegister: {
                if (aduSize != 8) {
                    status = BAD_RESPONSE;
                    break;
                }

                uint16_t addr  = word(adu[2], adu[3]);
                uint16_t value = word(adu[4], adu[5]);

                if (addr != m_writeAddress || value != m_transmitBuffer[0]) {
                    status = BAD_RESPONSE;
                }
                break;
            }
        }
    }

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
