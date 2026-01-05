#include "RenogyChargeController.h"
#include "DebugConfiguration.h"

RenogyStatus g_renogyStatus;

RenogyChargeController::RenogyChargeController(Stream &serial, uint8_t slaveId)
    : m_modbus(serial, slaveId)
{
}

enum RenogyChargingState {
    DEACTIVATED      = 0,
    ACTIVATED        = 1,
    MPPT_MODE        = 2,
    EQUALIZING_MODE  = 3,
    BOOST_MODE       = 4,
    FLOAT_MODE       = 5,
    CURRENT_LIMITING = 6,
};

static bool isCharging(uint16_t reg0120h)
{
    switch (reg0120h & 0x00FF)
    {
    case ACTIVATED:
    case MPPT_MODE:
    case EQUALIZING_MODE:
    case BOOST_MODE:
    case FLOAT_MODE:
        return true;

    default:
        return false;
    }
}

bool RenogyChargeController::poll()
{
    constexpr uint16_t DATA_START = 0x0100;
    constexpr uint16_t NUM_DATA   = 35;
    constexpr uint16_t INFO_START = 0x000A;
    constexpr uint16_t NUM_INFO   = 17;

    uint16_t data_regs[NUM_DATA];
    uint16_t info_regs[NUM_INFO];

    // Read data registers
    auto err = m_modbus.readHoldingRegisters(DATA_START, NUM_DATA);

    /**
     * TODO: Handle errors
     * - Retry on certain error types...
     */

    switch (err)
    {
    case ModbusRtuMaster::Error::OK:
        LOG_INFO("Read Renogy OK");
        break;
    
    case ModbusRtuMaster::Error::TIMEOUT:
        LOG_ERROR("Read Renogy TIMEOUT");
        break;
    
    case ModbusRtuMaster::Error::CRC_ERROR:
        LOG_ERROR("Read Renogy CRC_ERROR");
        break;
    
    case ModbusRtuMaster::Error::EXCEPTION:
        LOG_ERROR("Read Renogy EXCEPTION");
        break;
    
    case ModbusRtuMaster::Error::BAD_RESPONSE:
        LOG_ERROR("Read Renogy BAD_RESPONSE");
        break;
    
    default:
        break;
    }

    // Don't read from the response buffer if not OK
    if (err != ModbusRtuMaster::Error::OK) {
        m_status.valid = false;
        return false;
    }

    for (uint8_t i = 0; i < NUM_DATA; i++) {
        data_regs[i] = m_modbus.getResponseBuffer(i);
    }

    // Parse data into struct
    m_status.socPercent        = static_cast<int8_t>(data_regs[0]);
    m_status.batteryVoltage_mV = data_regs[1] * 100;
    m_status.batteryCharge_mA  = data_regs[2] * 10;
    m_status.loadVoltage_mV    = data_regs[4] * 100;
    m_status.loadCurrent_mA    = data_regs[5] * 10;
    m_status.panelVoltage_mV   = data_regs[7] * 100;
    m_status.panelCurrent_mA   = data_regs[8] * 10;
    m_status.pvPresent         = data_regs[7] != 0 ? true : false;
    m_status.loadOn            = (data_regs[32] & 0x8000) != 0 ? true : false;
    m_status.charging          = isCharging(data_regs[32]);
    m_status.lastUpdateMs      = millis();
    m_status.valid             = true;

    return true;
}

bool RenogyChargeController::setLoad(bool on)
{
    const uint16_t val = on ? 1 : 0;
    auto err = m_modbus.writeSingleRegister(0x010A, val);

    /**
     * TODO: Handle errors
     * - Retry on certain error types...
     */

    m_status.loadOn = on;
    return true;
}
