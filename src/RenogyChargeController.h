#pragma once

#include "Arduino.h"
#include "ModbusRtuMaster.h"

struct RenogyStatus {
    uint32_t lastUpdateMs = 0;
    uint16_t batteryVoltageMv = 0;
    int8_t   socPercent = -1;
    bool     pvPresent  = false;
    bool     loadOn     = false;
    bool     charging   = false;
    volatile bool valid = false;
    
};

class RenogyChargeController {
public:
    RenogyChargeController(Stream &serial, uint8_t slaveId);

    // Called periodically (from SerialModule) to refresh status
    bool poll();

    // Control load output
    bool setLoad(bool on);

    // Accessors for other subsystems (Power.cpp, command replies)
    bool hasValidStatus() const { return m_status.valid; }
    RenogyStatus getStatusSnapshot() const { return m_status; }

private:
    ModbusRtuMaster m_modbus;
    RenogyStatus    m_status;
};
