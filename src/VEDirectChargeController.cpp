#include "VEDirectChargeController.h"
#include "DebugConfiguration.h"

#include <ctype.h>
#include <string.h>
#include <strings.h>

VEDirectChargeController::VEDirectChargeController(Stream &serial) : m_serial(serial) {}

bool VEDirectChargeController::poll()
{
    m_updated = false;
    bool received = false;
    while (m_serial.available()) {
        received = true;
        m_bytesSinceLastLog++;
        processByte(static_cast<uint8_t>(m_serial.read()));
    }

    const uint32_t now = millis();
    if (received) {
        m_lastByteMs = now;
    }

    if (received && (!m_lastRxLogMs || now - m_lastRxLogMs >= 5000)) {
        LOG_INFO("VE.Direct rx: %lu bytes in last window, validFrames=%lu checksumFailures=%lu",
                 static_cast<unsigned long>(m_bytesSinceLastLog), static_cast<unsigned long>(m_validFrames),
                 static_cast<unsigned long>(m_checksumFailures));
        m_lastRxLogMs = now;
        m_bytesSinceLastLog = 0;
    } else if (!received && (!m_lastByteMs || now - m_lastByteMs >= 30000) &&
               (!m_lastNoDataLogMs || now - m_lastNoDataLogMs >= 30000)) {
        LOG_WARN("VE.Direct rx: no bytes received for %lus",
                 m_lastByteMs ? static_cast<unsigned long>((now - m_lastByteMs) / 1000) : 0UL);
        m_lastNoDataLogMs = now;
    }

    return m_updated;
}

void VEDirectChargeController::processByte(uint8_t inbyte)
{
    if ((inbyte == ':') && (m_state != ParseState::Checksum)) {
        m_state = ParseState::HexRecord;
    }

    if (m_state != ParseState::HexRecord) {
        m_checksum = static_cast<uint8_t>(m_checksum + inbyte);
    }

    const uint8_t upper = static_cast<uint8_t>(toupper(inbyte));

    switch (m_state) {
    case ParseState::Idle:
        if (upper == '\n') {
            resetFrame();
            m_state = ParseState::RecordBegin;
        }
        break;

    case ParseState::RecordBegin:
        if (upper == '\r' || upper == '\n') {
            break;
        }
        m_textIndex = 0;
        m_nameOverflow = false;
        m_name[0] = '\0';
        if (m_textIndex < sizeof(m_name) - 1) {
            m_name[m_textIndex++] = static_cast<char>(upper);
        } else {
            m_nameOverflow = true;
        }
        m_state = ParseState::RecordName;
        break;

    case ParseState::RecordName:
        if (upper == '\t') {
            if (m_textIndex < sizeof(m_name)) {
                m_name[m_textIndex] = '\0';
            }
            if (!m_nameOverflow && strcmp(m_name, "CHECKSUM") == 0) {
                m_state = ParseState::Checksum;
                break;
            }
            m_textIndex = 0;
            m_valueOverflow = false;
            m_value[0] = '\0';
            m_state = ParseState::RecordValue;
        } else if (m_textIndex < sizeof(m_name) - 1) {
            m_name[m_textIndex++] = static_cast<char>(upper);
        } else {
            m_nameOverflow = true;
        }
        break;

    case ParseState::RecordValue:
        if (upper == '\n') {
            if (m_textIndex < sizeof(m_value)) {
                m_value[m_textIndex] = '\0';
            }
            if (!m_nameOverflow && !m_valueOverflow) {
                handleRecord(m_name, m_value);
            }
            m_state = ParseState::RecordBegin;
        } else if (upper != '\r') {
            if (m_textIndex < sizeof(m_value) - 1) {
                m_value[m_textIndex++] = static_cast<char>(inbyte);
            } else {
                m_valueOverflow = true;
            }
        }
        break;

    case ParseState::Checksum:
        finishFrame(m_checksum == 0);
        m_checksum = 0;
        m_state = ParseState::Idle;
        break;

    case ParseState::HexRecord:
        if (upper == '\n') {
            m_checksum = 0;
            m_state = ParseState::Idle;
        }
        break;
    }
}

void VEDirectChargeController::resetFrame()
{
    m_frame = VEDirectStatus{};
    m_name[0] = '\0';
    m_value[0] = '\0';
    m_textIndex = 0;
    m_nameOverflow = false;
    m_valueOverflow = false;
}

void VEDirectChargeController::handleRecord(const char *name, const char *value)
{
    int32_t parsed = 0;

    if (strcmp(name, "V") == 0) {
        if (parseInt32(value, parsed) && parsed >= 0) {
            m_frame.batteryVoltage_mV = static_cast<uint32_t>(parsed);
            m_frame.hasBatteryVoltage = true;
        }
    } else if (strcmp(name, "I") == 0) {
        if (parseInt32(value, parsed)) {
            m_frame.batteryCurrent_mA = parsed;
            m_frame.hasBatteryCurrent = true;
        }
    } else if (strcmp(name, "VPV") == 0) {
        if (parseInt32(value, parsed) && parsed >= 0) {
            m_frame.panelVoltage_mV = static_cast<uint32_t>(parsed);
            m_frame.hasPanelVoltage = true;
        }
    } else if (strcmp(name, "PPV") == 0) {
        if (parseInt32(value, parsed)) {
            m_frame.panelPower_W = parsed;
            m_frame.hasPanelPower = true;
        }
    } else if (strcmp(name, "IL") == 0) {
        if (parseInt32(value, parsed)) {
            m_frame.loadCurrent_mA = parsed;
            m_frame.hasLoadCurrent = true;
        }
    } else if (strcmp(name, "LOAD") == 0) {
        m_frame.loadOn = strcasecmp(value, "ON") == 0;
        m_frame.hasLoadState = true;
    } else if (strcmp(name, "SOC") == 0) {
        if (parseInt32(value, parsed)) {
            m_frame.socPercent = static_cast<int8_t>(parsed / 10);
            m_frame.hasSoc = true;
        }
    } else if (strcmp(name, "CS") == 0) {
        if (parseInt32(value, parsed) && parsed >= 0) {
            m_frame.chargeState = static_cast<uint16_t>(parsed);
            m_frame.charging = isChargingState(m_frame.chargeState);
            m_frame.hasChargeState = true;
        }
    }
}

void VEDirectChargeController::finishFrame(bool valid)
{
    if (!valid) {
        m_checksumFailures++;
        LOG_WARN("VE.Direct checksum failed: failures=%lu", static_cast<unsigned long>(m_checksumFailures));
        resetFrame();
        return;
    }

    if (m_frame.hasBatteryVoltage) {
        m_status.batteryVoltage_mV = m_frame.batteryVoltage_mV;
        m_status.hasBatteryVoltage = true;
    }
    if (m_frame.hasBatteryCurrent) {
        m_status.batteryCurrent_mA = m_frame.batteryCurrent_mA;
        m_status.hasBatteryCurrent = true;
    }
    if (m_frame.hasLoadVoltage) {
        m_status.loadVoltage_mV = m_frame.loadVoltage_mV;
        m_status.hasLoadVoltage = true;
    }
    if (m_frame.hasLoadCurrent) {
        m_status.loadCurrent_mA = m_frame.loadCurrent_mA;
        m_status.hasLoadCurrent = true;
    }
    if ((m_frame.hasLoadCurrent || m_frame.hasBatteryVoltage) && m_status.hasLoadCurrent && m_status.hasBatteryVoltage) {
        m_status.loadVoltage_mV = m_status.batteryVoltage_mV;
        m_status.hasLoadVoltage = true;
    }
    if (m_frame.hasPanelVoltage) {
        m_status.panelVoltage_mV = m_frame.panelVoltage_mV;
        m_status.hasPanelVoltage = true;
    }
    if (m_frame.hasPanelPower) {
        m_status.panelPower_W = m_frame.panelPower_W;
        m_status.hasPanelPower = true;
    }
    if ((m_frame.hasPanelPower || m_frame.hasPanelVoltage) && m_status.hasPanelVoltage && m_status.panelVoltage_mV > 0) {
        m_status.panelCurrent_mA = static_cast<int32_t>((static_cast<int64_t>(m_status.panelPower_W) * 1000000LL) /
                                                       m_status.panelVoltage_mV);
        m_status.hasPanelCurrent = true;
    }
    if (m_frame.hasPanelCurrent) {
        m_status.panelCurrent_mA = m_frame.panelCurrent_mA;
        m_status.hasPanelCurrent = true;
    }
    if (m_frame.hasSoc) {
        m_status.socPercent = m_frame.socPercent;
        m_status.hasSoc = true;
    }

    if (m_frame.hasLoadState) {
        m_status.loadOn = m_frame.loadOn;
        m_status.hasLoadState = true;
    }
    if (m_frame.hasChargeState) {
        m_status.chargeState = m_frame.chargeState;
        m_status.hasChargeState = true;
    }
    m_status.pvPresent = (m_status.hasPanelVoltage && m_status.panelVoltage_mV > 0) || m_status.panelPower_W > 0;
    if (m_frame.hasChargeState || m_frame.hasBatteryCurrent) {
        m_status.charging = (m_status.hasChargeState && isChargingState(m_status.chargeState)) ||
                            (m_status.hasBatteryCurrent && m_status.batteryCurrent_mA > 0);
    }
    m_status.lastUpdateMs = millis();
    m_status.valid = m_status.hasBatteryVoltage || m_status.hasBatteryCurrent || m_status.hasLoadCurrent ||
                     m_status.hasPanelVoltage || m_status.hasPanelCurrent;
    m_updated = m_status.valid;
    m_validFrames++;

    LOG_INFO("VE.Direct frame OK: frames=%lu batt=%lumV/%ldmA panel=%lumV/%ldmA/%ldW load=%s/%ldmA cs=%u valid=%d",
             static_cast<unsigned long>(m_validFrames), static_cast<unsigned long>(m_status.batteryVoltage_mV),
             static_cast<long>(m_status.batteryCurrent_mA), static_cast<unsigned long>(m_status.panelVoltage_mV),
             static_cast<long>(m_status.panelCurrent_mA), static_cast<long>(m_status.panelPower_W),
             m_status.hasLoadState ? (m_status.loadOn ? "ON" : "OFF") : "?", static_cast<long>(m_status.loadCurrent_mA),
             m_status.chargeState, m_status.valid);

    resetFrame();
}

bool VEDirectChargeController::parseInt32(const char *value, int32_t &out)
{
    if (!value || !*value) {
        return false;
    }

    char *end = nullptr;
    long parsed = strtol(value, &end, 10);
    if (end == value || *end != '\0') {
        return false;
    }

    out = static_cast<int32_t>(parsed);
    return true;
}

bool VEDirectChargeController::isChargingState(uint16_t state)
{
    switch (state) {
    case 3:   // Bulk
    case 4:   // Absorption
    case 5:   // Float
    case 6:   // Storage
    case 7:   // Equalize
    case 245: // Starting-up
    case 246: // Repeated absorption
    case 247: // Auto equalize / Recondition
    case 248: // BatterySafe
    case 252: // External control
        return true;
    default:
        return false;
    }
}
