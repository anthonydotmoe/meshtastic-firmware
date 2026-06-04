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

VEDirectChargeController::HexError VEDirectChargeController::getProductId(uint16_t &productId, uint32_t timeoutMs)
{
    uint8_t payload[4] = {0};
    size_t payloadLen = 0;

    while (m_serial.available()) {
        processByte(static_cast<uint8_t>(m_serial.read()));
    }

    if (!sendHexFrame(0x4, nullptr, 0)) {
        return HexError::BAD_RESPONSE;
    }

    HexError err = readHexCommandResponse(0x1, payload, sizeof(payload), payloadLen, timeoutMs);
    if (err != HexError::OK) {
        return err;
    }
    if (payloadLen < 2) {
        return HexError::BAD_RESPONSE;
    }

    productId = static_cast<uint16_t>(payload[0]) | (static_cast<uint16_t>(payload[1]) << 8);
    return HexError::OK;
}

VEDirectChargeController::HexError VEDirectChargeController::getAppVersion(uint16_t &version, uint32_t timeoutMs)
{
    uint8_t payload[2] = {0};
    size_t payloadLen = 0;

    while (m_serial.available()) {
        processByte(static_cast<uint8_t>(m_serial.read()));
    }

    if (!sendHexFrame(0x3, nullptr, 0)) {
        return HexError::BAD_RESPONSE;
    }

    HexError err = readHexCommandResponse(0x1, payload, sizeof(payload), payloadLen, timeoutMs);
    if (err != HexError::OK) {
        return err;
    }
    if (payloadLen < 2) {
        return HexError::BAD_RESPONSE;
    }

    version = static_cast<uint16_t>(payload[0]) | (static_cast<uint16_t>(payload[1]) << 8);
    return HexError::OK;
}

VEDirectChargeController::HexError VEDirectChargeController::getRegister(uint16_t id, uint8_t *value, size_t maxLen,
                                                                          size_t &valueLen, uint32_t timeoutMs,
                                                                          uint8_t *replyFlags)
{
    valueLen = 0;
    while (m_serial.available()) {
        processByte(static_cast<uint8_t>(m_serial.read()));
    }

    const uint8_t payload[] = {
        static_cast<uint8_t>(id & 0xFF),
        static_cast<uint8_t>((id >> 8) & 0xFF),
        0x00,
    };

    if (!sendHexFrame(0x7, payload, sizeof(payload))) {
        return HexError::BAD_RESPONSE;
    }

    return readHexResponse(0x7, id, value, maxLen, valueLen, timeoutMs, replyFlags);
}

VEDirectChargeController::HexError VEDirectChargeController::setRegisterU8(uint16_t id, uint8_t value, uint32_t timeoutMs,
                                                                            uint8_t *replyFlags)
{
    size_t valueLen = 0;
    uint8_t echoedValue = 0;

    while (m_serial.available()) {
        processByte(static_cast<uint8_t>(m_serial.read()));
    }

    const uint8_t payload[] = {
        static_cast<uint8_t>(id & 0xFF),
        static_cast<uint8_t>((id >> 8) & 0xFF),
        0x00,
        value,
    };

    if (!sendHexFrame(0x8, payload, sizeof(payload))) {
        return HexError::BAD_RESPONSE;
    }

    HexError err = readHexResponse(0x8, id, &echoedValue, sizeof(echoedValue), valueLen, timeoutMs, replyFlags);
    if (err != HexError::OK) {
        return err;
    }

    return (valueLen == 1 && echoedValue == value) ? HexError::OK : HexError::BAD_RESPONSE;
}

const char *VEDirectChargeController::hexErrorName(HexError err)
{
    switch (err) {
    case HexError::OK:
        return "OK";
    case HexError::TIMEOUT:
        return "TIMEOUT";
    case HexError::CHECKSUM:
        return "CHECKSUM";
    case HexError::BAD_RESPONSE:
        return "BAD_RESPONSE";
    case HexError::DEVICE_ERROR:
        return "DEVICE_ERROR";
    case HexError::UNKNOWN_ID:
        return "UNKNOWN_ID";
    case HexError::NOT_SUPPORTED:
        return "NOT_SUPPORTED";
    case HexError::PARAMETER_ERROR:
        return "PARAMETER_ERROR";
    case HexError::BUFFER_TOO_SMALL:
        return "BUFFER_TOO_SMALL";
    }

    return "UNKNOWN";
}

bool VEDirectChargeController::sendHexFrame(uint8_t command, const uint8_t *payload, size_t payloadLen)
{
    static constexpr char HEX_DIGITS[] = "0123456789ABCDEF";

    if (command > 0x0F) {
        return false;
    }

    uint8_t checksum = command;
    m_serial.write(':');
    m_serial.write(HEX_DIGITS[command]);

    for (size_t i = 0; i < payloadLen; ++i) {
        checksum = static_cast<uint8_t>(checksum + payload[i]);
        m_serial.write(HEX_DIGITS[(payload[i] >> 4) & 0x0F]);
        m_serial.write(HEX_DIGITS[payload[i] & 0x0F]);
    }

    checksum = static_cast<uint8_t>(0x55 - checksum);
    m_serial.write(HEX_DIGITS[(checksum >> 4) & 0x0F]);
    m_serial.write(HEX_DIGITS[checksum & 0x0F]);
    m_serial.write('\n');
    return true;
}

VEDirectChargeController::HexError VEDirectChargeController::readHexCommandResponse(uint8_t expectedResponse, uint8_t *payload,
                                                                                    size_t maxPayloadLen, size_t &payloadLen,
                                                                                    uint32_t timeoutMs)
{
    char line[128] = {0};
    size_t pos = 0;
    bool inHexLine = false;
    const uint32_t start = millis();

    payloadLen = 0;

    while (millis() - start < timeoutMs) {
        while (m_serial.available()) {
            uint8_t c = static_cast<uint8_t>(m_serial.read());

            if (!inHexLine) {
                if (c == ':') {
                    inHexLine = true;
                    pos = 0;
                } else {
                    processByte(c);
                }
                continue;
            }

            if (c == '\n') {
                uint8_t response = 0;
                inHexLine = false;

                if (!parseHexLine(line, pos, response, payload, maxPayloadLen, payloadLen)) {
                    return HexError::CHECKSUM;
                }
                if (response == 0x4) {
                    return HexError::DEVICE_ERROR;
                }
                if (response == expectedResponse) {
                    return HexError::OK;
                }
                continue;
            }

            if (c == '\r') {
                continue;
            }

            if (pos < sizeof(line) - 1) {
                line[pos++] = static_cast<char>(c);
            } else {
                inHexLine = false;
                pos = 0;
            }
        }

        delay(5);
    }

    return HexError::TIMEOUT;
}

VEDirectChargeController::HexError VEDirectChargeController::readHexResponse(uint8_t expectedResponse, uint16_t expectedRegister,
                                                                              uint8_t *value, size_t maxLen, size_t &valueLen,
                                                                              uint32_t timeoutMs, uint8_t *replyFlags)
{
    char line[128] = {0};
    uint8_t payload[80] = {0};
    size_t pos = 0;
    bool inHexLine = false;
    const uint32_t start = millis();

    valueLen = 0;
    if (replyFlags) {
        *replyFlags = 0;
    }

    while (millis() - start < timeoutMs) {
        while (m_serial.available()) {
            uint8_t c = static_cast<uint8_t>(m_serial.read());

            if (!inHexLine) {
                if (c == ':') {
                    inHexLine = true;
                    pos = 0;
                } else {
                    processByte(c);
                }
                continue;
            }

            if (c == '\n') {
                uint8_t response = 0;
                size_t payloadLen = 0;
                inHexLine = false;

                if (!parseHexLine(line, pos, response, payload, sizeof(payload), payloadLen)) {
                    return HexError::CHECKSUM;
                }

                if (response == 0x4) {
                    return HexError::DEVICE_ERROR;
                }

                if (response != expectedResponse) {
                    continue;
                }

                if (payloadLen < 3) {
                    return HexError::BAD_RESPONSE;
                }

                const uint16_t responseRegister = static_cast<uint16_t>(payload[0]) |
                                                  (static_cast<uint16_t>(payload[1]) << 8);
                if (responseRegister != expectedRegister) {
                    continue;
                }

                const uint8_t flags = payload[2];
                if (replyFlags) {
                    *replyFlags = flags;
                }
                if (flags & 0x01) {
                    return HexError::UNKNOWN_ID;
                }
                if (flags & 0x02) {
                    return HexError::NOT_SUPPORTED;
                }
                if (flags & 0x04) {
                    return HexError::PARAMETER_ERROR;
                }
                if (flags != 0) {
                    return HexError::BAD_RESPONSE;
                }

                valueLen = payloadLen - 3;
                if (valueLen > maxLen) {
                    valueLen = 0;
                    return HexError::BUFFER_TOO_SMALL;
                }

                if (valueLen > 0 && value) {
                    memcpy(value, payload + 3, valueLen);
                }
                return HexError::OK;
            }

            if (c == '\r') {
                continue;
            }

            if (pos < sizeof(line) - 1) {
                line[pos++] = static_cast<char>(c);
            } else {
                inHexLine = false;
                pos = 0;
            }
        }

        delay(5);
    }

    return HexError::TIMEOUT;
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

int8_t VEDirectChargeController::hexNibble(uint8_t c)
{
    if (c >= '0' && c <= '9') {
        return static_cast<int8_t>(c - '0');
    }
    c = static_cast<uint8_t>(toupper(c));
    if (c >= 'A' && c <= 'F') {
        return static_cast<int8_t>(c - 'A' + 10);
    }

    return -1;
}

bool VEDirectChargeController::parseHexLine(const char *line, size_t lineLen, uint8_t &response, uint8_t *payload,
                                            size_t maxPayloadLen, size_t &payloadLen)
{
    payloadLen = 0;
    if (!line || lineLen < 3 || ((lineLen - 1) % 2) != 0) {
        return false;
    }

    int8_t cmd = hexNibble(static_cast<uint8_t>(line[0]));
    if (cmd < 0) {
        return false;
    }

    response = static_cast<uint8_t>(cmd);
    uint8_t sum = response;
    const size_t byteCount = (lineLen - 1) / 2;
    if (byteCount == 0 || byteCount - 1 > maxPayloadLen) {
        return false;
    }

    for (size_t i = 0; i < byteCount; ++i) {
        int8_t high = hexNibble(static_cast<uint8_t>(line[1 + (i * 2)]));
        int8_t low = hexNibble(static_cast<uint8_t>(line[2 + (i * 2)]));
        if (high < 0 || low < 0) {
            return false;
        }

        const uint8_t value = static_cast<uint8_t>((high << 4) | low);
        sum = static_cast<uint8_t>(sum + value);

        if (i < byteCount - 1) {
            payload[payloadLen++] = value;
        }
    }

    return sum == 0x55;
}
