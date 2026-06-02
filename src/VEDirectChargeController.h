#pragma once

#include "Arduino.h"

struct VEDirectStatus {
    uint32_t lastUpdateMs = 0;
    uint32_t batteryVoltage_mV = 0;
    int32_t batteryCurrent_mA = 0;
    uint32_t loadVoltage_mV = 0;
    int32_t loadCurrent_mA = 0;
    uint32_t panelVoltage_mV = 0;
    int32_t panelCurrent_mA = 0;
    int32_t panelPower_W = 0;
    int8_t socPercent = -1;
    uint16_t chargeState = 0;
    bool pvPresent = false;
    bool loadOn = false;
    bool charging = false;
    bool hasBatteryVoltage = false;
    bool hasBatteryCurrent = false;
    bool hasLoadVoltage = false;
    bool hasLoadCurrent = false;
    bool hasPanelVoltage = false;
    bool hasPanelCurrent = false;
    bool hasPanelPower = false;
    bool hasSoc = false;
    bool hasLoadState = false;
    bool hasChargeState = false;
    volatile bool valid = false;
};

class VEDirectChargeController {
  public:
    enum class HexError : uint8_t {
        OK,
        TIMEOUT,
        CHECKSUM,
        BAD_RESPONSE,
        DEVICE_ERROR,
        UNKNOWN_ID,
        NOT_SUPPORTED,
        PARAMETER_ERROR,
        BUFFER_TOO_SMALL,
    };

    explicit VEDirectChargeController(Stream &serial);

    bool poll();
    HexError getRegister(uint16_t id, uint8_t *value, size_t maxLen, size_t &valueLen, uint32_t timeoutMs = 1500,
                         uint8_t *replyFlags = nullptr);
    HexError setRegisterU8(uint16_t id, uint8_t value, uint32_t timeoutMs = 1500, uint8_t *replyFlags = nullptr);
    static const char *hexErrorName(HexError err);

    bool hasValidStatus() const { return m_status.valid; }
    VEDirectStatus getStatusSnapshot() const { return m_status; }

  private:
    enum class ParseState : uint8_t {
        Idle,
        RecordBegin,
        RecordName,
        RecordValue,
        Checksum,
        HexRecord,
    };

    void processByte(uint8_t inbyte);
    void resetFrame();
    void handleRecord(const char *name, const char *value);
    void finishFrame(bool valid);
    bool sendHexFrame(uint8_t command, const uint8_t *payload, size_t payloadLen);
    HexError readHexResponse(uint8_t expectedResponse, uint16_t expectedRegister, uint8_t *value, size_t maxLen,
                             size_t &valueLen, uint32_t timeoutMs, uint8_t *replyFlags);
    static bool parseInt32(const char *value, int32_t &out);
    static bool isChargingState(uint16_t state);
    static int8_t hexNibble(uint8_t c);
    static bool parseHexLine(const char *line, size_t lineLen, uint8_t &response, uint8_t *payload, size_t maxPayloadLen,
                             size_t &payloadLen);

    Stream &m_serial;
    VEDirectStatus m_status;
    VEDirectStatus m_frame;
    ParseState m_state = ParseState::Idle;
    uint8_t m_checksum = 0;
    char m_name[16] = {0};
    char m_value[24] = {0};
    uint8_t m_textIndex = 0;
    bool m_nameOverflow = false;
    bool m_valueOverflow = false;
    bool m_updated = false;
    uint32_t m_lastRxLogMs = 0;
    uint32_t m_lastNoDataLogMs = 0;
    uint32_t m_lastByteMs = 0;
    uint32_t m_bytesSinceLastLog = 0;
    uint32_t m_validFrames = 0;
    uint32_t m_checksumFailures = 0;
};
