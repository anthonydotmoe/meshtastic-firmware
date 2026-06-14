#include "VEDirectCommands.h"

#if (defined(ARCH_ESP32) || defined(ARCH_NRF52) || defined(ARCH_RP2040) || defined(ARCH_STM32WL)) &&                             \
    !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3)

#include "DebugConfiguration.h"
#include "NodeDB.h"
#include "SerialModule.h"
#include "VEDirectChargeController.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

namespace
{
constexpr uint16_t VEDIRECT_REG_LOAD_CONTROL = 0xEDAB;
constexpr uint16_t VEDIRECT_REG_CHARGER_ERROR = 0xEDDA;
constexpr uint16_t VEDIRECT_REG_TOTAL_HISTORY = 0x104F;
constexpr uint16_t VEDIRECT_REG_DAY_HISTORY_BASE = 0x1050;
constexpr uint16_t VEDIRECT_REG_DAY_HISTORY_MAX = 0x106E;
constexpr uint16_t VEDIRECT_REG_DEVICE_MODE = 0x0200;
constexpr uint16_t VEDIRECT_REG_DEVICE_STATE = 0x0201;
constexpr uint8_t VEDIRECT_MAX_HISTORY_DAY = VEDIRECT_REG_DAY_HISTORY_MAX - VEDIRECT_REG_DAY_HISTORY_BASE;
constexpr size_t VEDIRECT_DAY_HISTORY_LEN = 34;
constexpr size_t VEDIRECT_HISTORY_REPLY_MAX_BYTES = 200;

struct VEDirectCommand {
    enum class Type : uint8_t {
        Load,
        Errors,
        Status,
        History,
    } type;

    uint8_t loadControl = 0;
    uint8_t historyDay = 0;
    bool forceOn = false;
    NodeNum requestor = 0;
};

volatile bool s_vedirectCmdPending = false;
VEDirectCommand s_vedirectCmd;

struct ProductNameMapping {
    uint16_t productId;
    uint8_t familyId;
    uint8_t typeId;
    uint8_t ratingId;
    uint8_t revId;
};

static const ProductNameMapping PRODUCT_NAME_MAPPINGS[] = {
    {0x0300, 0, 0, 0, 0},   {0xA040, 0, 0, 3, 0},   {0xA041, 0, 0, 9, 0},   {0xA042, 0, 0, 2, 0},
    {0xA043, 0, 0, 4, 0},   {0xA044, 0, 0, 7, 0},   {0xA045, 0, 0, 8, 0},   {0xA046, 0, 0, 12, 0},
    {0xA047, 0, 0, 14, 0},  {0xA048, 0, 0, 3, 1},   {0xA049, 0, 0, 8, 1},   {0xA04A, 0, 0, 7, 1},
    {0xA04B, 0, 0, 9, 1},   {0xA04C, 0, 0, 1, 0},   {0xA04D, 0, 0, 10, 0},  {0xA04E, 0, 0, 11, 0},
    {0xA04F, 0, 0, 13, 0},  {0xA050, 1, 0, 19, 0},  {0xA051, 1, 0, 14, 0},  {0xA052, 1, 0, 13, 0},
    {0xA053, 1, 0, 2, 0},   {0xA054, 1, 0, 1, 0},   {0xA055, 1, 0, 4, 0},   {0xA056, 1, 0, 7, 0},
    {0xA057, 1, 0, 8, 0},   {0xA058, 1, 0, 9, 0},   {0xA059, 1, 0, 14, 1},  {0xA05A, 1, 0, 13, 1},
    {0xA05B, 1, 0, 17, 0},  {0xA05C, 1, 0, 18, 0},  {0xA05D, 1, 0, 16, 0},  {0xA05E, 1, 0, 15, 0},
    {0xA05F, 1, 0, 5, 0},   {0xA060, 1, 0, 6, 0},   {0xA061, 1, 0, 10, 0},  {0xA062, 1, 0, 11, 0},
    {0xA063, 1, 0, 12, 0},  {0xA064, 1, 0, 18, 1},  {0xA065, 1, 0, 19, 1},  {0xA066, 0, 0, 5, 0},
    {0xA067, 0, 0, 6, 0},   {0xA068, 1, 0, 16, 1},  {0xA069, 1, 0, 17, 1},  {0xA06A, 1, 0, 10, 1},
    {0xA06B, 1, 0, 11, 1},  {0xA06C, 1, 0, 12, 1},  {0xA06D, 1, 0, 13, 1},  {0xA06E, 1, 0, 14, 2},
    {0xA06F, 0, 0, 10, 1},  {0xA070, 0, 0, 11, 1},  {0xA071, 0, 0, 12, 1},  {0xA072, 0, 0, 10, 2},
    {0xA073, 1, 0, 10, 2},  {0xA074, 1, 0, 1, 1},   {0xA075, 1, 0, 2, 1},   {0xA076, 0, 0, 7, 2},
    {0xA077, 0, 0, 8, 2},   {0xA078, 0, 0, 9, 1},   {0xA079, 0, 0, 1, 1},   {0xA07A, 0, 0, 2, 1},
    {0xA07B, 0, 0, 4, 1},   {0xA07C, 0, 0, 1, 2},   {0xA07D, 0, 0, 2, 2},   {0xA07E, 2, 0, 7, 0},
    {0xA102, 1, 1, 12, 0},  {0xA103, 1, 1, 10, 0},  {0xA104, 1, 1, 11, 0},  {0xA105, 1, 1, 13, 0},
    {0xA106, 1, 1, 14, 0},  {0xA107, 1, 1, 15, 0},  {0xA108, 1, 1, 16, 0},  {0xA109, 1, 1, 17, 0},
    {0xA10A, 1, 1, 18, 0},  {0xA10B, 1, 1, 19, 0},  {0xA10C, 1, 1, 12, 1},  {0xA10D, 1, 1, 13, 1},
    {0xA10E, 1, 1, 14, 1},  {0xA10F, 0, 1, 14, 0},  {0xA110, 1, 2, 20, 0},  {0xA111, 1, 2, 21, 0},
    {0xA112, 0, 1, 17, 0},  {0xA113, 0, 1, 19, 0},  {0xA114, 1, 1, 17, 1},  {0xA115, 1, 1, 19, 1},
    {0xA116, 1, 1, 18, 1},  {0xA117, 0, 1, 14, 1},
};

bool hasExtraToken(char *saveptr)
{
    return strtok_r(nullptr, " \t\r\n", &saveptr) != nullptr;
}

bool parseHistoryDay(const char *token, uint8_t &out)
{
    out = 0;
    if (!token) {
        return true;
    }

    char *end = nullptr;
    long day = strtol(token, &end, 0);
    if (end == token || *end != '\0' || day < 0) {
        return false;
    }

    if (day > VEDIRECT_MAX_HISTORY_DAY) {
        day = VEDIRECT_MAX_HISTORY_DAY;
    }
    out = static_cast<uint8_t>(day);
    return true;
}

void appendFormat(char *reply, size_t replySize, size_t &pos, const char *format, ...)
{
    if (pos >= replySize) {
        return;
    }

    va_list args;
    va_start(args, format);
    int written = vsnprintf(reply + pos, replySize - pos, format, args);
    va_end(args);

    if (written > 0) {
        pos += static_cast<size_t>(written);
    }
}

void lowercase(char *s)
{
    while (s && *s) {
        *s = static_cast<char>(tolower(*s));
        ++s;
    }
}

const ProductNameMapping *findProductNameMapping(uint16_t productId)
{
    for (const auto &mapping : PRODUCT_NAME_MAPPINGS) {
        if (mapping.productId == productId) {
            return &mapping;
        }
    }

    return nullptr;
}

const char *productFamilyName(uint8_t id)
{
    switch (id) {
    case 0:
        return "BlueSolar";
    case 1:
        return "SmartSolar";
    case 2:
        return "SmartSolar Charger";
    default:
        return "Victron";
    }
}

const char *productTypeName(uint8_t id)
{
    switch (id) {
    case 0:
        return "MPPT";
    case 1:
        return "MPPT VE.Can";
    case 2:
        return "MPPT RS";
    default:
        return "";
    }
}

const char *productRatingName(uint8_t id)
{
    switch (id) {
    case 0:
        return "70|15";
    case 1:
        return "75|10";
    case 2:
        return "75|15";
    case 3:
        return "75|50";
    case 4:
        return "100|15";
    case 5:
        return "100|20";
    case 6:
        return "100|20 48V";
    case 7:
        return "100|30";
    case 8:
        return "100|50";
    case 9:
        return "150|35";
    case 10:
        return "150|45";
    case 11:
        return "150|60";
    case 12:
        return "150|70";
    case 13:
        return "150|85";
    case 14:
        return "150|100";
    case 15:
        return "250|45";
    case 16:
        return "250|60";
    case 17:
        return "250|70";
    case 18:
        return "250|85";
    case 19:
        return "250|100";
    case 20:
        return "450|100";
    case 21:
        return "450|200";
    default:
        return "";
    }
}

const char *productRevName(uint8_t id)
{
    switch (id) {
    case 1:
        return "rev2";
    case 2:
        return "rev3";
    default:
        return "";
    }
}

void formatProductName(uint16_t productId, char *out, size_t outSize)
{
    const ProductNameMapping *mapping = findProductNameMapping(productId);
    if (!mapping) {
        snprintf(out, outSize, "Victron 0x%04X", productId);
        return;
    }

    const char *rev = productRevName(mapping->revId);
    snprintf(out, outSize, "%s %s %s%s%s", productFamilyName(mapping->familyId), productTypeName(mapping->typeId),
             productRatingName(mapping->ratingId), rev[0] ? " " : "", rev);
}

void formatFirmwareVersion(uint16_t version, char *out, size_t outSize)
{
    snprintf(out, outSize, "%u.%02u", static_cast<unsigned>((version >> 8) & 0x3F), static_cast<unsigned>(version & 0xFF));
}

bool readRegisterU8(VEDirectChargeController *controller, uint16_t reg, uint8_t &out)
{
    size_t len = 0;
    VEDirectChargeController::HexError err = controller->getRegister(reg, &out, sizeof(out), len, 1000);
    return err == VEDirectChargeController::HexError::OK && len >= 1;
}

uint16_t readLe16(const uint8_t *p)
{
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t readLe32(const uint8_t *p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

bool parseCommandText(const uint8_t *data, size_t len, VEDirectCommand &out, bool &isHelp)
{
    isHelp = false;

    char buf[64];
    if (len >= sizeof(buf)) {
        len = sizeof(buf) - 1;
    }
    memcpy(buf, data, len);
    buf[len] = '\0';

    char *saveptr = nullptr;
    char *cmd = strtok_r(buf, " \t\r\n", &saveptr);
    if (!cmd) {
        return false;
    }
    lowercase(cmd);

    if (strcmp(cmd, "help") == 0) {
        if (hasExtraToken(saveptr)) {
            return false;
        }
        isHelp = true;
        return true;
    }

    if (strcmp(cmd, "errors") == 0) {
        if (hasExtraToken(saveptr)) {
            return false;
        }
        out.type = VEDirectCommand::Type::Errors;
        return true;
    }

    if (strcmp(cmd, "status") == 0) {
        if (hasExtraToken(saveptr)) {
            return false;
        }
        out.type = VEDirectCommand::Type::Status;
        return true;
    }

    if (strcmp(cmd, "history") == 0) {
        char *day = strtok_r(nullptr, " \t\r\n", &saveptr);
        if (hasExtraToken(saveptr)) {
            return false;
        }

        out.type = VEDirectCommand::Type::History;
        return parseHistoryDay(day, out.historyDay);
    }

    if (strcmp(cmd, "load") == 0) {
        char *mode = strtok_r(nullptr, " \t\r\n", &saveptr);
        if (!mode || hasExtraToken(saveptr)) {
            return false;
        }
        lowercase(mode);

        out.type = VEDirectCommand::Type::Load;
        if (strcmp(mode, "off") == 0) {
            out.loadControl = 0;
            return true;
        }
        if (strcmp(mode, "auto") == 0) {
            out.loadControl = 1;
            return true;
        }
        if (strcmp(mode, "force-on") == 0) {
            out.loadControl = 4;
            out.forceOn = true;
            return true;
        }
    }

    return false;
}

void sendHelp(NodeNum to)
{
    if (serialModuleRadio) {
        serialModuleRadio->sendText(to, "VE.Direct: help, status, history [day], load off|auto|force-on, errors");
    }
}

const char *loadModeName(uint8_t mode)
{
    switch (mode) {
    case 0:
        return "off";
    case 1:
        return "auto";
    case 4:
        return "force-on";
    default:
        return "?";
    }
}

const char *errorMeaning(uint8_t code)
{
    switch (code) {
    case 0:
        return "none";
    case 2:
        return "battery voltage high";
    case 3:
    case 4:
    case 5:
        return "battery temp sensor";
    case 6:
    case 7:
    case 8:
        return "battery voltage sensor";
    case 14:
        return "battery temp low";
    case 17:
        return "charger temp high";
    case 18:
        return "charger overcurrent";
    case 19:
        return "charger current reversed";
    case 20:
        return "bulk time expired";
    case 21:
        return "current sensor";
    case 22:
    case 23:
        return "internal temp sensor";
    case 26:
        return "terminals overheated";
    case 27:
        return "charger short-circuit";
    case 28:
        return "converter issue";
    case 29:
        return "battery overcharge";
    case 33:
        return "input voltage high";
    case 34:
        return "input current high";
    case 38:
        return "input shutdown batt voltage";
    case 39:
        return "input shutdown current";
    case 65:
        return "device comms lost";
    case 66:
        return "network incompatible";
    case 67:
        return "BMS lost";
    case 68:
        return "network misconfigured";
    case 116:
        return "calibration lost";
    case 117:
        return "firmware incompatible";
    case 119:
        return "settings invalid";
    default:
        return "unknown";
    }
}

const char *chargeStateName(uint8_t state)
{
    switch (state) {
    case 0:
        return "NOT_CHARGING";
    case 2:
        return "FAULT";
    case 3:
        return "BULK";
    case 4:
        return "ABSORPTION";
    case 5:
        return "FLOAT";
    case 6:
        return "STORAGE";
    case 7:
        return "EQUALISE";
    case 245:
        return "WAKE-UP";
    case 246:
        return "REPEATED ABS";
    case 247:
        return "AUTO EQUALISE";
    case 248:
        return "BATTERYSAFE";
    case 250:
        return "BLOCKED";
    case 252:
        return "EXT CONTROL";
    case 255:
        return "UNAVAILABLE";
    default:
        return "?";
    }
}

const char *deviceModeName(uint8_t mode)
{
    switch (mode) {
    case 0:
    case 4:
        return "charger off";
    case 1:
        return "charger on";
    default:
        return "?";
    }
}

const char *loadControlName(uint8_t control)
{
    switch (control & 0x0F) {
    case 0:
        return "OFF";
    case 1:
        return "AUTO";
    case 2:
        return "ALT1";
    case 3:
        return "ALT2";
    case 4:
        return "FORCE-ON";
    case 5:
        return "USER1";
    case 6:
        return "USER2";
    case 7:
        return "AES";
    default:
        return "?";
    }
}

void appendVoltage(char *reply, size_t replySize, size_t &pos, bool hasValue, uint32_t mV)
{
    if (!hasValue) {
        appendFormat(reply, replySize, pos, "? V");
        return;
    }

    appendFormat(reply, replySize, pos, "%lu.%02lu V", static_cast<unsigned long>(mV / 1000),
                 static_cast<unsigned long>((mV % 1000) / 10));
}

void appendCurrent(char *reply, size_t replySize, size_t &pos, bool hasValue, int32_t mA, bool includeSign)
{
    if (!hasValue) {
        appendFormat(reply, replySize, pos, "? A");
        return;
    }

    const bool negative = mA < 0;
    uint32_t magnitude = negative ? static_cast<uint32_t>(-mA) : static_cast<uint32_t>(mA);
    if (includeSign) {
        appendFormat(reply, replySize, pos, "%c", negative ? '-' : '+');
    } else if (negative) {
        appendFormat(reply, replySize, pos, "-");
    }
    appendFormat(reply, replySize, pos, "%lu.%02lu A", static_cast<unsigned long>(magnitude / 1000),
                 static_cast<unsigned long>((magnitude % 1000) / 10));
}

void appendPower(char *reply, size_t replySize, size_t &pos, const VEDirectStatus &status)
{
    if (status.hasPanelPower) {
        appendFormat(reply, replySize, pos, "%ld W", static_cast<long>(status.panelPower_W));
        return;
    }

    if (status.hasPanelVoltage && status.hasPanelCurrent) {
        const int32_t powerDeciW =
            static_cast<int32_t>((static_cast<int64_t>(status.panelVoltage_mV) * status.panelCurrent_mA) / 100000000LL);
        appendFormat(reply, replySize, pos, "%ld.%ld W", static_cast<long>(powerDeciW / 10),
                     static_cast<long>(abs(powerDeciW % 10)));
        return;
    }

    appendFormat(reply, replySize, pos, "? W");
}

void appendEnergyKwh(char *reply, size_t replySize, size_t &pos, uint32_t centiKwh)
{
    appendFormat(reply, replySize, pos, "%lu.%02lukWh", static_cast<unsigned long>(centiKwh / 100),
                 static_cast<unsigned long>(centiKwh % 100));
}

void appendSignedEnergyKwh(char *reply, size_t replySize, size_t &pos, int64_t centiKwh)
{
    const bool negative = centiKwh < 0;
    const uint64_t magnitude = negative ? static_cast<uint64_t>(-centiKwh) : static_cast<uint64_t>(centiKwh);
    appendFormat(reply, replySize, pos, "%c%lu.%02lukWh", negative ? '-' : '+',
                 static_cast<unsigned long>(magnitude / 100), static_cast<unsigned long>(magnitude % 100));
}

void appendCentivolts(char *reply, size_t replySize, size_t &pos, uint16_t centivolts)
{
    appendFormat(reply, replySize, pos, "%u.%02uV", static_cast<unsigned>(centivolts / 100),
                 static_cast<unsigned>(centivolts % 100));
}

void appendDeciamps(char *reply, size_t replySize, size_t &pos, uint16_t deciamps)
{
    appendFormat(reply, replySize, pos, "%u.%uA", static_cast<unsigned>(deciamps / 10),
                 static_cast<unsigned>(deciamps % 10));
}

void appendMinutes(char *reply, size_t replySize, size_t &pos, uint16_t minutes)
{
    const uint16_t hours = minutes / 60;
    const uint16_t remainingMinutes = minutes % 60;

    if (hours > 0) {
        appendFormat(reply, replySize, pos, "%uh", static_cast<unsigned>(hours));
    }
    appendFormat(reply, replySize, pos, "%um", static_cast<unsigned>(remainingMinutes));
}

size_t appendError(char *reply, size_t replySize, size_t pos, uint8_t code)
{
    if (pos >= replySize) {
        return pos;
    }

    return pos + snprintf(reply + pos, replySize - pos, "%u %s", code, errorMeaning(code));
}

void appendHistoryErrors(char *reply, size_t replySize, size_t &pos, const uint8_t *history, size_t historyLen)
{
    uint8_t errors[4] = {0};
    bool found = false;

    if (historyLen >= 7 && history[0] == 1) {
        memcpy(errors, history + 3, sizeof(errors));
    } else if (historyLen >= 6) {
        memcpy(errors, history + 2, sizeof(errors));
    } else {
        pos += snprintf(reply + pos, replySize - pos, "; history unavailable");
        return;
    }

    for (uint8_t i = 0; i < sizeof(errors); ++i) {
        if (errors[i] == 0) {
            continue;
        }
        pos += snprintf(reply + pos, replySize - pos, "%s", found ? ", " : "; history ");
        pos = appendError(reply, replySize, pos, errors[i]);
        found = true;
    }

    if (!found) {
        pos += snprintf(reply + pos, replySize - pos, "; history none");
    }
}

void appendDayErrors(char *reply, size_t replySize, size_t &pos, const uint8_t *errors, size_t errorLen)
{
    bool found = false;

    for (size_t i = 0; i < errorLen; ++i) {
        if (errors[i] == 0) {
            continue;
        }
        appendFormat(reply, replySize, pos, "%s", found ? ", " : "");
        pos = appendError(reply, replySize, pos, errors[i]);
        found = true;
    }

    if (!found) {
        appendFormat(reply, replySize, pos, "none");
    }
}

void sendHistoryReply(SerialModuleRadio *radio, NodeNum to, const char *reply)
{
    if (!radio || !reply) {
        return;
    }

    if (strnlen(reply, VEDIRECT_HISTORY_REPLY_MAX_BYTES + 1) <= VEDIRECT_HISTORY_REPLY_MAX_BYTES) {
        radio->sendText(to, reply);
        return;
    }

    const char *lineStart = reply;
    while (*lineStart) {
        const char *lineEnd = strchr(lineStart, '\n');
        const size_t lineLen = lineEnd ? static_cast<size_t>(lineEnd - lineStart) : strlen(lineStart);
        char line[VEDIRECT_HISTORY_REPLY_MAX_BYTES + 1] = {0};
        const size_t copyLen = lineLen > VEDIRECT_HISTORY_REPLY_MAX_BYTES ? VEDIRECT_HISTORY_REPLY_MAX_BYTES : lineLen;
        memcpy(line, lineStart, copyLen);
        radio->sendText(to, line);

        if (!lineEnd) {
            break;
        }
        lineStart = lineEnd + 1;
    }
}

void replyHexError(SerialModuleRadio *radio, NodeNum to, const char *prefix, VEDirectChargeController::HexError err)
{
    if (!radio) {
        return;
    }

    char reply[96] = {0};
    snprintf(reply, sizeof(reply), "ERR %s %s", prefix, VEDirectChargeController::hexErrorName(err));
    radio->sendText(to, reply);
}

void processLoadCommand(const VEDirectCommand &cmd, VEDirectChargeController *controller, SerialModuleRadio *radio)
{
    uint8_t flags = 0;
    VEDirectChargeController::HexError err =
        controller->setRegisterU8(VEDIRECT_REG_LOAD_CONTROL, cmd.loadControl, 1500, &flags);
    if (err != VEDirectChargeController::HexError::OK) {
        replyHexError(radio, cmd.requestor, "load", err);
        return;
    }

    char reply[96] = {0};
    if (cmd.forceOn) {
        LOG_WARN("VE.Direct load force-on requested by favorite node 0x%08x; no discharge guard", cmd.requestor);
        snprintf(reply, sizeof(reply), "OK load force-on; WARNING no discharge guard");
    } else {
        snprintf(reply, sizeof(reply), "OK load %s", loadModeName(cmd.loadControl));
    }

    if (radio) {
        radio->sendText(cmd.requestor, reply);
    }
}

void processErrorsCommand(const VEDirectCommand &cmd, VEDirectChargeController *controller, SerialModuleRadio *radio)
{
    uint8_t current = 0;
    size_t currentLen = 0;
    VEDirectChargeController::HexError err =
        controller->getRegister(VEDIRECT_REG_CHARGER_ERROR, &current, sizeof(current), currentLen);
    if (err != VEDirectChargeController::HexError::OK || currentLen < 1) {
        replyHexError(radio, cmd.requestor, "errors", err);
        return;
    }

    uint8_t history[40] = {0};
    size_t historyLen = 0;
    VEDirectChargeController::HexError historyErr =
        controller->getRegister(VEDIRECT_REG_TOTAL_HISTORY, history, sizeof(history), historyLen);

    char reply[192] = {0};
    size_t pos = snprintf(reply, sizeof(reply), "Errors: current ");
    pos = appendError(reply, sizeof(reply), pos, current);

    if (historyErr == VEDirectChargeController::HexError::OK) {
        appendHistoryErrors(reply, sizeof(reply), pos, history, historyLen);
    } else {
        pos += snprintf(reply + pos, sizeof(reply) - pos, "; history %s",
                        VEDirectChargeController::hexErrorName(historyErr));
    }

    if (radio) {
        radio->sendText(cmd.requestor, reply);
    }
}

void processHistoryCommand(const VEDirectCommand &cmd, VEDirectChargeController *controller, SerialModuleRadio *radio)
{
    const uint16_t reg = VEDIRECT_REG_DAY_HISTORY_BASE + cmd.historyDay;
    uint8_t history[VEDIRECT_DAY_HISTORY_LEN] = {0};
    size_t historyLen = 0;
    uint8_t flags = 0;
    VEDirectChargeController::HexError err =
        controller->getRegister(reg, history, sizeof(history), historyLen, 1500, &flags);

    if (err != VEDirectChargeController::HexError::OK) {
        if (err == VEDirectChargeController::HexError::PARAMETER_ERROR && flags == 0x04 && radio) {
            char reply[80] = {0};
            snprintf(reply, sizeof(reply), "History day %u: empty", static_cast<unsigned>(cmd.historyDay));
            radio->sendText(cmd.requestor, reply);
        } else {
            replyHexError(radio, cmd.requestor, "history", err);
        }
        return;
    }

    if (historyLen < VEDIRECT_DAY_HISTORY_LEN) {
        replyHexError(radio, cmd.requestor, "history", VEDirectChargeController::HexError::BAD_RESPONSE);
        return;
    }

    const uint32_t yieldCentiKwh = readLe32(history + 1);
    const uint32_t consumedCentiKwh = readLe32(history + 5);
    const uint16_t batteryMaxCentivolts = readLe16(history + 9);
    const uint16_t batteryMinCentivolts = readLe16(history + 11);
    const uint16_t timeBulkMin = readLe16(history + 18);
    const uint16_t timeAbsorptionMin = readLe16(history + 20);
    const uint16_t timeFloatMin = readLe16(history + 22);
    const uint32_t powerMaxW = readLe32(history + 24);
    const uint16_t batteryCurrentMaxDeciamps = readLe16(history + 28);
    const uint16_t panelMaxCentivolts = readLe16(history + 30);

    char reply[384] = {0};
    size_t pos = 0;

    appendFormat(reply, sizeof(reply), pos, "Yield: ");
    appendEnergyKwh(reply, sizeof(reply), pos, yieldCentiKwh);
    appendFormat(reply, sizeof(reply), pos, "\nLoad: ");
    if (consumedCentiKwh == 0xFFFFFFFF) {
        appendFormat(reply, sizeof(reply), pos, "n/a");
    } else {
        appendEnergyKwh(reply, sizeof(reply), pos, consumedCentiKwh);
    }
    appendFormat(reply, sizeof(reply), pos, "\nNet: ");
    if (consumedCentiKwh == 0xFFFFFFFF) {
        appendFormat(reply, sizeof(reply), pos, "n/a");
    } else {
        appendSignedEnergyKwh(reply, sizeof(reply), pos,
                              static_cast<int64_t>(yieldCentiKwh) - static_cast<int64_t>(consumedCentiKwh));
    }
    appendFormat(reply, sizeof(reply), pos, "\nBatt: ");
    appendCentivolts(reply, sizeof(reply), pos, batteryMinCentivolts);
    appendFormat(reply, sizeof(reply), pos, "-");
    appendCentivolts(reply, sizeof(reply), pos, batteryMaxCentivolts);
    appendFormat(reply, sizeof(reply), pos, ", Imax ");
    appendDeciamps(reply, sizeof(reply), pos, batteryCurrentMaxDeciamps);
    appendFormat(reply, sizeof(reply), pos, "\nPV: Pmax %luW, Vmax ", static_cast<unsigned long>(powerMaxW));
    appendCentivolts(reply, sizeof(reply), pos, panelMaxCentivolts);
    appendFormat(reply, sizeof(reply), pos, "\nCharge: bulk ");
    appendMinutes(reply, sizeof(reply), pos, timeBulkMin);
    appendFormat(reply, sizeof(reply), pos, ", abs ");
    appendMinutes(reply, sizeof(reply), pos, timeAbsorptionMin);
    appendFormat(reply, sizeof(reply), pos, ", float ");
    appendMinutes(reply, sizeof(reply), pos, timeFloatMin);
    appendFormat(reply, sizeof(reply), pos, "\nErrors: ");
    appendDayErrors(reply, sizeof(reply), pos, history + 14, 4);
    sendHistoryReply(radio, cmd.requestor, reply);
}

void processStatusCommand(const VEDirectCommand &cmd, VEDirectChargeController *controller, SerialModuleRadio *radio)
{
    controller->poll();

    VEDirectStatus status = controller->getStatusSnapshot();
    uint16_t productId = 0;
    uint16_t fwVersion = 0;
    uint8_t deviceMode = 0;
    uint8_t deviceState = 0;
    uint8_t loadControl = 0;
    uint8_t currentError = 0;

    bool hasProductId =
        controller->getProductId(productId) == VEDirectChargeController::HexError::OK;
    bool hasFwVersion =
        controller->getAppVersion(fwVersion) == VEDirectChargeController::HexError::OK;
    bool hasDeviceMode = readRegisterU8(controller, VEDIRECT_REG_DEVICE_MODE, deviceMode);
    bool hasDeviceState = readRegisterU8(controller, VEDIRECT_REG_DEVICE_STATE, deviceState);
    bool hasLoadControl = readRegisterU8(controller, VEDIRECT_REG_LOAD_CONTROL, loadControl);
    bool hasCurrentError = readRegisterU8(controller, VEDIRECT_REG_CHARGER_ERROR, currentError);

    char productName[48] = "VE.Direct";
    char fwText[8] = "?";
    if (hasProductId) {
        formatProductName(productId, productName, sizeof(productName));
    }
    if (hasFwVersion) {
        formatFirmwareVersion(fwVersion, fwText, sizeof(fwText));
    }

    char reply[240] = {0};
    size_t pos = 0;

    appendFormat(reply, sizeof(reply), pos, "%s FW:%s\n", productName, fwText);

    appendFormat(reply, sizeof(reply), pos, "Battery: ");
    appendVoltage(reply, sizeof(reply), pos, status.hasBatteryVoltage, status.batteryVoltage_mV);
    appendFormat(reply, sizeof(reply), pos, ", %s ", status.hasBatteryCurrent ? (status.batteryCurrent_mA < 0 ? "discharge" : "charge") : "current");
    appendCurrent(reply, sizeof(reply), pos, status.hasBatteryCurrent, status.batteryCurrent_mA, status.hasBatteryCurrent);
    appendFormat(reply, sizeof(reply), pos, "\n");

    appendFormat(reply, sizeof(reply), pos, "PV: ");
    appendVoltage(reply, sizeof(reply), pos, status.hasPanelVoltage, status.panelVoltage_mV);
    appendFormat(reply, sizeof(reply), pos, ", ");
    appendPower(reply, sizeof(reply), pos, status);
    appendFormat(reply, sizeof(reply), pos, "\n");

    appendFormat(reply, sizeof(reply), pos, "Load: %s, ", status.hasLoadState ? (status.loadOn ? "ON" : "OFF") : "?");
    appendVoltage(reply, sizeof(reply), pos, status.hasLoadVoltage, status.loadVoltage_mV);
    appendFormat(reply, sizeof(reply), pos, ", ");
    appendCurrent(reply, sizeof(reply), pos, status.hasLoadCurrent, status.loadCurrent_mA, false);
    appendFormat(reply, sizeof(reply), pos, "\n");

    appendFormat(reply, sizeof(reply), pos, "Mode: %s, state %s\n", hasDeviceMode ? deviceModeName(deviceMode) : "?",
                 hasDeviceState ? chargeStateName(deviceState)
                                : (status.hasChargeState ? chargeStateName(static_cast<uint8_t>(status.chargeState)) : "?"));

    appendFormat(reply, sizeof(reply), pos, "Load control: %s%s\n", hasLoadControl ? loadControlName(loadControl) : "?",
                 (hasLoadControl && (loadControl & 0x80)) ? " timer" : "");

    appendFormat(reply, sizeof(reply), pos, "Error: ");
    if (!hasCurrentError || currentError == 0) {
        appendFormat(reply, sizeof(reply), pos, "%s", hasCurrentError ? "none" : "?");
    } else {
        pos = appendError(reply, sizeof(reply), pos, currentError);
    }

    if (radio) {
        radio->sendText(cmd.requestor, reply);
    }
}

} // namespace

ProcessMessage handleVEDirectCommandReceived(const meshtastic_MeshPacket &mp)
{
    const auto &p = mp.decoded;

    LOG_DEBUG("Received VE.Direct command candidate self=0x%0x, from=0x%0x, to=0x%0x, id=%d, msg=%.*s",
              nodeDB->getNodeNum(), mp.from, mp.to, mp.id, p.payload.size, p.payload.bytes);

    if (!isToUs(&mp)) {
        return ProcessMessage::CONTINUE;
    }

    NodeNum from = getFrom(&mp);
    if (!nodeDB->isFavorite(from)) {
        return ProcessMessage::CONTINUE;
    }

    VEDirectCommand cmd;
    bool isHelp = false;
    if (!parseCommandText(p.payload.bytes, p.payload.size, cmd, isHelp)) {
        if (serialModuleRadio) {
            serialModuleRadio->sendText(from, "ERR BAD_CMD. Send: help");
        }
        return ProcessMessage::STOP;
    }

    if (isHelp) {
        sendHelp(from);
        return ProcessMessage::STOP;
    }

    cmd.requestor = from;
    if (!s_vedirectCmdPending) {
        s_vedirectCmd = cmd;
        __asm__ __volatile__("" ::: "memory");
        s_vedirectCmdPending = true;
    } else if (serialModuleRadio) {
        serialModuleRadio->sendText(from, "ERR BUSY");
    }

    return ProcessMessage::STOP;
}

void processVEDirectCommandQueue(VEDirectChargeController *controller, SerialModuleRadio *radio)
{
    if (!s_vedirectCmdPending) {
        return;
    }

    VEDirectCommand cmd = s_vedirectCmd;
    s_vedirectCmdPending = false;

    if (!controller) {
        if (radio) {
            radio->sendText(cmd.requestor, "ERR NO_CONTROLLER");
        }
        return;
    }

    switch (cmd.type) {
    case VEDirectCommand::Type::Load:
        processLoadCommand(cmd, controller, radio);
        break;
    case VEDirectCommand::Type::Errors:
        processErrorsCommand(cmd, controller, radio);
        break;
    case VEDirectCommand::Type::Status:
        processStatusCommand(cmd, controller, radio);
        break;
    case VEDirectCommand::Type::History:
        processHistoryCommand(cmd, controller, radio);
        break;
    }
}

#endif
