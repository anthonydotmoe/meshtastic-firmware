#include "VEDirectCommands.h"

#if (defined(ARCH_ESP32) || defined(ARCH_NRF52) || defined(ARCH_RP2040) || defined(ARCH_STM32WL)) &&                             \
    !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3)

#include "DebugConfiguration.h"
#include "NodeDB.h"
#include "SerialModule.h"
#include "VEDirectChargeController.h"

#include <ctype.h>
#include <string.h>
#include <strings.h>

namespace
{
constexpr uint16_t VEDIRECT_REG_LOAD_CONTROL = 0xEDAB;
constexpr uint16_t VEDIRECT_REG_CHARGER_ERROR = 0xEDDA;
constexpr uint16_t VEDIRECT_REG_TOTAL_HISTORY = 0x104F;

struct VEDirectCommand {
    enum class Type : uint8_t {
        Load,
        Errors,
    } type;

    uint8_t loadControl = 0;
    bool forceOn = false;
    NodeNum requestor = 0;
};

volatile bool s_vedirectCmdPending = false;
VEDirectCommand s_vedirectCmd;

bool hasExtraToken(char *saveptr)
{
    return strtok_r(nullptr, " \t\r\n", &saveptr) != nullptr;
}

void lowercase(char *s)
{
    while (s && *s) {
        *s = static_cast<char>(tolower(*s));
        ++s;
    }
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
        serialModuleRadio->sendText(to, "VE.Direct: help, load off|auto|force-on, errors");
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
    }
}

#endif
