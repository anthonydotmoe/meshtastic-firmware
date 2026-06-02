#pragma once

#include "MeshModule.h"

class SerialModuleRadio;
class VEDirectChargeController;

ProcessMessage handleVEDirectCommandReceived(const meshtastic_MeshPacket &mp);
void processVEDirectCommandQueue(VEDirectChargeController *controller, SerialModuleRadio *radio);

