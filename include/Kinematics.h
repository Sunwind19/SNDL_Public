#pragma once

#include <Arduino.h>
#include "Config.h"

namespace hexabot {

void computeLegIK(uint8_t legNumber, float x, float y, float z);

}  // namespace hexabot
