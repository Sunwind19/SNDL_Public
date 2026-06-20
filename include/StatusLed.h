#pragma once

#include <Arduino.h>

namespace hexabot {

void initializeStatusLed();
void setStatusLed(bool enabled);
void quickFlashLed();
void startPairingBlink();
void updatePairingBlink();

}  // namespace hexabot
