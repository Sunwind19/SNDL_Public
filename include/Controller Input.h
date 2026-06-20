#pragma once

#include <Arduino.h>
#include <Bluepad32.h>

namespace hexabot {

void initializeControllerInput();
void updateControllerInput();
Controller* getActiveController();
void handleModeButtons(Controller* controller);
void handleDpadAdjustments(Controller* controller);
void readMovementInputsAndActions(Controller* controller);

/*
void handleModeButtons(ControllerPtr controller);
void handleDpadAdjustments(ControllerPtr controller);
void readMovementInputsAndActions(ControllerPtr controller);
*/
}  // namespace hexabot
