#pragma once

#include <Arduino.h>
#include "Config.h"

namespace hexabot {

enum class RobotMode : int8_t {
    Pairing = -1,
    Waiting = 0,
    Walking = 1,
    Following = 2,
    Calibration = 99,
};

enum class SquatState : uint8_t {
    Idle,
    Down,
    Up,
};

extern RobotMode mode;

extern int tripodCase[LEG_COUNT];
extern int gaitTick;

extern int commandedX;
extern int commandedY;
extern int commandedR;
extern bool rotatingInPlaceMode;

extern float globalSpeedMultiplier;
extern float currentBodyZOffset;

extern float currentX[LEG_COUNT];
extern float currentY[LEG_COUNT];
extern float currentZ[LEG_COUNT];

extern float offsetX[LEG_COUNT];
extern float offsetY[LEG_COUNT];
extern float offsetZ[LEG_COUNT];

extern float frameTargetCoxa[LEG_COUNT];
extern float frameTargetFemur[LEG_COUNT];
extern float frameTargetTibia[LEG_COUNT];

extern float targetBodyPitch;
extern float targetBodyRoll;
extern float currentBodyPitch;
extern float currentBodyRoll;

extern float currentTemporaryZOffset;
extern float targetTemporaryZOffset;
extern unsigned long squatStartTime;
extern SquatState squatState;

void resetMotionCommands();
void resetBodyAttitude();
void resetSquatState();

}  // namespace hexabot
