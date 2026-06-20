#include "RobotState.h"

namespace hexabot {

RobotMode mode = RobotMode::Pairing;

int tripodCase[LEG_COUNT] = { 1, 2, 1, 2, 1, 2 };
int gaitTick = 0;

int commandedX = 0;
int commandedY = 0;
int commandedR = 0;
bool rotatingInPlaceMode = false;

float globalSpeedMultiplier = DEFAULT_SPEED_MULTIPLIER;
float currentBodyZOffset = 0.0f;

float currentX[LEG_COUNT] = {};
float currentY[LEG_COUNT] = {};
float currentZ[LEG_COUNT] = {};

float offsetX[LEG_COUNT] = {};
float offsetY[LEG_COUNT] = {};
float offsetZ[LEG_COUNT] = {};

float frameTargetCoxa[LEG_COUNT] = {};
float frameTargetFemur[LEG_COUNT] = {};
float frameTargetTibia[LEG_COUNT] = {};

float targetBodyPitch = 0.0f;
float targetBodyRoll = 0.0f;
float currentBodyPitch = 0.0f;
float currentBodyRoll = 0.0f;

float currentTemporaryZOffset = 0.0f;
float targetTemporaryZOffset = 0.0f;
unsigned long squatStartTime = 0;
SquatState squatState = SquatState::Idle;

void resetMotionCommands() {
    commandedX = 0;
    commandedY = 0;
    commandedR = 0;
    rotatingInPlaceMode = false;
}

void resetBodyAttitude() {
    targetBodyPitch = 0.0f;
    targetBodyRoll = 0.0f;
    currentBodyPitch = 0.0f;
    currentBodyRoll = 0.0f;
}

void resetSquatState() {
    currentTemporaryZOffset = 0.0f;
    targetTemporaryZOffset = 0.0f;
    squatStartTime = 0;
    squatState = SquatState::Idle;
}

}  // namespace hexabot
