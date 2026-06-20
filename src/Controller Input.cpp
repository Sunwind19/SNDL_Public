#include "Controller Input.h"

#include "Config.h"
#include "Gait.h"
#include "RobotState.h"
#include "StatusLed.h"

namespace hexabot {

static Controller* controllers[BP32_MAX_GAMEPADS] = {};

static bool lastYButtonState = false;
static bool lastXButtonState = false;
static bool lastAButtonState = false;
static bool lastDpadUpState = false;
static bool lastDpadDownState = false;
static bool lastDpadLeftState = false;
static bool lastDpadRightState = false;

static float mapFloat(float value, float inMin, float inMax, float outMin, float outMax) {
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

static int axisToCommand(int axis, bool invert = false) {
    int value = static_cast<int>(roundf(mapFloat(static_cast<float>(axis), -512.0f, 511.0f, -127.0f, 127.0f)));
    value = constrain(value, -127, 127);
    return invert ? -value : value;
}

static float axisToAngle(int axis, float minAngle, float maxAngle) {
    return mapFloat(static_cast<float>(axis), -512.0f, 511.0f, minAngle, maxAngle);
}

static void onConnectedController(Controller* controller) {
    Serial.println("Controller connected callback fired.");
    Serial.printf("  Connected: %s\n", controller->isConnected() ? "true" : "false");
    Serial.printf("  Gamepad: %s\n", controller->isGamepad() ? "true" : "false");

    for (uint8_t i = 0; i < BP32_MAX_GAMEPADS; ++i) {
        if (controllers[i] == nullptr) {
            controllers[i] = controller;
            Serial.printf("  Assigned controller to slot %u.\n", i);
            return;
        }
    }

    Serial.println("  No free controller slots available.");
}

static void onDisconnectedController(Controller* controller) {
    Serial.println("Controller disconnected. Bluepad32 will keep trying to reconnect known devices.");

    for (uint8_t i = 0; i < BP32_MAX_GAMEPADS; ++i) {
        if (controllers[i] == controller) {
            controllers[i] = nullptr;
        }
    }
}

void initializeControllerInput() {
    BP32.setup(&onConnectedController, &onDisconnectedController);
    // Uncomment while testing if you need to erase all remembered Bluetooth pairings.
    // BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);
}

void updateControllerInput() {
    BP32.update();
}

Controller* getActiveController() {
    for (uint8_t i = 0; i < BP32_MAX_GAMEPADS; ++i) {
        Controller* controller = controllers[i];
        if (controller != nullptr && controller->isConnected() && controller->isGamepad()) {
            return controller;
        }
    }
    return nullptr;
}

void handleModeButtons(Controller* controller) {
    if (controller == nullptr) {
        lastYButtonState = false;
        lastXButtonState = false;
        return;
    }

    const bool currentY = controller->y();
    if (currentY && !lastYButtonState) {
        if (mode == RobotMode::Waiting) {
            mode = RobotMode::Walking;
            gaitTick = 0;
            resetTripodPhases();
            Serial.println("Mode: WALKING. Robot is rising into its home pose.");
        } else if (mode == RobotMode::Walking || mode == RobotMode::Following) {
            mode = RobotMode::Waiting;
            gaitTick = 0;
            resetSquatState();
            Serial.println("Mode: WAITING. Robot is returning to the safe waiting pose.");
        }
        quickFlashLed();
    }
    lastYButtonState = currentY;

    const bool currentX = controller->x();
    if (currentX && !lastXButtonState) {
        if (mode == RobotMode::Walking) {
            mode = RobotMode::Following;
            gaitTick = 0;
            resetBodyAttitude();
            resetSquatState();
            Serial.println("Mode: FOLLOWING. Body attitude follows the right joystick.");
            quickFlashLed();
        } else if (mode == RobotMode::Following) {
            mode = RobotMode::Walking;
            gaitTick = 0;
            resetBodyAttitude();
            resetSquatState();
            Serial.println("Mode: WALKING. Robot is returning to gait control.");
            quickFlashLed();
        }
    }
    lastXButtonState = currentX;
}

void handleDpadAdjustments(Controller* controller) {
    if (controller == nullptr) {
        lastDpadUpState = false;
        lastDpadDownState = false;
        lastDpadLeftState = false;
        lastDpadRightState = false;
        return;
    }

    const uint8_t dpad = controller->dpad();
    const bool dpadUp = (dpad & DPAD_UP) != 0;
    const bool dpadDown = (dpad & DPAD_DOWN) != 0;
    const bool dpadLeft = (dpad & DPAD_LEFT) != 0;
    const bool dpadRight = (dpad & DPAD_RIGHT) != 0;

    if (dpadUp && !lastDpadUpState) {
        globalSpeedMultiplier = min(globalSpeedMultiplier + SPEED_STEP, MAX_SPEED_MULTIPLIER);
        Serial.printf("Speed multiplier: %.1f\n", globalSpeedMultiplier);
        quickFlashLed();
    } else if (dpadDown && !lastDpadDownState) {
        globalSpeedMultiplier = max(globalSpeedMultiplier - SPEED_STEP, MIN_SPEED_MULTIPLIER);
        Serial.printf("Speed multiplier: %.1f\n", globalSpeedMultiplier);
        quickFlashLed();
    }

    if (dpadLeft && !lastDpadLeftState) {
        currentBodyZOffset = max(currentBodyZOffset - BODY_HEIGHT_STEP_MM, MIN_BODY_HEIGHT_OFFSET_MM);
        Serial.printf("Body height Z offset: %.1f mm\n", currentBodyZOffset);
        quickFlashLed();
    } else if (dpadRight && !lastDpadRightState) {
        currentBodyZOffset = min(currentBodyZOffset + BODY_HEIGHT_STEP_MM, MAX_BODY_HEIGHT_OFFSET_MM);
        Serial.printf("Body height Z offset: %.1f mm\n", currentBodyZOffset);
        quickFlashLed();
    }

    lastDpadUpState = dpadUp;
    lastDpadDownState = dpadDown;
    lastDpadLeftState = dpadLeft;
    lastDpadRightState = dpadRight;
}

static void updateSquatState() {
    if (squatState == SquatState::Idle) {
        return;
    }

    const unsigned long elapsedMs = millis() - squatStartTime;
    const float phaseDurationMs = SQUAT_DURATION_MS / 2.0f;

    if (squatState == SquatState::Down) {
        const float progress = min(1.0f, static_cast<float>(elapsedMs) / phaseDurationMs);
        currentTemporaryZOffset = targetTemporaryZOffset * progress;

        if (progress >= 1.0f) {
            currentTemporaryZOffset = targetTemporaryZOffset;
            squatState = SquatState::Up;
            squatStartTime = millis();
            Serial.println("Squat: moving up.");
        }
    } else if (squatState == SquatState::Up) {
        const float progress = min(1.0f, static_cast<float>(elapsedMs) / phaseDurationMs);
        currentTemporaryZOffset = targetTemporaryZOffset + (0.0f - targetTemporaryZOffset) * progress;

        if (progress >= 1.0f) {
            resetSquatState();
            Serial.println("Squat: finished.");
        }
    }
}

static void tryStartSquat(Controller* controller) {
    const bool currentA = controller->a();
    if (!currentA || lastAButtonState) {
        lastAButtonState = currentA;
        return;
    }

    bool allowed = false;
    if (mode == RobotMode::Walking) {
        allowed = gaitTick == 0 &&
                  !rotatingInPlaceMode &&
                  abs(commandedX) <= JOYSTICK_DEADZONE &&
                  abs(commandedY) <= JOYSTICK_DEADZONE &&
                  abs(commandedR) <= JOYSTICK_DEADZONE;
    } else if (mode == RobotMode::Following) {
        allowed = abs(currentBodyPitch) < 1.0f && abs(currentBodyRoll) < 1.0f;
    }

    if (allowed && squatState == SquatState::Idle) {
        squatState = SquatState::Down;
        squatStartTime = millis();
        targetTemporaryZOffset = MIN_BODY_HEIGHT_OFFSET_MM - currentBodyZOffset;
        Serial.printf(
            "Squat started. Temporary Z target: %.1f mm (body Z: %.1f mm).\n",
            targetTemporaryZOffset,
            currentBodyZOffset
        );
    } else if (!allowed) {
        Serial.println("Squat rejected. Stop movement and level the body first.");
    }

    lastAButtonState = currentA;
}

void readMovementInputsAndActions(Controller* controller) {
    resetMotionCommands();

    if (controller == nullptr) {
        resetBodyAttitude();
        resetSquatState();
        lastAButtonState = false;
        return;
    }

    if (mode != RobotMode::Walking && mode != RobotMode::Following) {
        resetBodyAttitude();
        resetSquatState();
        lastAButtonState = controller->a();
        return;
    }

    if (mode == RobotMode::Walking) {
        const int leftX = controller->axisX();
        const int leftY = controller->axisY();
        const int rightX = controller->axisRX();
        const bool rotateInPlaceButton = controller->b();

        if (rotateInPlaceButton) {
            rotatingInPlaceMode = true;
            if (abs(leftX) > JOYSTICK_DEADZONE) {
                commandedR = axisToCommand(leftX, true);
            }
        } else {
            if (abs(leftY) > JOYSTICK_DEADZONE) {
                commandedX = axisToCommand(leftY, true);
            }
            if (abs(leftX) > JOYSTICK_DEADZONE) {
                commandedY = axisToCommand(leftX, false);
            }
            if (abs(rightX) > JOYSTICK_DEADZONE) {
                commandedR = axisToCommand(rightX, true);
            }
        }
    } else if (mode == RobotMode::Following) {
        const int rightX = controller->axisRX();
        const int rightY = controller->axisRY();

        targetBodyRoll = abs(rightX) > JOYSTICK_DEADZONE
            ? axisToAngle(rightX, MAX_BODY_ROLL_DEG, -MAX_BODY_ROLL_DEG)
            : 0.0f;

        targetBodyPitch = abs(rightY) > JOYSTICK_DEADZONE
            ? axisToAngle(rightY, MAX_BODY_PITCH_DEG, -MAX_BODY_PITCH_DEG)
            : 0.0f;
    }

    tryStartSquat(controller);
    updateSquatState();
}

}  // namespace hexabot
