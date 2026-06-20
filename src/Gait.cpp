#include <Arduino.h>
#include "Gait.h"
#include "Config.h"
#include "Kinematics.h"
#include "RobotState.h"

namespace hexabot {

struct StepAmplitude {
    float x;
    float y;
    float z;
};

static float strideX = 0.0f;
static float strideY = 0.0f;
static float strideR = 0.0f;
static float sinRotZ = 0.0f;
static float cosRotZ = 1.0f;
static long gaitDurationMs = 0;

static int maxAbs3(int a, int b, int c) {
    return max(max(abs(a), abs(b)), abs(c));
}

static float mapFloat(float value, float inMin, float inMax, float outMin, float outMax) {
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void resetTripodPhases() {
    tripodCase[0] = 1;
    tripodCase[1] = 2;
    tripodCase[2] = 1;
    tripodCase[3] = 2;
    tripodCase[4] = 1;
    tripodCase[5] = 2;
}

static void computeStrides() {
    const int commandMagnitude = maxAbs3(commandedX, commandedY, commandedR);

    constexpr float BASE_MIN_DURATION_MS = 1080.0f * 2.0f;
    constexpr float BASE_MAX_DURATION_MS = 3240.0f * 2.0f;

    const float scaledMinDurationMs = BASE_MIN_DURATION_MS / globalSpeedMultiplier;
    const float scaledMaxDurationMs = BASE_MAX_DURATION_MS / globalSpeedMultiplier;
    const float currentTravel = BASE_TRAVEL_MM * globalSpeedMultiplier;

    if (rotatingInPlaceMode) {
        strideX = 0.0f;
        strideY = 0.0f;
        strideR = (currentTravel / 2.0f) * (static_cast<float>(commandedR) / 127.0f);
    } else {
        strideX = currentTravel * static_cast<float>(commandedX) / 127.0f;
        strideY = currentTravel * static_cast<float>(commandedY) / 127.0f;
        strideR = (currentTravel / 2.0f) * static_cast<float>(commandedR) / 127.0f;
    }

    if (commandMagnitude == 0) {
        gaitDurationMs = static_cast<long>(scaledMaxDurationMs);
    } else {
        gaitDurationMs = static_cast<long>(mapFloat(
            static_cast<float>(commandMagnitude),
            1.0f,
            127.0f,
            scaledMaxDurationMs,
            scaledMinDurationMs
        ));
    }

    sinRotZ = sinf(radians(strideR));
    cosRotZ = cosf(radians(strideR));
}

static StepAmplitude computeAmplitudes(uint8_t leg) {
    const float totalX = HOME_X[leg] + BODY_X[leg];
    const float totalY = HOME_Y[leg] + BODY_Y[leg];

    float rotOffsetX = 0.0f;
    float rotOffsetY = 0.0f;

    if (commandedR != 0) {
        rotOffsetX = totalY * sinRotZ + totalX * cosRotZ - totalX;
        rotOffsetY = totalY * cosRotZ - totalX * sinRotZ - totalY;
    }

    StepAmplitude amplitude;
    amplitude.x = constrain((strideX + rotOffsetX) / 2.0f, -80.0f, 80.0f);
    amplitude.y = constrain((strideY + rotOffsetY) / 2.0f, -80.0f, 80.0f);

    const float travelX = fabsf(strideX + rotOffsetX);
    const float travelY = fabsf(strideY + rotOffsetY);
    amplitude.z = STEP_HEIGHT_MULTIPLIER * max(travelX, travelY) / 4.0f;

    if (amplitude.z < 10.0f && (commandedX != 0 || commandedY != 0 || commandedR != 0)) {
        amplitude.z = 10.0f;
    }

    return amplitude;
}

static void updateLegHomeTargets() {
    for (uint8_t leg = 0; leg < LEG_COUNT; ++leg) {
        currentX[leg] = HOME_X[leg];
        currentY[leg] = HOME_Y[leg];
        currentZ[leg] = HOME_Z[leg] + currentBodyZOffset + currentTemporaryZOffset;

        computeLegIK(
            leg,
            currentX[leg] + offsetX[leg],
            currentY[leg] + offsetY[leg],
            currentZ[leg] + offsetZ[leg]
        );
    }
}

void updateTripodGait() {
    const bool hasCommand =
        abs(commandedX) > JOYSTICK_DEADZONE ||
        abs(commandedY) > JOYSTICK_DEADZONE ||
        abs(commandedR) > JOYSTICK_DEADZONE;

    if (!hasCommand && gaitTick == 0) {
        updateLegHomeTargets();
        return;
    }

    if (!hasCommand && gaitTick != 0) {
        gaitTick = 0;
        resetTripodPhases();
        updateLegHomeTargets();
        return;
    }

    computeStrides();

    int halfCycleTicks = roundf(static_cast<float>(gaitDurationMs) / FRAME_TIME_MS / 2.0f);
    if (halfCycleTicks < 1) {
        halfCycleTicks = 1;
    }

    for (uint8_t leg = 0; leg < LEG_COUNT; ++leg) {
        const StepAmplitude amplitude = computeAmplitudes(leg);
        const float phase = PI * static_cast<float>(gaitTick) / static_cast<float>(halfCycleTicks);

        if (tripodCase[leg] == 1) {
            currentX[leg] = HOME_X[leg] - amplitude.x * cosf(phase);
            currentY[leg] = HOME_Y[leg] - amplitude.y * cosf(phase);
            currentZ[leg] = HOME_Z[leg] + currentBodyZOffset + currentTemporaryZOffset + amplitude.z * sinf(phase);

            if (gaitTick >= halfCycleTicks - 1) {
                tripodCase[leg] = 2;
            }
        } else {
            currentX[leg] = HOME_X[leg] + amplitude.x * cosf(phase);
            currentY[leg] = HOME_Y[leg] + amplitude.y * cosf(phase);
            currentZ[leg] = HOME_Z[leg] + currentBodyZOffset + currentTemporaryZOffset;

            if (gaitTick >= halfCycleTicks - 1) {
                tripodCase[leg] = 1;
            }
        }

        computeLegIK(
            leg,
            currentX[leg] + offsetX[leg],
            currentY[leg] + offsetY[leg],
            currentZ[leg] + offsetZ[leg]
        );
    }

    if (gaitTick < halfCycleTicks - 1) {
        ++gaitTick;
    } else {
        gaitTick = 0;
    }
}

void updateFollowingPose() {
    currentBodyPitch += (targetBodyPitch - currentBodyPitch) * SERVO_SMOOTHING_FACTOR;
    currentBodyRoll += (targetBodyRoll - currentBodyRoll) * SERVO_SMOOTHING_FACTOR;

    const float sinPitch = sinf(radians(currentBodyPitch));
    const float cosPitch = cosf(radians(currentBodyPitch));
    const float sinRoll = sinf(radians(currentBodyRoll));
    const float cosRoll = cosf(radians(currentBodyRoll));

    for (uint8_t leg = 0; leg < LEG_COUNT; ++leg) {
        const float bodyX = BODY_X[leg];
        const float bodyY = BODY_Y[leg];
        const float bodyZ = BODY_Z[leg];

        const float pitchX = bodyX * cosPitch + bodyZ * sinPitch;
        const float pitchY = bodyY;
        const float pitchZ = -bodyX * sinPitch + bodyZ * cosPitch;

        const float rotatedX = pitchX;
        const float rotatedY = pitchY * cosRoll - pitchZ * sinRoll;
        const float rotatedZ = pitchY * sinRoll + pitchZ * cosRoll;

        const float deltaX = rotatedX - bodyX;
        const float deltaY = rotatedY - bodyY;
        const float deltaZ = rotatedZ - bodyZ;

        currentX[leg] = HOME_X[leg] - deltaX + offsetX[leg];
        currentY[leg] = HOME_Y[leg] - deltaY + offsetY[leg];
        currentZ[leg] = HOME_Z[leg] - deltaZ + currentBodyZOffset + currentTemporaryZOffset + offsetZ[leg];

        computeLegIK(leg, currentX[leg], currentY[leg], currentZ[leg]);
    }

    gaitTick = 0;
}

}  // namespace hexabot
