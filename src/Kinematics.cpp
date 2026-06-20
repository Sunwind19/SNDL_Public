#include "Kinematics.h"

#include "Hardware.h"
#include "RobotState.h"

namespace hexabot {

static float clampFloat(float value, float minValue, float maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static void keepCurrentServoAngles(uint8_t legNumber) {
    const uint8_t base = legNumber * SERVOS_PER_LEG;
    frameTargetCoxa[legNumber] = allServos[base + 0].currentAngleDeg;
    frameTargetFemur[legNumber] = allServos[base + 1].currentAngleDeg;
    frameTargetTibia[legNumber] = allServos[base + 2].currentAngleDeg;
}

static float compensateCoxaMounting(uint8_t legNumber, float coxaAngleDeg) {
    switch (legNumber) {
        case FRONT_RIGHT:
            return coxaAngleDeg + 45.0f;
        case MIDDLE_RIGHT:
            return coxaAngleDeg + 90.0f;
        case BACK_RIGHT:
            return coxaAngleDeg + 135.0f;
        case BACK_LEFT:
            return (coxaAngleDeg < 0.0f) ? coxaAngleDeg + 225.0f : coxaAngleDeg - 135.0f;
        case MIDDLE_LEFT:
            return (coxaAngleDeg < 0.0f) ? coxaAngleDeg + 270.0f : coxaAngleDeg - 90.0f;
        case FRONT_LEFT:
            return (coxaAngleDeg < 0.0f) ? coxaAngleDeg + 315.0f : coxaAngleDeg - 45.0f;
        default:
            return coxaAngleDeg;
    }
}

void computeLegIK(uint8_t legNumber, float x, float y, float z) {
    if (legNumber >= LEG_COUNT) {
        return;
    }

    const float l0 = sqrtf((x * x) + (y * y)) - COXA_LENGTH_MM;
    const float l3 = sqrtf((l0 * l0) + (z * z));

    const float maxReach = FEMUR_LENGTH_MM + TIBIA_LENGTH_MM + 0.1f;
    const float minReach = fabsf(TIBIA_LENGTH_MM - FEMUR_LENGTH_MM) - 0.1f;

    if (l3 >= maxReach || l3 <= minReach || l3 <= 0.001f) {
        keepCurrentServoAngles(legNumber);
        return;
    }

    const float tibiaCos = clampFloat(
        ((FEMUR_LENGTH_MM * FEMUR_LENGTH_MM) + (TIBIA_LENGTH_MM * TIBIA_LENGTH_MM) - (l3 * l3)) /
            (2.0f * FEMUR_LENGTH_MM * TIBIA_LENGTH_MM),
        -1.0f,
        1.0f
    );
    const float phiTibia = acosf(tibiaCos);
    const float thetaTibia = (phiTibia * RAD_TO_DEG) - 40.0f;

    const float gammaFemur = atan2f(z, l0);
    const float femurCos = clampFloat(
        ((FEMUR_LENGTH_MM * FEMUR_LENGTH_MM) + (l3 * l3) - (TIBIA_LENGTH_MM * TIBIA_LENGTH_MM)) /
            (2.0f * FEMUR_LENGTH_MM * l3),
        -1.0f,
        1.0f
    );
    const float phiFemur = acosf(femurCos);
    const float thetaFemur = ((phiFemur + gammaFemur) * RAD_TO_DEG) + 90.0f;

    const float rawCoxa = atan2f(x, y) * RAD_TO_DEG;
    const float thetaCoxa = compensateCoxaMounting(legNumber, rawCoxa);

    frameTargetCoxa[legNumber] = thetaCoxa;
    frameTargetFemur[legNumber] = thetaFemur;
    frameTargetTibia[legNumber] = thetaTibia;
}

}  // namespace hexabot
