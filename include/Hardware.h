#pragma once

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "Config.h"

namespace hexabot {

struct ServoConfig {
    Adafruit_PWMServoDriver* driver;
    uint8_t channel;
    const char* label;
    int8_t offsetDeg;
    bool inverted;
};

struct ServoState {
    ServoConfig* config;
    float currentAngleDeg;
};

extern Adafruit_PWMServoDriver pwmLeft;
extern Adafruit_PWMServoDriver pwmRight;

extern ServoConfig* coxaServos[LEG_COUNT];
extern ServoConfig* femurServos[LEG_COUNT];
extern ServoConfig* tibiaServos[LEG_COUNT];

extern ServoState allServos[SERVO_COUNT];

void initializeServoDrivers();
uint16_t angleToTicks(int angleDeg);
void setServoAngle(const ServoConfig& config, float angleDeg);

void fillWaitingPoseTargets();
void fillCalibrationPoseTargets();
void setWaitingPoseImmediate(uint16_t settleMs);
void interpolateServosToFrameTargets();

}  // namespace hexabot
