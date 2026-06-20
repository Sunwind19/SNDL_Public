// Hardware.cpp - Simplified PCA9685 initialization
// Put this in your src/Hardware.cpp or adapt to your existing code

#include "Hardware.h"
#include <Wire.h>
#include <Arduino.h>

namespace hexabot {

// PCA9685 I2C addresses (example: both on address 0x40)
static const uint8_t PCA9685_ADDR_LEFT = 0x40;    // Left leg/arm board
static const uint8_t PCA9685_ADDR_RIGHT = 0x41;   // Right leg/tong board

// ============================================
// PCA9685 Register definitions
// ============================================
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_LED0_ON_H 0x07
#define PCA9685_LED0_OFF_L 0x08
#define PCA9685_LED0_OFF_H 0x09

#define PCA9685_MODE1_SLEEP (1 << 4)
#define PCA9685_MODE1_AI (1 << 5)     // Auto-increment
#define PCA9685_MODE1_RESTART (1 << 7)

static void pca9685_write_register(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t pca9685_read_register(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);  // Don't release bus
    
    Wire.requestFrom((int)addr, 1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0xFF;
}

void initializeServoDrivers() {
    Serial.println("[HW] Initializing PCA9685 boards...");
    
    // Initialize left board (legs)
    initializePCA9685(PCA9685_ADDR_LEFT, "Left");
    
    // Initialize right board (more servos)
    initializePCA9685(PCA9685_ADDR_RIGHT, "Right");
    
    Serial.println("[HW] PCA9685 initialization complete");
}

void initializePCA9685(uint8_t addr, const char* name) {
    Serial.printf("[HW] Initializing PCA9685 (%s) at 0x%02X\n", name, addr);
    
    // Step 1: Exit sleep mode
    uint8_t mode1 = pca9685_read_register(addr, PCA9685_MODE1);
    mode1 &= ~PCA9685_MODE1_SLEEP;  // Clear sleep bit
    pca9685_write_register(addr, PCA9685_MODE1, mode1 | PCA9685_MODE1_AI);
    
    // Step 2: Set prescaler for 50Hz (servo standard frequency)
    // Prescaler = (25MHz / (4096 * 50Hz)) - 1 = 121 (0x79)
    pca9685_write_register(addr, PCA9685_PRESCALE, 121);
    
    // Step 3: Set all channels to 0% (all servos neutral at startup)
    // Faster than setting 16 channels individually
    Wire.beginTransmission(addr);
    Wire.write(PCA9685_LED0_ON_L);
    Wire.write(0x00);      // ON_L = 0
    Wire.write(0x00);      // ON_H = 0
    Wire.write(0x00);      // OFF_L = 0
    Wire.write(0x10);      // OFF_H = 0x10 (center position for MG90S: 1500µs)
    Wire.endTransmission();
    
    delay(50);  // Small delay for register write to settle
    
    Serial.printf("[HW] %s board ready (50Hz PWM, neutral position)\n", name);
}

void setServoAngle(uint8_t boardAddr, uint8_t channel, float angleDegrees) {
    // Map angle to PWM pulse width (MG90S: 500-2500µs for -90 to +90°)
    // Center: 1500µs = 0°
    // Formula: pulse = 1500 + (angle * 1000/180)
    
    float pulse_ms = 1.5f + (angleDegrees * 10.0f / 180.0f);
    pulse_ms = constrain(pulse_ms, 1.0f, 2.0f);  // Clamp to safe range
    
    // Convert to PCA9685 tick count (4096 ticks per 20ms period = 50Hz)
    uint16_t ticks = (uint16_t)(pulse_ms * 4096.0f / 20.0f);
    ticks = constrain(ticks, 200, 500);  // Safe limits (1-2.5ms)
    
    // Write to channel
    uint8_t on_l = 0;
    uint8_t on_h = 0;
    uint8_t off_l = ticks & 0xFF;
    uint8_t off_h = (ticks >> 8) & 0x0F;
    
    uint8_t reg = PCA9685_LED0_ON_L + (channel * 4);
    
    Wire.beginTransmission(boardAddr);
    Wire.write(reg);
    Wire.write(on_l);
    Wire.write(on_h);
    Wire.write(off_l);
    Wire.write(off_h);
    Wire.endTransmission();
}

} 
 // namespace hexabot
 /*
#include <Wire.h>
#include "RobotState.h"
#include "Hardware.h"

namespace hexabot {

Adafruit_PWMServoDriver pwmLeft(PCA9685_ADDRESS_LEFT);
Adafruit_PWMServoDriver pwmRight(PCA9685_ADDRESS_RIGHT);

// Right side: 0=front right, 1=middle right, 2=back right.
static ServoConfig frontRightCoxa  = { &pwmRight, 15, "front right coxa",  COXA_CAL[FRONT_RIGHT], false };
static ServoConfig frontRightFemur = { &pwmRight, 14, "front right femur", FEMUR_CAL[FRONT_RIGHT], false };
static ServoConfig frontRightTibia = { &pwmRight, 13, "front right tibia", TIBIA_CAL[FRONT_RIGHT], false };

static ServoConfig middleRightCoxa  = { &pwmRight,  4, "middle right coxa",  COXA_CAL[MIDDLE_RIGHT], false };
static ServoConfig middleRightFemur = { &pwmRight, 5, "middle right femur", FEMUR_CAL[MIDDLE_RIGHT], false };
static ServoConfig middleRightTibia = { &pwmRight, 6, "middle right tibia", TIBIA_CAL[MIDDLE_RIGHT], false };

static ServoConfig backRightCoxa  = { &pwmRight, 0, "back right coxa",  COXA_CAL[BACK_RIGHT], false };
static ServoConfig backRightFemur = { &pwmRight, 1, "back right femur", FEMUR_CAL[BACK_RIGHT], false };
static ServoConfig backRightTibia = { &pwmRight, 2, "back right tibia", TIBIA_CAL[BACK_RIGHT], false };

// Left side: 3=back left, 4=middle left, 5=front left.
static ServoConfig backLeftCoxa  = { &pwmLeft, 13, "back left coxa",  COXA_CAL[BACK_LEFT], false };
static ServoConfig backLeftFemur = { &pwmLeft, 14, "back left femur", FEMUR_CAL[BACK_LEFT], false };
static ServoConfig backLeftTibia = { &pwmLeft, 15, "back left tibia", TIBIA_CAL[BACK_LEFT], false };

static ServoConfig middleLeftCoxa  = { &pwmLeft, 9, "middle left coxa",  COXA_CAL[MIDDLE_LEFT], false };
static ServoConfig middleLeftFemur = { &pwmLeft, 10, "middle left femur", FEMUR_CAL[MIDDLE_LEFT], false };
static ServoConfig middleLeftTibia = { &pwmLeft, 11, "middle left tibia", TIBIA_CAL[MIDDLE_LEFT], false };

static ServoConfig frontLeftCoxa  = { &pwmLeft, 0, "front left coxa",  COXA_CAL[FRONT_LEFT], false };
static ServoConfig frontLeftFemur = { &pwmLeft, 1, "front left femur", FEMUR_CAL[FRONT_LEFT], false };
static ServoConfig frontLeftTibia = { &pwmLeft, 2, "front left tibia", TIBIA_CAL[FRONT_LEFT], false };

ServoConfig* coxaServos[LEG_COUNT] = {
    &frontRightCoxa, &middleRightCoxa, &backRightCoxa,
    &backLeftCoxa, &middleLeftCoxa, &frontLeftCoxa,
};

ServoConfig* femurServos[LEG_COUNT] = {
    &frontRightFemur, &middleRightFemur, &backRightFemur,
    &backLeftFemur, &middleLeftFemur, &frontLeftFemur,
};

ServoConfig* tibiaServos[LEG_COUNT] = {
    &frontRightTibia, &middleRightTibia, &backRightTibia,
    &backLeftTibia, &middleLeftTibia, &frontLeftTibia,
};

ServoState allServos[SERVO_COUNT] = {
    { &frontRightCoxa, 0.0f },  { &frontRightFemur, 0.0f },  { &frontRightTibia, 0.0f },
    { &middleRightCoxa, 0.0f }, { &middleRightFemur, 0.0f }, { &middleRightTibia, 0.0f },
    { &backRightCoxa, 0.0f },   { &backRightFemur, 0.0f },   { &backRightTibia, 0.0f },
    { &backLeftCoxa, 0.0f },    { &backLeftFemur, 0.0f },    { &backLeftTibia, 0.0f },
    { &middleLeftCoxa, 0.0f },  { &middleLeftFemur, 0.0f },  { &middleLeftTibia, 0.0f },
    { &frontLeftCoxa, 0.0f },   { &frontLeftFemur, 0.0f },   { &frontLeftTibia, 0.0f },
};

void initializeServoDrivers() {
    Wire.begin(SDA_PIN, SCL_PIN);

    pwmLeft.begin();
    pwmLeft.setPWMFreq(SERVO_PWM_FREQUENCY_HZ);
    delay(50);

    pwmRight.begin();
    pwmRight.setPWMFreq(SERVO_PWM_FREQUENCY_HZ);
    delay(50);

    Serial.println("PCA9685 servo drivers initialized.");
}

uint16_t angleToTicks(int angleDeg) {
    angleDeg = constrain(angleDeg, 0, 180);

    const float pulseUs = SERVO_MIN_US +
        (static_cast<float>(angleDeg) / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
    const float usPerTick = 1000000.0f / SERVO_PWM_FREQUENCY_HZ / 4096.0f;
    return static_cast<uint16_t>(pulseUs / usPerTick);
}

void setServoAngle(const ServoConfig& config, float angleDeg) {
    int finalAngle = static_cast<int>(roundf(angleDeg));

    if (config.inverted) {
        finalAngle = 180 - finalAngle;
    }

    finalAngle += config.offsetDeg;
    config.driver->setPWM(config.channel, 0, angleToTicks(finalAngle));
}

void fillWaitingPoseTargets() {
    for (uint8_t leg = 0; leg < LEG_COUNT; ++leg) {
        frameTargetCoxa[leg] = WAITING_COXA_DEG;
        frameTargetFemur[leg] = WAITING_FEMUR_DEG;
        frameTargetTibia[leg] = WAITING_TIBIA_DEG;
    }
}

void fillCalibrationPoseTargets() {
    for (uint8_t leg = 0; leg < LEG_COUNT; ++leg) {
        frameTargetCoxa[leg] = CALIBRATION_SERVO_DEG;
        frameTargetFemur[leg] = CALIBRATION_SERVO_DEG;
        frameTargetTibia[leg] = CALIBRATION_SERVO_DEG;
    }
}

void setWaitingPoseImmediate(uint16_t settleMs) {
    fillWaitingPoseTargets();

    for (uint8_t leg = 0; leg < LEG_COUNT; ++leg) {
        const uint8_t base = leg * SERVOS_PER_LEG;
        allServos[base + 0].currentAngleDeg = frameTargetCoxa[leg];
        allServos[base + 1].currentAngleDeg = frameTargetFemur[leg];
        allServos[base + 2].currentAngleDeg = frameTargetTibia[leg];

        setServoAngle(*coxaServos[leg], allServos[base + 0].currentAngleDeg);
        setServoAngle(*femurServos[leg], allServos[base + 1].currentAngleDeg);
        setServoAngle(*tibiaServos[leg], allServos[base + 2].currentAngleDeg);
    }

    if (settleMs > 0) {
        delay(settleMs);
    }
}

void interpolateServosToFrameTargets() {
    for (uint8_t servo = 0; servo < SERVO_COUNT; ++servo) {
        const uint8_t leg = servo / SERVOS_PER_LEG;
        const uint8_t joint = servo % SERVOS_PER_LEG;

        float targetAngle = frameTargetTibia[leg];
        if (joint == 0) {
            targetAngle = frameTargetCoxa[leg];
        } else if (joint == 1) {
            targetAngle = frameTargetFemur[leg];
        }

        const float currentAngle = allServos[servo].currentAngleDeg;
        const float interpolatedAngle = currentAngle +
            (targetAngle - currentAngle) * SERVO_SMOOTHING_FACTOR;

        allServos[servo].currentAngleDeg = interpolatedAngle;
        setServoAngle(*allServos[servo].config, interpolatedAngle);
    }
}

}  // namespace hexabot
*/