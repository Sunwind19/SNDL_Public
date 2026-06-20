#include "Robot.h"

#include <Arduino.h>
#include "Config.h"
#include "Gait.h"
#include "Hardware.h"
#include "RobotState.h"
#include "StatusLed.h"
#include "ControllerInput.h"

namespace hexabot {

static unsigned long previousFrameTime = 0;
static unsigned long bootStartTime = 0;
static int bootStage = 0;
static bool hardwareInitialized = false;

// ============================================
// SAFE BOOT SEQUENCE (NO BLOCKING I2C IN SETUP)
// ============================================

void setupRobot() {
    Serial.begin(115200);
    delay(500);
    
    Serial.println("\n[BOOT] Starting hexabot initialization...");
    
    // Phase 1: GPIO-only operations (no I2C)
    initializeStatusLed();
    setStatusLed(true);  // Status LED ON during boot
    
    // Do NOT initialize PCA9685 here!
    // Do NOT call Wire.begin() here!
    // Do NOT set Wire.setTimeOut() here!
    
    // Phase 2: Initialize global state
    mode = RobotMode::Waiting;
    resetMotionCommands();
    resetBodyAttitude();
    resetSquatState();
    
    // Phase 3: Wait for power rails to stabilize
    Serial.println("[BOOT] Phase 1: Waiting for power rails to stabilize (3s)...");
    delay(3000);
    
    // Record boot start time for lazy hardware init
    bootStartTime = millis();
    bootStage = 0;
    hardwareInitialized = false;
    previousFrameTime = millis();
    
    Serial.println("[BOOT] Setup complete. Waiting for frame loop...");
}

// ============================================
// LAZY HARDWARE INITIALIZATION (IN UPDATE LOOP)
// ============================================

void updateRobot() {
    const unsigned long now = millis();
    const unsigned long elapsedSinceBoot = now - bootStartTime;
    
    // ========================================
    // STAGE 0: Initialize Wire & PCA9685 (one-time, ~1.5s after boot)
    // ========================================
    if (bootStage == 0 && elapsedSinceBoot > 3500) {
        Serial.println("[BOOT] Stage 0: Initializing I2C and PCA9685...");
        
        // Now safe to initialize I2C and servo drivers
        Wire.begin(21, 22);  // SDA=21, SCL=22 for ESP32
        Wire.setClock(100000);  // 100 kHz I2C clock (stable, not fast)
        
        // Initialize both PCA9685 boards
        initializeServoDrivers();
        
        // Give servo drivers time to settle
        delay(500);
        
        Serial.println("[BOOT] Stage 0: PCA9685 boards initialized");
        bootStage = 1;
        return;  // Skip this frame, try again next frame
    }
    
    // ========================================
    // STAGE 1: Set waiting pose (standing neutral)
    // ========================================
    if (bootStage == 1 && elapsedSinceBoot > 4100) {
        Serial.println("[BOOT] Stage 1: Setting waiting pose...");
        
        fillWaitingPoseTargets();
        interpolateServosToFrameTargets();  // Single smooth interpolation
        
        gaitTick = 0;
        resetTripodPhases();
        
        delay(1000);  // Let servos settle into position
        
        Serial.println("[BOOT] Stage 1: Waiting pose set");
        bootStage = 2;
        return;
    }
    
    // ========================================
    // STAGE 2: Mark hardware as ready, allow walking
    // ========================================
    if (bootStage == 2 && elapsedSinceBoot > 5200) {
        Serial.println("[BOOT] Stage 2: Hardware fully initialized. Ready for motion.");
        
        hardwareInitialized = true;
        mode = RobotMode::Walking;
        bootStage = 3;
        setStatusLed(false);  // Status LED OFF after boot
        
        return;
    }
    
    // ========================================
    // If hardware not ready, wait (no motion)
    // ========================================
    if (!hardwareInitialized) {
        return;  // Still booting, do nothing
    }
    
    // ========================================
    // MAIN LOOP: Only runs after full boot
    // ========================================
    
    if (now - previousFrameTime >= FRAME_TIME_MS) {
        previousFrameTime = now;
        
        // Dummy motion: forward movement only (tripod gait)
        // Remove this and add controller input when ready
        commandedX = 80;    // Forward motion (safe low speed)
        commandedY = 0;     // No strafe
        commandedR = 0;     // No rotation
        rotatingInPlaceMode = false;
        
        // Update gait and servo positions
        switch (mode) {
            case RobotMode::Walking:
                updateTripodGait();
                currentBodyPitch = 0.0f;
                currentBodyRoll = 0.0f;
                break;
                
            case RobotMode::Waiting:
                fillWaitingPoseTargets();
                gaitTick = 0;
                resetBodyAttitude();
                currentTemporaryZOffset = 0.0f;
                break;
                
            default:
                fillWaitingPoseTargets();
                break;
        }
        
        // Send commands to servos
        interpolateServosToFrameTargets();
    }
    
    // Prevent watchdog timeout
    yield();
    vTaskDelay(1);
}

}  
// namespace hexabot
/*
#pragma once

#include <Arduino.h>

namespace hexabot {

constexpr uint8_t LEG_COUNT = 6;
constexpr uint8_t SERVOS_PER_LEG = 3;
constexpr uint8_t SERVO_COUNT = LEG_COUNT * SERVOS_PER_LEG;

// Leg index convention used everywhere in the firmware.
enum LegIndex : uint8_t {
    FRONT_RIGHT = 0,
    MIDDLE_RIGHT = 1,
    BACK_RIGHT = 2,
    BACK_LEFT = 3,
    MIDDLE_LEFT = 4,
    FRONT_LEFT = 5,
};

// Physical dimensions of the leg segments in millimetres.
constexpr float COXA_LENGTH_MM = 50.0f;
constexpr float FEMUR_LENGTH_MM = 65.0f;
constexpr float TIBIA_LENGTH_MM = 138.9f;

// Neutral toe positions relative to each coxa joint, in millimetres.
extern const float HOME_X[LEG_COUNT];
extern const float HOME_Y[LEG_COUNT];
extern const float HOME_Z[LEG_COUNT];

// Coxa joint positions relative to the body centre, in millimetres.
extern const float BODY_X[LEG_COUNT];
extern const float BODY_Y[LEG_COUNT];
extern const float BODY_Z[LEG_COUNT];

// Per-servo calibration offsets in degrees.
extern const int8_t COXA_CAL[LEG_COUNT];
extern const int8_t FEMUR_CAL[LEG_COUNT];
extern const int8_t TIBIA_CAL[LEG_COUNT];

// Motion and control parameters.
constexpr float BASE_TRAVEL_MM = 30.0f;
constexpr float STEP_HEIGHT_MULTIPLIER = 1.0f;
constexpr uint16_t FRAME_TIME_MS = 20;       // 50 Hz control loop.
constexpr float SERVO_SMOOTHING_FACTOR = 0.3f;
constexpr int JOYSTICK_DEADZONE = 50;        // Bluepad32 axes are roughly -512..511.

constexpr float DEFAULT_SPEED_MULTIPLIER = 3.5f;
constexpr float MIN_SPEED_MULTIPLIER = 2.5f;
constexpr float MAX_SPEED_MULTIPLIER = 5.0f;
constexpr float SPEED_STEP = 0.5f;

constexpr float MAX_BODY_PITCH_DEG = 20.0f;
constexpr float MAX_BODY_ROLL_DEG = 20.0f;
constexpr float MIN_BODY_HEIGHT_OFFSET_MM = -30.0f;
constexpr float MAX_BODY_HEIGHT_OFFSET_MM = 10.0f;
constexpr float BODY_HEIGHT_STEP_MM = 5.0f;
constexpr float SQUAT_DURATION_MS = 200.0f;

// ESP32 pins and PCA9685 configuration.
constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t STATUS_LED_PIN = 2;
constexpr uint8_t PCA9685_ADDRESS_LEFT = 0x40;
constexpr uint8_t PCA9685_ADDRESS_RIGHT = 0x41;
constexpr uint16_t SERVO_PWM_FREQUENCY_HZ = 50;
constexpr uint16_t SERVO_MIN_US = 500;
constexpr uint16_t SERVO_MAX_US = 2500;

// Safe pose used while waiting for the user to start motion.
constexpr float WAITING_COXA_DEG = 90.0f;
constexpr float WAITING_FEMUR_DEG = 175.0f;
constexpr float WAITING_TIBIA_DEG = 5.0f;

// Pose used for manual mechanical calibration.
constexpr float CALIBRATION_SERVO_DEG = 90.0f;

// Timing helpers.
constexpr uint16_t STARTUP_SETTLE_MS = 500;
constexpr uint16_t PAIRING_SETTLE_MS = 1000;
constexpr uint16_t LED_BLINK_INTERVAL_MS = 500;
constexpr uint8_t LED_FLASH_MS = 50;

}  // namespace hexabot
*/