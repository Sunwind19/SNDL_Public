#include "Robot.h"

#include <Arduino.h>
#include "Config.h"
#include "Gait.h"
#include "Hardware.h"
#include "RobotState.h"
#include "StatusLed.h"
#include "Controller Input.h"

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

}  // namespace hexabot // namespace hexabot // namespace hexabot // namespace hexabot // namespace hexabot
/*
#include "Robot.h"

#include <Arduino.h>
#include "Config.h"
#include "Gait.h"
#include "Hardware.h"
#include "RobotState.h"
#include "StatusLed.h"
#include "Controller Input.h"

namespace hexabot {

static unsigned long previousFrameTime = 0;

static bool controllerIsAlreadyConnected() {
    ControllerPtr controller = getActiveController();
    return controller != nullptr;
}

static void enterWaitingMode(uint16_t settleMs) {
    mode = RobotMode::Waiting;
    setStatusLed(false);
    resetMotionCommands();
    resetBodyAttitude();
    resetSquatState();
    setWaitingPoseImmediate(settleMs);

    Serial.println("Controller ready. Robot entered WAITING mode.");
    Serial.printf("Speed multiplier: %.1f\n", globalSpeedMultiplier);
    Serial.printf("Body height Z offset: %.1f mm\n", currentBodyZOffset);
}

static bool handlePairingMode(ControllerPtr controller) {
    if (mode != RobotMode::Pairing) {
        return false;
    }

    if (controller != nullptr) {
        enterWaitingMode(PAIRING_SETTLE_MS);
        return false;
    }

    updatePairingBlink();
    vTaskDelay(1);
    return true;
}

static void updateFrame(ControllerPtr controller) {
    readMovementInputsAndActions(controller);

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

        case RobotMode::Calibration:
            fillCalibrationPoseTargets();
            gaitTick = 0;
            resetBodyAttitude();
            currentTemporaryZOffset = 0.0f;
            break;

        case RobotMode::Following:
            updateFollowingPose();
            break;

        case RobotMode::Pairing:
        default:
            return;
    }

    interpolateServosToFrameTargets();
}

void setupRobot() {
    Serial.begin(115200);
    delay(200);

    initializeStatusLed();
    initializeServoDrivers();
    initializeControllerInput();

    delay(STARTUP_SETTLE_MS);

    if (controllerIsAlreadyConnected()) {
        enterWaitingMode(STARTUP_SETTLE_MS);
    } else {
        mode = RobotMode::Pairing;
        startPairingBlink();
        Serial.println("No paired controller detected. Entering PAIRING mode.");
    }

    previousFrameTime = millis();
}

void updateRobot() {
    const unsigned long now = millis();

    updateControllerInput();
    ControllerPtr controller = getActiveController();

    if (handlePairingMode(controller)) {
        return;
    }

    handleModeButtons(controller);
    handleDpadAdjustments(controller);

    if (now - previousFrameTime >= FRAME_TIME_MS) {
        previousFrameTime = now;
        updateFrame(controller);
    }

    // Give FreeRTOS time for Bluetooth and other ESP32 background tasks.
    vTaskDelay(1);
}

}  // namespace hexabot
*/