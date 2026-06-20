#include "StatusLed.h"

#include "Config.h"

namespace hexabot {

static unsigned long ledBlinkTimer = 0;
static bool ledState = false;

void initializeStatusLed() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    setStatusLed(true);
}

void setStatusLed(bool enabled) {
    ledState = enabled;
    digitalWrite(STATUS_LED_PIN, enabled ? HIGH : LOW);
}

void quickFlashLed() {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(LED_FLASH_MS);
    digitalWrite(STATUS_LED_PIN, LOW);
    ledState = false;
}

void startPairingBlink() {
    ledBlinkTimer = millis();
    setStatusLed(true);
}

void updatePairingBlink() {
    const unsigned long now = millis();
    if (now - ledBlinkTimer < LED_BLINK_INTERVAL_MS) {
        return;
    }

    ledBlinkTimer = now;
    ledState = !ledState;
    digitalWrite(STATUS_LED_PIN, ledState ? HIGH : LOW);
}

}  // namespace hexabot
