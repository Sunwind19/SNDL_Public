#include <Arduino.h>
#include "Robot.h"

void setup() {
    hexabot::setupRobot();
}

void loop() {
    hexabot::updateRobot();
}
