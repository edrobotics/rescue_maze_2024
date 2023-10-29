#include <Arduino.h>
#include "MotorController.h"

MotorController testController {2, 5, 3, 4, 46.85, 48};


void setup()
{
    pinMode(A1, INPUT);
    Serial.begin(115200);
    testController.init();
    testController.isReversed = false;
    // testController.setPWM(200);
    // delay(42);
    // testController.setPWM(-255);
    // testController.setSpeed(107);
    testController.pwmRPMCalibration();
}

void loop()
{
    // int corrVal = analogRead(A1)*float(255)/float(1024);
    // testController.setPWM(corrVal);
    // testController.update(false);
    // testController.printValues();
    // testController.printMotorSpeed();
    delay(10);
}
