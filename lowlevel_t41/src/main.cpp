#include <Arduino.h>
#include "MotorController.h"

//                     ENCA, ENCB, PWM, DIR, ENCRATIO, CPR
MotorController motorRF {3,     2,   7,   6,    46.85,  48};
MotorController motorLF {39,   38,  28,  29,    46.85,  48};
MotorController motorRB {1,     0,   5,   4,    46.85,  48};
MotorController motorLB {23,   22,   37, 36,    46.85,  48};

int pwmPin = 28;
int dirPin = 29;

void setup()
{
  // pinMode(dirPin, OUTPUT);
  // pinMode(pwmPin, OUTPUT);
  // analogWriteFrequency(pwmPin, 80000);
  // delay(1000);
  motorRF.init();
  motorLF.init();
  motorLB.init();
  motorRB.init();
  motorRF.isReversed = true;
  motorRB.isReversed = true;
  delay(1000);
}

void loop()
{
  static long timeflag;

  motorRF.setSpeed(100);
  motorRB.setSpeed(100);
  motorLF.setSpeed(100);
  motorLB.setSpeed(100);
  timeflag = millis();
  while (millis()-timeflag < 1000)
  {
    motorRB.update();
    motorRF.update();
    motorLF.update();
    motorLB.update();
    motorLF.printValues();
    delay(1);
  }

  motorRF.setSpeed(0);
  motorRB.setSpeed(0);
  motorLF.setSpeed(0);
  motorLB.setSpeed(0);
  timeflag = millis();
  while (millis()-timeflag < 1000)
  {
    motorRB.update();
    motorRF.update();
    motorLF.update();
    motorLB.update();
    delay(1);
  }

}