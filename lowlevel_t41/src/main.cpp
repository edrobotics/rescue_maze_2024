#include <Arduino.h>
#include "MotorController.h"
#include "Communicator.h"

//                     ENCA, ENCB, PWM, DIR, ENCRATIO, CPR
MotorController motorRF {3,     2,   7,   6,    46.85,  48};
MotorController motorLF {39,   38,  28,  29,    46.85,  48};
MotorController motorRB {1,     0,   5,   4,    46.85,  48};
MotorController motorLB {23,   22,   37, 36,    46.85,  48};

Communicator communicator {1};

void setup()
{
  Serial.begin(9600);
  communicator.init();
  pinMode(LED_BUILTIN, OUTPUT);

  motorRF.init();
  motorLF.init();
  motorLB.init();
  motorRB.init();
  motorRF.isReversed = true;
  motorRB.isReversed = true;
  delay(1000);
}

void motorTestLoop();
void i2cTestLoop();

void loop()
{
  // motorTestLoop();
  i2cTestLoop();

}

void i2cTestLoop()
{
  if (communicator.check())
  {
    communicator.recompute();
  }
  // digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  // digitalWrite(LED_BUILTIN, LOW);
  delay(150);

}

void motorTestLoop()
{
  static long timeflag;

  motorRF.setSpeed(150);
  motorRB.setSpeed(150);
  motorLF.setSpeed(150);
  motorLB.setSpeed(150);
  timeflag = millis();
  while (millis()-timeflag < 1000)
  {
    motorRB.update();
    motorRF.update();
    motorLF.update();
    motorLB.update();
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