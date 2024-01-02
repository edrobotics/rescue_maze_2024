#include <Arduino.h>
#include "MotorController.h"
#include "Communicator.h"

//                     ENCA, ENCB, PWM, DIR, ENCRATIO, CPR
MotorController motorRF {3,     2,   7,   6,    46.85,  48};
MotorController motorLF {39,   38,  28,  29,    46.85,  48};
MotorController motorRB {1,     0,   5,   4,    46.85,  48};
MotorController motorLB {23,   22,   37, 36,    46.85,  48};

MotorController* motorControllers[] {&motorRF, &motorLF, &motorRB, &motorLB};
constexpr int MOTOR_NUM = 4;

Communicator communicator {1};

int rpmVals[4] {};


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
void communicationLoop();
void i2cTestLoop();

void loop()
{

  communicationLoop();
  for (int i=0;i<MOTOR_NUM;++i)
  {
    motorControllers[i]->update();
  }
  delay(1);

}

void communicationLoop()
{
  static long timeFlag = 0;

  if (communicator.check())
  {
    communicator.updateSettings();
    communicator.getRpmVals(rpmVals);
    for (int i=0;i<MOTOR_NUM;++i)
    {
      Serial.print("Motor ");Serial.print(i);Serial.print(" = ");Serial.print(rpmVals[i]);Serial.print("    ");
      motorControllers[i]->setSpeed(rpmVals[i]);
    }
    Serial.println("");
  }

  if (millis()-timeFlag>20)
  {
    for (int i=0;i<MOTOR_NUM;++i)
    {
      communicator.transDat.setRPM(i, motorControllers[i]->getOutputSpeed());
      // communicator.transDat.setRPM(i, 55);
    }

    // Some more updating of other things
    // ...
    // Done updating everything

    // Update the registers
    communicator.updateRegisters();
  }

}

void i2cTestLoop()
{
  
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