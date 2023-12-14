#include <Arduino.h>
#include "MotorController.h"

//                     ENCA, ENCB, PWM, DIR, ENCRATIO, CPR
MotorController motorRF {14,   15,   5,   6,    46.85,  48};
MotorController motorLF {2,     1,   7,   8,    46.85,  48};
MotorController motorLB {3,     4,   9,  10,    46.85,  48};
MotorController motorRB {17,   16,  11,  12,    46.85,  48};

int pwmPin = 9;
int dirPin = 10;

void setup()
{
  motorRF.init();
  motorLF.init();
  motorLB.init();
  motorRB.init();
  motorLB.isReversed = true;
  // motorLB.isReversed = true;
  // pinMode(dirPin, OUTPUT);
  // pinMode(pwmPin, OUTPUT);
  // analogWriteFrequency(pwmPin, 80000);
  // delay(1000);
  delay(2000);

}

long timeflag;

void loop()
{
  // motorLB.setSpeed(100);
  // timeflag = millis();
  // while (millis()-timeflag < 1000)
  // {
  //   motorLB.update();
  //   delay(1);
  // }
  // motorLB.setSpeed(0);
  // timeflag = millis();
  // while (millis()-timeflag < 1000)
  // {
  //   motorLB.update();
  //   delay(1);
  // }


  // motorRF.setPWM(200);
  // motorRB.setPWM(200);
  // motorLB.setPWM(-200);
  motorLF.setPWM(256);
  delay(1000);
  // motorRB.setPWM(0);
  // motorRF.setPWM(0);
  // motorLB.setPWM(0);
  // motorLF.setPWM(0);
  // delay(1000);
  // motorLB.setPWM(200);
  // motorLF.setPWM(200);
  // delay(1000);
  // motorLB.setPWM(0);
  // motorLF.setPWM(0);
  // delay(1000);


}
