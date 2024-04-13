
#include "Robot.h"
Robot robot {};

void setup()
{
  robot.init();
  // Calibrate the motors if there is a need
  // delay(3000);
  // robot.calibrateMotorPid();
}

void loop()
{
  // robot.testDrive();
  // delay(1000);
  robot.updateLoop();
}