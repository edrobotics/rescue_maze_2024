
#include "Robot.h"
Robot robot {};

void setup()
{
  robot.init();
  // Calibrate the motors if there is a need
  // robot.calibrateMotorPid();
}

void loop()
{
 robot.updateLoop();
}