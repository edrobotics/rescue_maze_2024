
#include "Robot.h"
Robot robot {};

void setup()
{
  robot.init();
}

void loop()
{
 robot.updateLoop();
}