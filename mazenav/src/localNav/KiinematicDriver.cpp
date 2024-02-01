#include "localNav/KinematicDriver.h"


KinematicDriver::KinematicDriver(communication::Communicator* globComm)
{
    this->globComm = globComm;
}


void KinematicDriver::setSpeeds()
{
    globComm->motors.setSpeeds(motorSpeeds);
}

void KinematicDriver::testComm()
{
    MotorControllers::MotorSpeeds speeds {};
    motorSpeeds = speeds;
    setSpeeds();
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    speeds.rf = 100;
    speeds.lf = 100;
    speeds.rb = 100;
    speeds.lb = 100;
    motorSpeeds = speeds;
    setSpeeds();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}